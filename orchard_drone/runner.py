from pathlib import Path
import argparse, json, math, random, yaml
import numpy as np
from scipy.spatial.transform import Rotation
from .core import ADRC, EKF, Mission, Battery, MotorMixer, OrchardGraph, DroneParams, TREE_POSITIONS, ORDER, AppleVision
from .armed_controller import ArmedADRCController, lidar_avoidance_acceleration
from .simulator import MuJoCoSim

class Simulation:
    def __init__(self,root=None):
        self.root=Path(root or Path(__file__).resolve().parents[1])
        read=lambda name: yaml.safe_load((self.root/'config'/name).read_text())
        dc=read('drone.yaml'); self.params=DroneParams(**{k:dc[k] for k in DroneParams.__dataclass_fields__ if k in dc})
        self.mission_config=read('mission.yaml'); ac=read('adrc.yaml'); armed_tuning=read('armed_controller.yaml'); bc=read('battery.yaml')
        self.sim=MuJoCoSim(self.root/'models/orchard.xml',params=self.params,wind_force=dc.get('wind_force',.008),accel_noise=dc.get('sensor_noise_accel',.08),gyro_noise=dc.get('sensor_noise_gyro',.008))
        ec=read('ekf.yaml'); self.ekf=EKF(ec['imu_accel_variance'],ec['aruco_position_variance'],ec.get('max_visual_innovation',4.0)); self.ekf.x[:3]=[0.,-13.,.25]; self.adrc=ADRC(bw=ac['position_bandwidth'],obs=ac['observer_bandwidth'],vel_gain=ac['velocity_gain'],max_acc=ac['max_acceleration']); self.mixer=MotorMixer(self.sim.params)
        self.armed_controller=ArmedADRCController(self.sim.params,armed_tuning)
        self.marker_standoff=armed_tuning['marker_standoff']; self.marker_height_offset=armed_tuning['marker_height_offset']; self.arrival_tolerance=armed_tuning['arrival_tolerance']
        self.marker_lock_max_range=armed_tuning['marker_lock_max_range']
        self.marker_capture_staging_radius=armed_tuning['marker_capture_staging_radius']
        self.marker_capture_speed=armed_tuning['marker_capture_speed']
        self.arrival_velocity_tolerance=armed_tuning['arrival_velocity_tolerance']; self.takeoff_altitude=armed_tuning['takeoff_altitude']; self.takeoff_tolerance=armed_tuning['takeoff_tolerance']; self.takeoff_velocity_tolerance=armed_tuning['takeoff_velocity_tolerance']
        mission_path=self.root/'data/mission_state.json'; self.mission=Mission(mission_path,self.mission_config.get('inspection_order',ORDER))
        self.battery=Battery(bc['capacity_wh'],bc['power_coefficient']); self.reserve_wh=bc['reserve_wh']; self.graph=OrchardGraph()
        self.view_offset=self.mission_config['view_offset']; self.altitude=self.mission_config['inspection_altitude']; self.inspection_dwell=self.mission_config['inspection_dwell']
        self.inspected_images=[]; self.true_path=[]; self.estimated_path=[]; self.last_detected=None; self.last_detection=None; self.last_visual_time=-1; self.target=None
        self.view_stage=1 if self.mission.next_tree is not None and f'tree_{self.mission.next_tree:02d}_view_A' in self.mission.completed_points else 0
        self.inspect_started=None; self.last_path_log=-1; self.last_save=0.; self.returning=False; self.transit_queue=[]

    def print_scene_poses(self):
        """Print the world-frame poses of every tree and the helipad."""
        import mujoco
        fmt=lambda values: '['+', '.join(f'{value:.3f}' for value in values)+']'
        print('Scene poses (world frame; position in metres; quaternion w,x,y,z):')
        for tree in range(1,10):
            body=mujoco.mj_name2id(self.sim.model,mujoco.mjtObj.mjOBJ_BODY,f'tree_{tree:02d}')
            print(f'  tree_{tree:02d}: position={fmt(self.sim.data.xpos[body])}, orientation={fmt(self.sim.data.xquat[body])}')
        geom=mujoco.mj_name2id(self.sim.model,mujoco.mjtObj.mjOBJ_GEOM,'helipad')
        rotation=Rotation.from_matrix(self.sim.data.geom_xmat[geom].reshape(3,3)).as_quat()
        quat=np.array([rotation[3],rotation[0],rotation[1],rotation[2]])
        print(f'  helipad: position={fmt(self.sim.data.geom_xpos[geom])}, orientation={fmt(quat)}')

    def run_manual(self,max_time=240):
        """Arm to fly an ADRC-controlled approach to the next orchard marker."""
        import threading, time
        import mujoco
        import mujoco.viewer
        import cv2

        sim=self.sim; target=self.ekf.x[:3].copy(); armed=False; command_lock=threading.Lock()
        manual_marker_world=None; final_setpoint=None; phase='DISARMED'; takeoff_target=None
        yaw_target=float(self.ekf.x[11])
        remaining=[tree for tree in ORDER if tree not in self.mission.done]
        first_tree=random.choice(remaining or ORDER)
        staging_target=self.viewpoint(first_tree,'A')
        visited_trees=set()
        hover_until=0.0
        print(f'X arm/disarm | vertical takeoff, then approach tree {first_tree:02d} | Esc quit')
        def on_key(key):
            nonlocal armed, target, manual_marker_world, final_setpoint, phase, takeoff_target, yaw_target
            with command_lock:
                if key in (ord('X'),ord('x')):
                    armed=not armed
                    manual_marker_world=None; final_setpoint=None
                    if armed:
                        self.armed_controller.reset()
                        yaw_target=float(self.ekf.x[11])
                        takeoff_target=self.ekf.x[:3].copy(); takeoff_target[2]=self.takeoff_altitude
                        target=takeoff_target.copy(); phase='TAKEOFF'
                        print('Drone armed: vertical takeoff')
                    else:
                        target=self.ekf.x[:3].copy(); phase='DISARMED'
                        print('Drone disarmed')

        viewer=mujoco.viewer.launch_passive(sim.model,sim.data,key_callback=on_key)
        viewer.cam.type=mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid=sim.body
        viewer.cam.lookat[:]=target; viewer.cam.distance=7.; viewer.cam.azimuth=135; viewer.cam.elevation=-20
        camera_window='Drone front camera'
        cv2.namedWindow(camera_window,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(camera_window,640,480)
        power=0.; next_vision_time=0.; next_camera_time=0.; next_pwm_time=0.; camera_image=None
        try:
            while sim.data.time<max_time and viewer.is_running():
                cycle_start=time.perf_counter(); dt=sim.dt
                accel,gyro,_=sim.imu(); self.ekf.predict(accel,gyro,dt)
                with command_lock:
                    armed_now=armed; target_now=target.copy(); yaw_now=yaw_target
                if armed_now:
                    motor_rpm=self.armed_controller.update(
                        self.ekf.x[:3],self.ekf.x[3:6],self.ekf.x[9:12],gyro,target_now,dt,yaw_now,sim.lidar()
                    )
                    sim.set_rpm(motor_rpm)
                else:
                    sim.set_rpm(np.zeros(4))
                sim.set_propeller_visual_rpm(None)
                sim.step(); power=self.battery.update(sim.rpm,dt); self.target=target_now.copy()
                if sim.data.time>=next_pwm_time:
                    label=f'ARMED — {phase}' if armed_now else 'DISARMED'
                    print(f'\r[{label}]',end='',flush=True)
                    next_pwm_time=sim.data.time+.25
                if sim.data.time>=next_camera_time:
                    camera_image=sim.image()
                    cv2.imshow(camera_window,cv2.cvtColor(camera_image,cv2.COLOR_RGB2BGR))
                    cv2.waitKey(1)
                    next_camera_time=sim.data.time+1./15.
                if sim.data.time>=next_vision_time:
                    observation=self._update_visual_position(sim,camera_image,target_marker_id=first_tree)
                    with command_lock:
                        capture_marker=(armed and phase=='APPROACH' and manual_marker_world is None
                                        and np.linalg.norm(self.ekf.x[:3]-staging_target)<=self.marker_capture_staging_radius
                                        and np.linalg.norm(self.ekf.x[3:6])<=self.marker_capture_speed)
                    if capture_marker and observation is not None:
                        marker_world=sim.marker_positions[first_tree]
                        with command_lock:
                            if armed and manual_marker_world is None:
                                manual_marker_world=marker_world.copy()
                                final_setpoint=marker_world+np.array([0.,-self.marker_standoff,self.marker_height_offset])
                                target=final_setpoint.copy(); phase='FINAL_APPROACH'
                                arrival_target=final_setpoint.copy()
                            else:
                                arrival_target=None
                        if arrival_target is not None:
                            print(f'\nArUco {first_tree:02d} world position {marker_world.round(2)}; navigating to {arrival_target.round(2)}')
                    next_vision_time=sim.data.time+.2
                with command_lock:
                    if armed and phase=='TAKEOFF':
                        takeoff_xy_error=np.linalg.norm(self.ekf.x[:2]-takeoff_target[:2])
                        takeoff_xy_speed=np.linalg.norm(self.ekf.x[3:5])
                        if (abs(self.ekf.x[2]-takeoff_target[2])<=self.takeoff_tolerance
                                and takeoff_xy_error<=max(.75,self.takeoff_tolerance)
                                and abs(self.ekf.x[5])<=self.takeoff_velocity_tolerance
                                and takeoff_xy_speed<=.6):
                            target=staging_target.copy(); phase='APPROACH'
                            print(f'\nTakeoff stable; moving forward to tree {first_tree:02d}')
                    if armed and phase=='HOVER' and sim.data.time>=hover_until:
                        candidates=[tree for tree in ORDER if tree not in visited_trees and tree not in self.mission.done]
                        if candidates:
                            first_tree=random.choice(candidates)
                            staging_target=self.viewpoint(first_tree,'A')
                            manual_marker_world=None; final_setpoint=None
                            target=staging_target.copy(); phase='APPROACH'
                            print(f'\n5-second hover complete; moving to next tree {first_tree:02d}')
                        else:
                            armed=False; target=self.ekf.x[:3].copy(); phase='COMPLETE'
                            print('\nAll selected trees visited; disarming')
                    reached_setpoint=armed and phase=='FINAL_APPROACH' and np.linalg.norm(self.ekf.x[:3]-final_setpoint)<=self.arrival_tolerance and np.linalg.norm(self.ekf.x[3:6])<=self.arrival_velocity_tolerance
                    if reached_setpoint:
                        visited_trees.add(first_tree)
                        target=self.ekf.x[:3].copy(); phase='HOVER'; hover_until=sim.data.time+5.0
                        print(f'\nReached tree {first_tree:02d}; hovering for 5 seconds')
                if reached_setpoint:
                    print(f'Reached marker setpoint within {self.arrival_tolerance:.1f} m at low speed; scanning')
                    self.scan_apple(sim,sim.image(),first_tree)
                viewer.sync()
                time.sleep(max(0.,dt-(time.perf_counter()-cycle_start)))
            return {'state':'MANUAL_STOPPED','time':sim.data.time,'battery_percent':self.battery.percent,'estimated_position':self.ekf.x[:3].tolist(),'power_w':power}
        finally:
            print()
            sim.set_rpm(np.zeros(4)); sim.set_propeller_visual_rpm(None); viewer.close()
            cv2.destroyWindow(camera_window)

    def scan_apple(self,sim,image,tree):
        """Detect and save visible apples after arriving at the target tree."""
        import cv2
        if not hasattr(self,'apple_vision'): self.apple_vision=AppleVision()
        detections=self.apple_vision.detect(image)
        annotated=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        for detection in detections:
            x,y,w,h=detection['bbox']
            cv2.rectangle(annotated,(x,y),(x+w,y+h),(40,220,40),2)
            cv2.putText(annotated,'apple',(x,max(14,y-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,(40,220,40),1,cv2.LINE_AA)
        output=self.root/'data'/'scans'; output.mkdir(parents=True,exist_ok=True)
        image_path=output/f'tree_{tree:02d}_apple_scan.png'
        report_path=output/f'tree_{tree:02d}_apple_scan.json'
        if not cv2.imwrite(str(image_path),annotated): raise OSError(f'cannot write apple scan image: {image_path}')
        report={'tree_id':int(tree),'simulation_time':float(sim.data.time),'apple_detected':bool(detections),
                'detections':detections,'image':str(image_path.relative_to(self.root))}
        report_path.write_text(json.dumps(report,indent=2))
        if detections:
            print(f"Apple scan: found {len(detections)} red fruit candidate(s); saved {image_path}")
        else:
            print(f'Apple scan: no apple detected in frame; saved {image_path}')
        return report

    def _update_visual_position(self,sim,image=None,target_marker_id=None):
        """Correct inertial position drift from a surveyed orchard marker observation."""
        try:
            from .core import ArucoVision
            if not hasattr(self,'vision'):
                self.vision=ArucoVision([[313.7,0,320],[0,313.7,240],[0,0,1]],marker_size=.24)
            detections=self.vision.detect(sim.image() if image is None else image)
            detections=[item for item in detections
                        if np.linalg.norm(item['tvec'])<=self.marker_lock_max_range]
            if not detections: return
            detection=min(detections,key=lambda item:np.linalg.norm(item['tvec']))
            marker_id=detection['id']
            if marker_id not in sim.marker_positions: return
            body_position=self.ekf.x[:3]
            body_rotation=Rotation.from_euler('xyz',self.ekf.x[9:12]).as_matrix()
            camera_offset=np.array([0.,.260,.020])
            camera_rotation=body_rotation@Rotation.from_euler('x',90,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
            camera_prior=body_position+body_rotation@camera_offset
            camera_position=self.vision.world_position(detection,camera_prior,camera_rotation,sim.marker_positions[marker_id])
            body_estimate=camera_position-body_rotation@camera_offset
            if self.ekf.correct_position(body_estimate):
                self.last_detected=marker_id; self.last_detection=detection
            if target_marker_id is not None:
                target_detections=[item for item in detections
                                   if item['id']==target_marker_id
                                   and np.linalg.norm(item['tvec'])<=self.marker_lock_max_range]
                if target_detections:
                    detection=min(target_detections,key=lambda item:np.linalg.norm(item['tvec']))
                    body_position=self.ekf.x[:3]
                    body_rotation=Rotation.from_euler('xyz',self.ekf.x[9:12]).as_matrix()
                    camera_position=body_position+body_rotation@camera_offset
                    camera_rotation=body_rotation@Rotation.from_euler('x',90,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
                    marker_world=camera_position+camera_rotation@detection['tvec']
                    return {'marker_id':target_marker_id,'world_position':marker_world}
            return None
        except RuntimeError:
            # OpenCV without the contrib ArUco module: keep the inertial estimate.
            return None
    def transit_points(self,previous,tree,final_target):
        if previous is None: return []
        old=TREE_POSITIONS[previous]; new=TREE_POSITIONS[tree]
        if abs(old[0]-new[0])<.01 and abs(old[1]-new[1])>7.9:
            side=(1. if old[0]>=0 else -1.)*(abs(old[0])+4.)
            return [np.array([side,self.target[1],self.altitude]),np.array([side,final_target[1],self.altitude])]
        return []
    def viewpoint(self,tree,view='A'):
        p=TREE_POSITIONS[tree].copy(); p[1]-=self.view_offset; p[0]+=.45 if view=='B' else 0.; p[2]=self.altitude; return p
    def run(self,max_time=240,headless=True,show_map=False):
        sim=self.sim
        if self.mission.next_tree is None: self.mission.state='MISSION_COMPLETE'; self.mission.save(); return {'state':self.mission.state,'completed':sorted(self.mission.done),'time':0.0,'battery_percent':self.battery.percent}
        tree=self.mission.next_tree; target=self.viewpoint(tree,'B' if self.view_stage else 'A'); self.target=target.copy(); self.mission.state='TAKEOFF'
        if not headless:
            import mujoco.viewer
            viewer=mujoco.viewer.launch_passive(sim.model,sim.data)
            # The default MuJoCo free camera is centered on the world origin,
            # while the drone starts at the edge of the orchard. Frame it on
            # the aircraft so it is visible as soon as the viewer opens.
            viewer.cam.lookat[:] = sim.pose()[0]
            viewer.cam.distance = 2.5
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
        else: viewer=None
        try:
            while sim.data.time<max_time and self.mission.state not in ('MISSION_COMPLETE','EMERGENCY'):
                dt=sim.dt; accel,gyro,_=sim.imu(); self.ekf.predict(accel,gyro,dt)
                p,v=sim.pose()[0],sim.velocity(); epos=self.ekf.x[:3]; evel=self.ekf.x[3:6]
                # ADRC outer loop returns acceleration; attitude/wrench allocation tracks tilt and yaw.
                a=self.adrc.update(epos,evel,target,dt)
                a += lidar_avoidance_acceleration(sim.lidar(), self.ekf.x[11])
                roll,pitch,yaw=self.ekf.x[9:12]
                desired_roll=np.clip(-a[1]/9.81,-.4,.4); desired_pitch=np.clip(a[0]/9.81,-.4,.4)
                thrust=sim.params.mass*(9.81+a[2])/max(.75,math.cos(roll)*math.cos(pitch))
                torques=np.array([.0018*(desired_roll-roll)-.00035*gyro[0],.0018*(desired_pitch-pitch)-.00035*gyro[1],.0008*(-yaw)-.0002*gyro[2]])
                sim.set_rpm(self.mixer.mix(thrust,*torques))
                sim.step(); power=self.battery.update(sim.rpm,dt)
                if not headless and viewer is not None and not viewer.is_running(): break
                if show_map and int(sim.data.time*10)%2==0: self.draw_map()
                if int(sim.data.time*10)>self.last_path_log:
                    self.last_path_log=int(sim.data.time*10); self.true_path.append(p.copy()); self.estimated_path.append(epos.copy()); self.mission.log(sim.data.time,p,epos)
                if sim.data.time-self.last_save>=1.: self.mission.save(); self.last_save=sim.data.time
                if self.mission.state=='TAKEOFF' and epos[2]>.8: self.mission.state='NAVIGATING'
                # Process the actual rendered RGB image at 5 Hz; visual pose update is only from detected corner PnP.
                if int(sim.data.time*5)>self.last_visual_time:
                    self.last_visual_time=int(sim.data.time*5)
                    self._update_visual_position(sim)
                if self.mission.next_tree is None:
                    self.mission.state='MISSION_COMPLETE'; break
                if np.linalg.norm(epos-target)<.60 and np.linalg.norm(evel)<.35 and not self.returning:
                    transit_advanced=bool(self.transit_queue)
                    if transit_advanced:
                        target=self.transit_queue.pop(0); self.target=target.copy(); self.inspect_started=None
                    tree=self.mission.next_tree
                    if not transit_advanced and tree is not None and self.last_detected==tree:
                        if self.inspect_started is None: self.inspect_started=sim.data.time; self.mission.state='INSPECTING'
                        if sim.data.time-self.inspect_started>=self.inspection_dwell:
                            self.inspected_images.append(sim.image()); self.save_inspection(tree,'A' if self.view_stage==0 else 'B')
                            self.scan_apple(sim,self.inspected_images[-1],tree)
                            if self.view_stage==0:
                                self.mission.complete_point(tree,'A'); self.view_stage=1; target=self.viewpoint(tree,'B'); self.target=target.copy(); self.inspect_started=None; self.last_detected=None; self.last_detection=None; self.mission.state='TRANSITIONING'
                            else:
                                self.mission.complete_point(tree,'B'); self.mission.complete(tree); tree=self.mission.next_tree
                                if tree is None: self.mission.state='MISSION_COMPLETE'; break
                                self.view_stage=0; target=self.viewpoint(tree,'A'); self.transit_queue=self.transit_points(self.mission.current_tree,tree,target);
                                if self.transit_queue: target=self.transit_queue.pop(0)
                                self.target=target.copy(); self.inspect_started=None; self.last_detected=None; self.last_detection=None; self.mission.state='NAVIGATING'
                    else: self.inspect_started=None
                # Estimate the next leg from EKF distance and include a route to a safe graph node plus reserve.
                if self.mission.next_tree is not None and not self.returning:
                    tree=self.mission.next_tree; energy_to_next=np.linalg.norm(self.viewpoint(tree)-epos)*.004
                    safe_edge=min(edge[2]/3600 for edge in self.graph.edges[tree]) if self.graph.edges[tree] else 0.
                    if self.battery.remaining/3600 < energy_to_next+safe_edge+self.reserve_wh:
                        self.mission.state='LOW_ENERGY'; safe=self.graph.safe_reachable(tree,self.battery.remaining/3600,self.reserve_wh)
                        target=self.viewpoint(safe); self.target=target.copy(); self.returning=True; self.mission.state='RETURNING'
                elif self.returning and np.linalg.norm(epos-target)<.60 and np.linalg.norm(evel)<.35:
                    target=target.copy(); target[2]=.22; self.target=target.copy(); sim.set_rpm(np.zeros(4))
                    if epos[2]<.08: self.mission.state='EMERGENCY'; break
            self.mission.save(); return {'state':self.mission.state,'completed':sorted(self.mission.done),'time':sim.data.time,'battery_percent':self.battery.percent,'last_marker':self.last_detected,'true_position':sim.pose()[0].tolist(),'estimated_position':self.ekf.x[:3].tolist(),'localization_error':float(np.linalg.norm(sim.pose()[0]-self.ekf.x[:3])),'power_w':power}
        finally:
            if viewer is not None: viewer.close()
            if sim.renderer is not None: sim.renderer.close(); sim.renderer=None
    def save_inspection(self,tree,view):
        import cv2
        folder=self.root/'data/inspections'; folder.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(folder/f'tree_{tree:02d}_view_{view}.png'),cv2.cvtColor(self.inspected_images[-1],cv2.COLOR_RGB2BGR))
    def draw_map(self):
        import matplotlib.pyplot as plt
        plt.ion(); fig=plt.figure(1); plt.clf()
        for i,p in TREE_POSITIONS.items(): plt.scatter(p[0],p[1],c='green' if i in self.mission.done else 'red'); plt.text(p[0],p[1],f'{i:02d}')
        if self.true_path: plt.plot(np.array(self.true_path)[:,0],np.array(self.true_path)[:,1],label='MuJoCo true')
        if self.estimated_path: plt.plot(np.array(self.estimated_path)[:,0],np.array(self.estimated_path)[:,1],label='EKF estimate')
        if self.target is not None: plt.scatter(self.target[0],self.target[1],c='blue',marker='x',s=80,label='planned waypoint')
        if self.true_path: plt.scatter(self.true_path[-1][0],self.true_path[-1][1],c='orange',marker='o',label='current')
        plt.axis('equal'); plt.grid(); plt.legend(); plt.pause(.001)

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--max-time',type=float,default=240); ap.add_argument('--headless',action='store_true'); ap.add_argument('--map',action='store_true'); ap.add_argument('--reset-mission',action='store_true'); args=ap.parse_args()
    simulation=Simulation()
    if args.reset_mission: simulation.mission.reset()
    print(simulation.run(args.max_time,args.headless,args.map))
