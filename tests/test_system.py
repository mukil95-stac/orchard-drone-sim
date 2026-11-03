from pathlib import Path
import os
os.environ.setdefault("MUJOCO_GL","egl")
import json
import numpy as np
import mujoco
from orchard_drone.core import (ADRC, EKF, Mission, Battery, MotorMixer, OrchardGraph,
                               TREE_POSITIONS, ORDER, ArucoVision)
from orchard_drone.simulator import MuJoCoSim

ROOT=Path(__file__).resolve().parents[1]

def test_mjcf_orchard_drone_tree_marker_counts():
    m=mujoco.MjModel.from_xml_path(str(ROOT/'models/orchard.xml'))
    assert sum(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,i).startswith('tree_') for i in range(m.nbody))==9
    assert sum((mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('marker_') for i in range(m.ngeom))==9
    # One free drone joint plus four propeller animation hinges.
    assert m.ncam==1 and m.njnt==5
    assert sum((mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_SENSOR,i) or '').startswith('lidar_') for i in range(m.nsensor))==16
    assert {int((mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or '')[-2:]) for i in range(m.ngeom) if (mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('marker_')}==set(range(1,10))

def test_generated_aruco_pngs_and_opencv_detector():
    import cv2
    for i in range(1,10): assert cv2.imread(str(ROOT/f'assets/aruco_{i:02d}.png')) is not None
    vision=ArucoVision([[320,0,128],[0,320,128],[0,0,1]])
    img=cv2.imread(str(ROOT/'assets/aruco_01.png'))
    img=cv2.copyMakeBorder(img,40,40,40,40,cv2.BORDER_CONSTANT,value=(255,255,255))
    detections=vision.detect(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
    assert detections and detections[0]['id']==1 and detections[0]['corners'].shape==(4,2)

def test_camera_imu_and_delayed_motor_rpm():
    sim=MuJoCoSim(); image=sim.image(); assert image.shape==(480,640,3)
    vision=ArucoVision([[313.7,0,320],[0,313.7,240],[0,0,1]])
    # From the launch pad the centerline camera faces marker 08 in the nearest row.
    assert any(d['id']==8 and d['corners'].shape==(4,2) for d in vision.detect(image))
    acc,gyro,q=sim.imu(); assert acc.shape==(3,) and gyro.shape==(3,) and q.shape==(4,)
    # MuJoCo uses -1 for a rangefinder beam with no surface hit.
    assert sim.lidar().shape==(16,) and np.all(sim.lidar()>=-1)
    sim.set_rpm([20000]*4); sim.step(); assert 0<sim.rpm[0]<20000; sim.renderer.close()

def test_mixer_allocation_and_adrc_response():
    mixer=MotorMixer(); rpm=mixer.mix(.055*9.81,0,0,0); assert np.max(np.abs(mixer.wrench(rpm)[1:]))<1e-8
    ctl=ADRC(); u=ctl.update([0,0,0],[0,0,0],[1,0,1],.01); assert u[0]>0 and u[2]>0

def test_ekf_prediction_and_aruco_measurement_correction():
    ekf=EKF(); x=ekf.x.copy(); ekf.predict([1,0,9.81],[0,0,0],.01); assert not np.allclose(ekf.x,x)
    before=abs(ekf.x[0]-(-7.)); assert ekf.correct_position([-7,4,.25]); assert abs(ekf.x[0]+7)<before
    assert not ekf.correct_position([40,40,40])

def test_serpentine_navigation_graph_and_energy():
    graph=OrchardGraph(); assert ORDER==[4,5,6,3,2,1,7,8,9]
    assert graph.path(1,9)[0]==1 and graph.path(1,9)[-1]==9 and len(graph.path(1,9))==5
    assert graph.path(1,9,'dijkstra')[0]==1 and graph.path(1,9,'dijkstra')[-1]==9
    assert len(graph.edges)==9 and len(graph.edges[5])==4
    battery=Battery(); p=battery.update([22000]*4,1); assert p>0 and battery.remaining<battery.capacity

def test_mission_resume_persistence_and_trajectory(tmp_path):
    f=tmp_path/'mission_state.json'; m=Mission(f); m.state='NAVIGATING'; m.complete(4); m.log(1,[1,2,3],[1.1,2.1,3.1]); m.save()
    restored=Mission(f); assert restored.done=={4} and restored.next_tree==5
    saved=json.loads(f.read_text()); assert saved['trajectory'][0]['ekf_x']==1.1
    m.complete_point(5,'A'); resumed=Mission(f); assert 'tree_05_view_A' in resumed.completed_points

def test_runner_navigation_does_not_teleport(tmp_path):
    from orchard_drone.runner import Simulation
    s=Simulation(); s.mission=Mission(tmp_path/'state.json'); s.mission.reset(); initial=s.sim.pose()[0].copy(); result=s.run(max_time=1,headless=True)
    assert result['time']>=.99 and result['state'] in {'TAKEOFF','NAVIGATING'}
    assert np.linalg.norm(s.sim.pose()[0]-initial)>0.01
