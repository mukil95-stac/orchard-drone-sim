from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
import heapq, json, math, time
import numpy as np

TREE_POSITIONS={i:np.array([((i-1)%3-1)*8., (1-(i-1)//3)*8., 0.]) for i in range(1,10)}
ORDER=[4,5,6,3,2,1,7,8,9]

# Motor order used by the MJCF model: front-left, rear-left, rear-right, front-right.
ROTOR_LAYOUT=np.array([[-.400,.400],[-.315,-.315],[.315,-.315],[.400,.400]],dtype=float)
ROTOR_YAW_SIGNS=np.array([1.,-1.,1.,-1.])

@dataclass
class DroneParams:
    mass:float=2.0; arm_length:float=.40; kf:float=6.5e-7; km:float=9.75e-9
    motor_time_constant:float=.045; maximum_rpm:float=30000.; minimum_rpm:float=0.
    @property
    def max_omega(self): return self.maximum_rpm*2*math.pi/60

class MotorMixer:
    """X-frame motor order: front-left CCW, rear-left CW, rear-right CCW, front-right CW."""
    def __init__(self,p=DroneParams()): self.p=p
    def mix(self,thrust,roll,pitch,yaw):
        b=self.p.kf; d=self.p.km
        positions=ROTOR_LAYOUT*(self.p.arm_length/.400)
        # Torque signs follow tau = r x F for upward thrust on the model's body +Z axis.
        a=np.vstack((np.full(4,b),positions[:,1]*b,-positions[:,0]*b,ROTOR_YAW_SIGNS*d))
        sq=np.linalg.solve(a,np.array([thrust,roll,pitch,yaw],float))
        return np.sqrt(np.clip(sq,0,self.p.max_omega**2))*60/(2*np.pi)
    def wrench(self,rpm):
        w=np.asarray(rpm)*2*np.pi/60; forces=self.p.kf*w*w
        positions=ROTOR_LAYOUT*(self.p.arm_length/.400)
        roll_torque=np.sum(positions[:,1]*forces)
        pitch_torque=np.sum(-positions[:,0]*forces)
        yaw_torque=self.p.km*np.sum(ROTOR_YAW_SIGNS*w*w)
        return np.array([forces.sum(),roll_torque,pitch_torque,yaw_torque])

class ADRC:
    """Per-axis second-order linear ADRC: tracking setpoint, ESO disturbance state, nonlinear saturated feedback."""
    def __init__(self,bw=1.7,obs=5.,vel_gain=2.2,max_acc=2.5):
        self.bw=bw; self.obs=obs; self.vel_gain=vel_gain; self.max_acc=max_acc
        self.z1=np.zeros(3); self.z2=np.zeros(3); self.z3=np.zeros(3); self.initialized=False
    def update(self,pos,vel,target,dt, measured_accel=None):
        p=np.asarray(pos,dtype=float); v=np.asarray(vel,dtype=float); r=np.asarray(target,dtype=float)
        if not self.initialized: self.z1=p.copy(); self.z2=v.copy(); self.initialized=True
        e=p-self.z1; e3=self.z3-(np.asarray(measured_accel) if measured_accel is not None else np.zeros(3))
        b1=3*self.obs; b2=3*self.obs**2; b3=self.obs**3
        self.z1+=dt*(self.z2+b1*e); self.z2+=dt*(self.z3+b2*e); self.z3+=dt*(b3*e)
        err=r-self.z1
        # smooth nonlinear position feedback plus velocity damping and lumped disturbance rejection
        u=self.bw**2*np.tanh(err)+self.vel_gain*(np.zeros(3)-self.z2)-self.z3
        return np.clip(u,-self.max_acc,self.max_acc)

class EKF:
    """15-state error-state approximation: xyz, velocity, accelerometer bias, roll/pitch/yaw."""
    def __init__(self,accel_variance=.04,position_variance=.04,max_visual_innovation=1.8):
        self.accel_variance=accel_variance; self.position_variance=position_variance; self.max_visual_innovation=max_visual_innovation
        self.x=np.zeros(15); self.x[:3]=[-8.,4.,.25]; self.P=np.eye(15)*.1
    def predict(self,accel_body,gyro,dt):
        roll,pitch,yaw=self.x[9:12]
        cr,sr,cp,sp,cy,sy=np.cos(roll),np.sin(roll),np.cos(pitch),np.sin(pitch),np.cos(yaw),np.sin(yaw)
        R=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
        aw=R@(np.asarray(accel_body)-self.x[12:15])+[0,0,-9.81]
        self.x[:3]+=self.x[3:6]*dt+.5*aw*dt*dt; self.x[3:6]+=aw*dt
        self.x[9:12]+=np.asarray(gyro)*dt
        F=np.eye(15); F[:3,3:6]=np.eye(3)*dt; F[3:6,12:15]=-R*dt
        self.P=F@self.P@F.T+np.eye(15)*dt*self.accel_variance*.2
    def correct_position(self,z,var=None):
        var=self.position_variance if var is None else var
        H=np.zeros((3,15)); H[:,:3]=np.eye(3); R=np.eye(3)*var
        innovation=np.asarray(z)-H@self.x
        if np.linalg.norm(innovation)>self.max_visual_innovation: return False
        K=self.P@H.T@np.linalg.inv(H@self.P@H.T+R); self.x+=K@(np.asarray(z)-H@self.x); self.P=(np.eye(15)-K@H)@self.P; return True

class OrchardGraph:
    def __init__(self):
        self.edges={i:[] for i in TREE_POSITIONS}
        for i,p in TREE_POSITIONS.items():
            for j,q in TREE_POSITIONS.items():
                if i<j and np.linalg.norm(p[:2]-q[:2])==8:
                    d=float(np.linalg.norm(p[:2]-q[:2])); self.edges[i].append((j,d,d*1.4,d*.9)); self.edges[j].append((i,d,d*1.4,d*.9))
    def path(self,start,goal,method="astar",weights=(1.,.1,.05)):
        q=[(0.,start)]; prev={}; cost={start:0.}
        while q:
            _,u=heapq.heappop(q)
            if u==goal: break
            for v,d,e,s in self.edges[u]:
                nc=cost[u]+weights[0]*d+weights[1]*e+weights[2]*s
                if nc<cost.get(v,float("inf")):
                    cost[v]=nc; prev[v]=u
                    h=0 if method.lower()=="dijkstra" else weights[0]*np.linalg.norm(TREE_POSITIONS[v][:2]-TREE_POSITIONS[goal][:2])
                    heapq.heappush(q,(nc+h,v))
        if goal not in cost: return []
        out=[goal]
        while out[-1]!=start: out.append(prev[out[-1]])
        return out[::-1]
    def safe_reachable(self,node,energy_wh,reserve_wh=.32):
        reachable=[]; q=[(0.,node)]; best={node:0.}
        while q:
            cost,u=heapq.heappop(q)
            if cost!=best[u]: continue
            for v,d,e,s in self.edges[u]:
                nc=cost+e/3600
                if nc<best.get(v,float('inf')) and nc+reserve_wh<=energy_wh: best[v]=nc; heapq.heappush(q,(nc,v))
        return max(best,key=lambda i: (best[i],-i))

class Battery:
    def __init__(self,capacity_wh=2.2,coefficient=1.e-10): self.capacity=capacity_wh*3600; self.remaining=self.capacity; self.k=coefficient
    def update(self,rpm,dt):
        w=np.asarray(rpm)*2*np.pi/60; power=8.+self.k*np.sum(w**3)
        self.remaining=max(0.,self.remaining-power*dt); return power
    @property
    def percent(self): return 100*self.remaining/self.capacity

class Mission:
    def __init__(self,path,order=ORDER):
        self.path=Path(path); self.order=list(order); self.done=set(); self.completed_points=set(); self.state='IDLE'; self.current_tree=None; self.in_progress_tree=None; self.trajectory=[]
        self.load()
    def load(self):
        if self.path.exists():
            try:
                saved=json.loads(self.path.read_text()); self.done=set(saved.get('completed_trees',[])); self.completed_points=set(saved.get('completed_points',[])); self.current_tree=saved.get('last_completed_tree'); self.in_progress_tree=saved.get('in_progress_tree')
                self.trajectory=saved.get('trajectory',[])
            except (ValueError,OSError): self.done=set(); self.completed_points=set()
    @property
    def next_tree(self): return next((i for i in self.order if i not in self.done),None)
    def reset(self):
        self.done.clear(); self.completed_points.clear(); self.current_tree=None; self.in_progress_tree=None; self.trajectory=[]; self.state='IDLE'; self.save()
    def complete_point(self,i,label):
        self.in_progress_tree=int(i); self.completed_points.add(f'tree_{int(i):02d}_view_{label}'); self.save()
    def complete(self,i):
        self.done.add(int(i)); self.current_tree=int(i); self.in_progress_tree=None; self.state='TRANSITIONING'; self.save()
    def log(self,t,true,estimated):
        self.trajectory.append({'timestamp':float(t),'x':float(true[0]),'y':float(true[1]),'z':float(true[2]),'ekf_x':float(estimated[0]),'ekf_y':float(estimated[1]),'ekf_z':float(estimated[2])})
    def save(self):
        self.path.parent.mkdir(parents=True,exist_ok=True); self.path.write_text(json.dumps({'completed_trees':sorted(self.done),'completed_points':sorted(self.completed_points),'last_completed_tree':self.current_tree,'in_progress_tree':self.in_progress_tree,'next_tree':self.next_tree,'trajectory':self.trajectory,'timestamp':time.time()},indent=2))

class ArucoVision:
    """Detect image markers and estimate marker-to-camera pose using OpenCV PnP."""
    def __init__(self, camera_matrix, marker_size=.24):
        import cv2
        if not hasattr(cv2,'aruco'): raise RuntimeError('Install opencv-contrib-python for ArUco support')
        self.cv2=cv2; self.K=np.asarray(camera_matrix,float); self.size=marker_size
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); self.params=cv2.aruco.DetectorParameters(); self.detector=cv2.aruco.ArucoDetector(self.dictionary,self.params)
        h=marker_size/2; self.obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
    def detect(self,image):
        gray=self.cv2.cvtColor(image,self.cv2.COLOR_RGB2GRAY); corners,ids,_=self.detector.detectMarkers(gray)
        out=[]
        if ids is not None:
            for c,i in zip(corners,ids.flatten()):
                ok,r,t=self.cv2.solvePnP(self.obj,c.reshape(-1,2),self.K,np.zeros(5),flags=self.cv2.SOLVEPNP_IPPE_SQUARE)
                if ok: out.append({'id':int(i),'corners':c.reshape(-1,2),'rvec':r.reshape(3),'tvec':t.reshape(3)})
        return out
    def world_position(self,detection,camera_position,camera_rotation,marker_world):
        return np.asarray(marker_world)-np.asarray(camera_rotation)@detection['tvec']

class AppleVision:
    """Find red apple-shaped regions in an RGB camera image."""
    def __init__(self,min_area=20):
        import cv2
        self.cv2=cv2; self.min_area=min_area

    def detect(self,image):
        hsv=self.cv2.cvtColor(image,self.cv2.COLOR_RGB2HSV)
        low=self.cv2.inRange(hsv,(0,90,45),(12,255,255))
        high=self.cv2.inRange(hsv,(170,90,45),(180,255,255))
        mask=self.cv2.bitwise_or(low,high)
        mask=self.cv2.morphologyEx(mask,self.cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        contours,_=self.cv2.findContours(mask,self.cv2.RETR_EXTERNAL,self.cv2.CHAIN_APPROX_SIMPLE)
        detections=[]
        for contour in contours:
            area=float(self.cv2.contourArea(contour))
            if area<self.min_area: continue
            perimeter=float(self.cv2.arcLength(contour,True))
            if perimeter<=0: continue
            circularity=4.*math.pi*area/(perimeter*perimeter)
            x,y,w,h=self.cv2.boundingRect(contour)
            aspect=w/max(1,h)
            if circularity<.35 or not .55<=aspect<=1.8: continue
            moments=self.cv2.moments(contour)
            if moments['m00']<=0: continue
            detections.append({'bbox':[int(x),int(y),int(w),int(h)],
                               'center':[float(moments['m10']/moments['m00']),float(moments['m01']/moments['m00'])],
                               'area':area,'circularity':float(circularity)})
        return sorted(detections,key=lambda d:d['area'],reverse=True)
