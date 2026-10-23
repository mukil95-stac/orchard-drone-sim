from pathlib import Path
import math
import os
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from .core import DroneParams, MotorMixer, ROTOR_LAYOUT, ROTOR_YAW_SIGNS

class MuJoCoSim:
    def __init__(self,xml=None,params=None,seed=5,wind_force=.008,accel_noise=.08,gyro_noise=.008):
        self.xml=Path(xml or Path(__file__).resolve().parents[1]/'models/orchard.xml'); self.model=mujoco.MjModel.from_xml_path(str(self.xml)); self.data=mujoco.MjData(self.model)
        self.params=params or DroneParams(); self.mixer=MotorMixer(self.params); self.rng=np.random.default_rng(seed); self.wind=wind_force; self.accel_noise=accel_noise; self.gyro_noise=gyro_noise
        self.rpm=np.zeros(4); self.command=np.zeros(4); self.visual_rpm=None; self.renderer=None; self.last_image=None
        self.body=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,'whoop'); self.qadr=self.model.body_jntadr[self.body]; self.dadr=self.model.body_dofadr[self.body]
        self.propeller_dofs=[self.model.jnt_dofadr[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,f'propeller_spin_{i}')] for i in range(1,5)]
        self.propeller_visual_speed_scale=.025
        self.camera=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_CAMERA,'front_camera')
        mujoco.mj_forward(self.model,self.data)
        self.marker_positions={i:np.array([((i-1)%3-1)*8.+1.8,(1-(i-1)//3)*8.-.670,.95]) for i in range(1,10)}
    @property
    def dt(self): return self.model.opt.timestep
    def pose(self): return self.data.xpos[self.body].copy(), self.data.xquat[self.body].copy()
    def velocity(self): return self.data.qvel[self.dadr:self.dadr+3].copy()
    def set_rpm(self,rpm): self.command=np.clip(np.asarray(rpm),0,self.params.maximum_rpm)
    def set_propeller_visual_rpm(self,rpm): self.visual_rpm=None if rpm is None else np.broadcast_to(np.asarray(rpm,dtype=float),(4,)).copy()
    def step(self):
        a=1-math.exp(-self.dt/self.params.motor_time_constant); self.rpm+=a*(self.command-self.rpm)
        forces=self.params.kf*(self.rpm*2*math.pi/60)**2
        q=self.data.xquat[self.body]; R=np.zeros(9); mujoco.mju_quat2Mat(R,q); R=R.reshape(3,3)
        positions=ROTOR_LAYOUT*(self.params.arm_length/.400)
        for i,f in enumerate(forces):
            x,y=positions[i]; r=np.array([x,y,0.])
            body_force=np.array([0.,0.,f]); world_force=R@body_force
            self.data.xfrc_applied[self.body,:3]+=world_force
            self.data.xfrc_applied[self.body,3:6]+=R@np.cross(r,body_force)
        self.data.xfrc_applied[self.body,:2]+=self.rng.normal(0,self.wind,2)
        self.data.xfrc_applied[self.body,5]+=float(np.sum(ROTOR_YAW_SIGNS*forces)*self.params.km/self.params.kf)
        # Spin the visible propeller joints from motor RPM; slow the display rate to prevent visual aliasing.
        visual_rpm=self.rpm if self.visual_rpm is None else self.visual_rpm
        for dof,sign,rpm in zip(self.propeller_dofs,ROTOR_YAW_SIGNS,visual_rpm):
            self.data.qvel[dof]=sign*rpm*2*math.pi/60*self.propeller_visual_speed_scale
        mujoco.mj_step(self.model,self.data); self.data.xfrc_applied[self.body]=0
    def imu(self):
        # Accelerometer specific force from MuJoCo's inertial acceleration, expressed in the sensor frame.
        q=self.data.xquat[self.body]; R=np.zeros(9); mujoco.mju_quat2Mat(R,q); R=R.reshape(3,3)
        gravity=np.array([0.,0.,-9.81])
        specific=R.T@(self.data.qacc[self.dadr:self.dadr+3]-gravity)+self.rng.normal(0,self.accel_noise,3)
        gyro=self.data.qvel[self.dadr+3:self.dadr+6]+self.rng.normal(0,self.gyro_noise,3)
        return specific,gyro,q.copy()
    def image(self):
        if self.renderer is None: self.renderer=mujoco.Renderer(self.model,height=480,width=640)
        self.renderer.update_scene(self.data,camera='front_camera'); self.last_image=self.renderer.render().copy(); return self.last_image
