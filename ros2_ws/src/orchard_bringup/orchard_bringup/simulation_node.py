from pathlib import Path
import sys
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, Image, BatteryState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(get_package_share_directory('orchard_bringup'))/'python'))
from orchard_drone.runner import Simulation
from orchard_drone.core import TREE_POSITIONS

class OrchardSimulationNode(Node):
    """Run the full autonomous MuJoCo stack and publish its ROS sensor/state interfaces."""
    def __init__(self):
        super().__init__('orchard_sim')
        self.root=Path(get_package_share_directory('orchard_bringup'))
        self.stack=Simulation(self.root)
        self.result=None; self.worker=threading.Thread(target=self._run_stack,daemon=True); self.worker.start()
        self.imu_pub=self.create_publisher(Imu,'/imu',10)
        self.rpm_pub=self.create_publisher(Float32MultiArray,'/motor_rpm',10)
        self.command_pub=self.create_publisher(Float32MultiArray,'/motor_commands',10)
        self.image_pub=self.create_publisher(Image,'/camera/image_raw',2)
        self.pose_pub=self.create_publisher(Odometry,'/drone_pose',10)
        self.waypoint_pub=self.create_publisher(PoseStamped,'/waypoint_command',10)
        self.battery_pub=self.create_publisher(BatteryState,'/battery_state',10)
        self.path_pub=self.create_publisher(NavPath,'/planned_path',10)
        self.aruco_pub=self.create_publisher(MarkerArray,'/aruco_detections',10)
        self.timer=self.create_timer(.05,self._publish)
    def _run_stack(self):
        self.result=self.stack.run(max_time=240,headless=True)
        self.get_logger().info(f"mission ended: {self.result}")
    def _stamp(self): return self.get_clock().now().to_msg()
    def _publish(self):
        sim=self.stack.sim; stamp=self._stamp(); accel,gyro,_=sim.imu(); ekf=self.stack.ekf
        im=Imu(); im.header.stamp=stamp; im.header.frame_id='imu_link'; im.linear_acceleration.x,im.linear_acceleration.y,im.linear_acceleration.z=map(float,accel); im.angular_velocity.x,im.angular_velocity.y,im.angular_velocity.z=map(float,gyro)
        quat=Rotation.from_euler('xyz',ekf.x[9:12]).as_quat(); im.orientation.x,im.orientation.y,im.orientation.z,im.orientation.w=map(float,quat); self.imu_pub.publish(im)
        rpm=Float32MultiArray(); rpm.data=list(map(float,sim.rpm)); self.rpm_pub.publish(rpm); commands=Float32MultiArray(); commands.data=list(map(float,sim.command)); self.command_pub.publish(commands)
        od=Odometry(); od.header.stamp=stamp; od.header.frame_id='orchard'; od.child_frame_id='whoop'; od.pose.pose.position.x,od.pose.pose.position.y,od.pose.pose.position.z=map(float,ekf.x[:3]); od.pose.pose.orientation.x,od.pose.pose.orientation.y,od.pose.pose.orientation.z,od.pose.pose.orientation.w=map(float,quat); od.twist.twist.linear.x,od.twist.twist.linear.y,od.twist.twist.linear.z=map(float,ekf.x[3:6]); self.pose_pub.publish(od)
        target=self.stack.target
        if target is not None:
            w=PoseStamped(); w.header.stamp=stamp; w.header.frame_id='orchard'; w.pose.position.x,w.pose.position.y,w.pose.position.z=map(float,target); w.pose.orientation.w=1.; self.waypoint_pub.publish(w)
        bs=BatteryState(); bs.header.stamp=stamp; bs.percentage=float(self.stack.battery.percent/100); bs.charge=float(self.stack.battery.remaining/3600); bs.capacity=float(self.stack.battery.capacity/3600); self.battery_pub.publish(bs)
        image=sim.last_image
        if image is not None:
            msg=Image(); msg.header.stamp=stamp; msg.header.frame_id='front_camera'; msg.height, msg.width=image.shape[:2]; msg.encoding='rgb8'; msg.is_bigendian=False; msg.step=msg.width*3; msg.data=image.tobytes(); self.image_pub.publish(msg)
        detected=self.stack.last_detection
        if detected is not None:
            marker=Marker(); marker.header.stamp=stamp; marker.header.frame_id='front_camera'; marker.ns='aruco'; marker.id=int(detected['id']); marker.type=Marker.CUBE; marker.action=Marker.ADD; marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=map(float,detected['tvec']); marker.pose.orientation.w=1.; marker.scale.x=marker.scale.y=marker.scale.z=.03; marker.color.r=1.; marker.color.g=.2; marker.color.a=1.; arr=MarkerArray(); arr.markers=[marker]; self.aruco_pub.publish(arr)
        path=NavPath(); path.header.stamp=stamp; path.header.frame_id='orchard'; current=self.stack.mission.current_tree or self.stack.mission.next_tree; nxt=self.stack.mission.next_tree
        if current is not None and nxt is not None:
            ids=self.stack.graph.path(current,nxt)
            for tree in ids:
                p=TREE_POSITIONS[tree].copy(); p[1]-=3.; p[2]=2.2; pose=PoseStamped(); pose.header=path.header; pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,p); pose.pose.orientation.w=1.; path.poses.append(pose)
        self.path_pub.publish(path)
    def destroy_node(self):
        if self.worker.is_alive(): self.worker.join(timeout=2.)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node=OrchardSimulationNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
