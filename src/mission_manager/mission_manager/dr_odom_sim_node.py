#!/usr/bin/env python3
import math,time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32,Int32
from std_srvs.srv import Trigger

class DrOdomSim(Node):
    def __init__(self):
        super().__init__('dr_odom_sim')
        for n,v in {'odom_topic':'/odom','drive_topic':'/gps_drive','wheel_topic':'/gps_wheel',
            'frame_id':'odom','child_frame_id':'base_link','wheelbase_m':.73,'max_steer_deg':22.0,
            'rate_hz':30.0,'command_timeout_s':.5,'stage1_mps':.25,'stage2_mps':.50,
            'stage3_mps':.75,'reverse_mps':.25,'initial_x':0.0,'initial_y':0.0,'initial_yaw_deg':0.0}.items():
            self.declare_parameter(n,v)
        self.odom_topic=str(self.get_parameter('odom_topic').value); self.drive_topic=str(self.get_parameter('drive_topic').value)
        self.wheel_topic=str(self.get_parameter('wheel_topic').value); self.frame_id=str(self.get_parameter('frame_id').value)
        self.child=str(self.get_parameter('child_frame_id').value); self.L=max(.01,float(self.get_parameter('wheelbase_m').value))
        self.max_steer=abs(float(self.get_parameter('max_steer_deg').value)); self.rate=max(5.0,float(self.get_parameter('rate_hz').value))
        self.timeout=max(.05,float(self.get_parameter('command_timeout_s').value))
        self.speeds={1:float(self.get_parameter('stage1_mps').value),2:float(self.get_parameter('stage2_mps').value),3:float(self.get_parameter('stage3_mps').value)}
        self.rev=abs(float(self.get_parameter('reverse_mps').value))
        self.x0=float(self.get_parameter('initial_x').value); self.y0=float(self.get_parameter('initial_y').value)
        self.yaw0=math.radians(float(self.get_parameter('initial_yaw_deg').value)); self.reset_state()
        self.pub=self.create_publisher(Odometry,self.odom_topic,20)
        self.create_subscription(Float32,self.drive_topic,self.on_drive,20); self.create_subscription(Int32,self.wheel_topic,self.on_wheel,20)
        self.create_service(Trigger,'/dr_sim/reset',self.on_reset); self.create_timer(1.0/self.rate,self.tick)
        self.get_logger().info('No-car DR closed-loop odom simulator ready')
    def reset_state(self):
        self.x,self.y,self.yaw=self.x0,self.y0,self.yaw0; self.drive=0.0; self.wheel=0.0; self.t_drive=None; self.t_wheel=None; self.last=time.monotonic()
    def on_drive(self,m): self.drive=float(m.data); self.t_drive=time.monotonic()
    def on_wheel(self,m): self.wheel=max(-self.max_steer,min(self.max_steer,float(m.data))); self.t_wheel=time.monotonic()
    def speed(self,now):
        if self.t_drive is None or now-self.t_drive>self.timeout or abs(self.drive)<.01: return 0.0
        if self.drive<0: return -self.rev
        return self.speeds.get(int(round(abs(self.drive))),0.0)
    def tick(self):
        now=time.monotonic(); dt=now-self.last; self.last=now
        if dt<=0 or dt>.2: dt=1.0/self.rate
        v=self.speed(now); steer=self.wheel if self.t_wheel is not None and now-self.t_wheel<=self.timeout else 0.0
        sr=math.radians(steer); self.x+=v*math.cos(self.yaw)*dt; self.y+=v*math.sin(self.yaw)*dt
        self.yaw+=v/self.L*math.tan(sr)*dt; self.yaw=math.atan2(math.sin(self.yaw),math.cos(self.yaw))
        o=Odometry(); o.header.stamp=self.get_clock().now().to_msg(); o.header.frame_id=self.frame_id; o.child_frame_id=self.child
        o.pose.pose.position.x=self.x; o.pose.pose.position.y=self.y; o.pose.pose.orientation.z=math.sin(self.yaw/2); o.pose.pose.orientation.w=math.cos(self.yaw/2)
        o.twist.twist.linear.x=v; o.twist.twist.angular.z=v/self.L*math.tan(sr); self.pub.publish(o)
    def on_reset(self,req,res):
        del req; self.reset_state(); res.success=True; res.message='DR sim reset'; return res

def main(args=None):
    rclpy.init(args=args); n=DrOdomSim()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
