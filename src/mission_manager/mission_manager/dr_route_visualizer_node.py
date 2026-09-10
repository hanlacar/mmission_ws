#!/usr/bin/env python3
import csv, math, re
from pathlib import Path
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_q(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

def q_from_yaw(yaw):
    q=Quaternion(); q.z=math.sin(yaw/2); q.w=math.cos(yaw/2); return q

def norm(a): return math.atan2(math.sin(a), math.cos(a))

class DrRouteVisualizer(Node):
    def __init__(self):
        super().__init__('dr_route_visualizer')
        for n,v in {
            'route_path':'','odom_topic':'/odom','status_topic':'/dr_navigation/status',
            'frame_id':'odom','align_route_to_start':True,'max_actual_points':12000,
            'publish_rate_hz':10.0}.items(): self.declare_parameter(n,v)
        self.route_path=Path(str(self.get_parameter('route_path').value)).expanduser()
        if not self.route_path.is_file(): raise RuntimeError(f'route_path not found: {self.route_path}')
        self.odom_topic=str(self.get_parameter('odom_topic').value)
        self.status_topic=str(self.get_parameter('status_topic').value)
        self.frame_id=str(self.get_parameter('frame_id').value)
        self.align_enabled=bool(self.get_parameter('align_route_to_start').value)
        self.max_actual_points=max(100,int(self.get_parameter('max_actual_points').value))
        self.raw=self.load_route(self.route_path); self.route=[dict(p) for p in self.raw]
        self.aligned=False; self.locked=False; self.last_odom=None; self.actual=[]
        self.status_text='WAITING'; self.cursor=0; self.nearest_idx=0; self.cte=0.0
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.ref_pub=self.create_publisher(NavPath,'/dr_viz/reference_path',latched)
        self.act_pub=self.create_publisher(NavPath,'/dr_viz/actual_path',10)
        self.marker_pub=self.create_publisher(MarkerArray,'/dr_viz/markers',10)
        self.create_subscription(Odometry,self.odom_topic,self.on_odom,30)
        self.create_subscription(String,self.status_topic,self.on_status,30)
        self.create_service(Trigger,'/dr_viz/reset',self.on_reset)
        hz=max(1.0,float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0/hz,self.publish_all)
        self.get_logger().info(f'DR RViz ready: {self.route_path} ({len(self.raw)} pts)')

    def load_route(self,path):
        pts=[]
        with path.open(newline='',encoding='utf-8-sig') as f:
            r=csv.DictReader(f); names=set(r.fieldnames or [])
            if not {'dr_x_m','dr_y_m','direction'}.issubset(names):
                raise RuntimeError('DR CSV requires dr_x_m, dr_y_m, direction')
            for row in r:
                pts.append({'x':float(row['dr_x_m']),'y':float(row['dr_y_m']),
                    'yaw':math.radians(float(row.get('dr_yaw_deg',0.0) or 0.0)),
                    'direction':1 if int(float(row.get('direction',1)))>=0 else -1,
                    'mode':int(float(row.get('mode',1) or 1))})
        if len(pts)<2: raise RuntimeError('DR route requires >=2 points')
        return pts

    def align_to_odom(self,msg,lock=False):
        self.route=[dict(p) for p in self.raw]
        if self.align_enabled:
            first=self.route[0]; yaw=norm(yaw_from_q(msg.pose.pose.orientation)-first['yaw'])
            c,s=math.cos(yaw),math.sin(yaw); x0,y0=first['x'],first['y']
            ox,oy=float(msg.pose.pose.position.x),float(msg.pose.pose.position.y)
            for p in self.route:
                dx,dy=p['x']-x0,p['y']-y0
                p['x']=ox+c*dx-s*dy; p['y']=oy+s*dx+c*dy; p['yaw']=norm(p['yaw']+yaw)
        self.aligned=True
        if lock: self.locked=True; self.actual.clear()
        self.publish_reference()

    def on_odom(self,msg):
        self.last_odom=msg
        if not self.aligned: self.align_to_odom(msg,False)
        x,y=float(msg.pose.pose.position.x),float(msg.pose.pose.position.y)
        self.actual.append((x,y))
        if len(self.actual)>self.max_actual_points: del self.actual[:-self.max_actual_points]
        self.nearest_idx,self.cte=self.nearest(x,y)

    def on_status(self,msg):
        self.status_text=str(msg.data)
        m=re.search(r'idx=(\d+)',self.status_text)
        if m: self.cursor=max(0,min(int(m.group(1)),len(self.route)-1))
        active=self.status_text.startswith(('TRACKING','OFF_ROUTE','GOAL_REACHED'))
        if active and not self.locked and self.last_odom is not None:
            self.align_to_odom(self.last_odom,True)

    def nearest(self,x,y):
        best_i=0; best=float('inf')
        for i,p in enumerate(self.route):
            d=(p['x']-x)**2+(p['y']-y)**2
            if d<best: best=d; best_i=i
        return best_i,math.sqrt(best)

    def path_msg(self,points):
        msg=NavPath(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id=self.frame_id
        for p in points:
            ps=PoseStamped(); ps.header=msg.header
            if isinstance(p,tuple): ps.pose.position.x,ps.pose.position.y=p; ps.pose.orientation.w=1.0
            else: ps.pose.position.x=p['x']; ps.pose.position.y=p['y']; ps.pose.orientation=q_from_yaw(p['yaw'])
            msg.poses.append(ps)
        return msg

    def publish_reference(self):
        if self.aligned: self.ref_pub.publish(self.path_msg(self.route))

    def marker(self,mid,mtype,ns):
        m=Marker(); m.header.frame_id=self.frame_id; m.header.stamp=self.get_clock().now().to_msg()
        m.ns=ns; m.id=mid; m.type=mtype; m.action=Marker.ADD; m.pose.orientation.w=1.0; return m

    def publish_markers(self):
        if self.last_odom is None or not self.aligned: return
        arr=MarkerArray(); x=float(self.last_odom.pose.pose.position.x); y=float(self.last_odom.pose.pose.position.y)
        yaw=yaw_from_q(self.last_odom.pose.pose.orientation)
        v=self.marker(0,Marker.ARROW,'vehicle'); v.pose.position.x=x; v.pose.position.y=y; v.pose.position.z=.1
        v.pose.orientation=q_from_yaw(yaw); v.scale.x=.75; v.scale.y=.22; v.scale.z=.22
        v.color.r=v.color.g=v.color.b=v.color.a=1.0; arr.markers.append(v)
        tp=self.route[max(0,min(self.cursor,len(self.route)-1))]
        t=self.marker(1,Marker.SPHERE,'target'); t.pose.position.x=tp['x']; t.pose.position.y=tp['y']; t.pose.position.z=.1
        t.scale.x=t.scale.y=t.scale.z=.35; t.color.r=1.0; t.color.b=1.0; t.color.a=1.0; arr.markers.append(t)
        np_=self.route[self.nearest_idx]
        e=self.marker(2,Marker.LINE_STRIP,'cte'); e.scale.x=.07; e.color.r=1.0; e.color.a=1.0
        a,b=Point(),Point(); a.x,a.y,a.z=x,y,.06; b.x,b.y,b.z=np_['x'],np_['y'],.06; e.points=[a,b]; arr.markers.append(e)
        txt=self.marker(3,Marker.TEXT_VIEW_FACING,'status'); txt.pose.position.x=x; txt.pose.position.y=y; txt.pose.position.z=.85
        txt.scale.z=.30; txt.color.r=txt.color.g=txt.color.b=txt.color.a=1.0
        txt.text=f'{self.status_text}\nnearest={self.nearest_idx}  CTE={self.cte:.2f}m'; arr.markers.append(txt)
        self.marker_pub.publish(arr)

    def publish_all(self):
        if self.aligned: self.publish_reference()
        self.act_pub.publish(self.path_msg(self.actual))
        self.publish_markers()

    def on_reset(self,req,res):
        del req; self.actual.clear(); self.status_text='WAITING'; self.cursor=0; self.locked=False
        if self.last_odom is not None: self.align_to_odom(self.last_odom,False)
        else: self.aligned=False; self.route=[dict(p) for p in self.raw]
        res.success=True; res.message='DR RViz trace reset'; return res

def main(args=None):
    rclpy.init(args=args); n=DrRouteVisualizer()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
