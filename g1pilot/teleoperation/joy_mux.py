#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

def yaw_from_quat(x,y,z,w):
    s=2.0*(w*z+x*y); c=1.0-2.0*(y*y+z*z)
    return math.atan2(s,c)

def clamp(v,a,b):
    return max(a,min(b,v))

class JoyMux(Node):
    def __init__(self):
        super().__init__('pure_pursuit_joy_mux')
        self.declare_parameter('path_topic','/g1pilot/path_smooth')
        self.declare_parameter('odom_topic','/g1pilot/odometry')
        self.declare_parameter('manual_topic','/g1pilot/joy_manual')
        self.declare_parameter('out_topic','/g1pilot/joy')
        self.declare_parameter('auto_enable_topic','/g1pilot/auto_enable')
        self.declare_parameter('publish_rate',50.0)
        self.declare_parameter('lookahead',0.6)
        self.declare_parameter('goal_tol',0.10)
        self.declare_parameter('vx_limit',0.5)
        self.declare_parameter('vy_limit',0.4)
        self.declare_parameter('wz_limit',0.4)
        self.declare_parameter('yaw_kp',1.5)
        self.declare_parameter('manual_priority_window',0.05)
        self.path_topic=self.get_parameter('path_topic').value
        self.odom_topic=self.get_parameter('odom_topic').value
        self.manual_topic=self.get_parameter('manual_topic').value
        self.out_topic=self.get_parameter('out_topic').value
        self.auto_enable_topic=self.get_parameter('auto_enable_topic').value
        self.rate=float(self.get_parameter('publish_rate').value)
        self.L=float(self.get_parameter('lookahead').value)
        self.goal_tol=float(self.get_parameter('goal_tol').value)
        self.vx_lim=float(self.get_parameter('vx_limit').value)
        self.vy_lim=float(self.get_parameter('vy_limit').value)
        self.wz_lim=float(self.get_parameter('wz_limit').value)
        self.yaw_kp=float(self.get_parameter('yaw_kp').value)
        self.man_win=float(self.get_parameter('manual_priority_window').value)
        qos=QoSProfile(depth=10)
        self.sub_odom=self.create_subscription(Odometry,self.odom_topic,self.cb_odom,qos)
        self.sub_path=self.create_subscription(Path,self.path_topic,self.cb_path,qos)
        self.sub_man=self.create_subscription(Joy,self.manual_topic,self.cb_manual,qos)
        self.sub_en=self.create_subscription(Bool,self.auto_enable_topic,self.cb_enable,qos)
        self.pub=self.create_publisher(Joy,self.out_topic,qos)
        self.timer=self.create_timer(1.0/self.rate,self.loop)
        self.x=self.y=self.yaw=0.0
        self.have_pose=False
        self.path=[]
        self.cumlen=[]
        self.goal=(0.0,0.0)
        self.auto_enabled=False
        self.last_manual=None
        self.t_last_manual=0.0

    def cb_odom(self,msg:Odometry):
        self.x=float(msg.pose.pose.position.x)
        self.y=float(msg.pose.pose.position.y)
        qx=msg.pose.pose.orientation.x
        qy=msg.pose.pose.orientation.y
        qz=msg.pose.pose.orientation.z
        qw=msg.pose.pose.orientation.w
        self.yaw=yaw_from_quat(qx,qy,qz,qw)
        self.have_pose=True

    def cb_path(self,msg:Path):
        self.path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        self.cumlen=[0.0]
        for i in range(1,len(self.path)):
            dl=math.hypot(self.path[i][0]-self.path[i-1][0],self.path[i][1]-self.path[i-1][1])
            self.cumlen.append(self.cumlen[-1]+dl)
        if self.path:
            self.goal=self.path[-1]

    def cb_manual(self,msg:Joy):
        self.last_manual=msg
        self.t_last_manual=time.time()
        if len(msg.buttons)>4 and msg.buttons[4]==1:
            self.auto_enabled=False
            self.pub.publish(msg)

    def cb_enable(self,msg:Bool):
        self.auto_enabled=bool(msg.data)

    def nearest_index(self):
        if not self.path:
            return 0
        best=0; bd=1e9
        for i,(px,py) in enumerate(self.path):
            d=(px-self.x)*(px-self.x)+(py-self.y)*(py-self.y)
            if d<bd:
                bd=d; best=i
        return best

    def target_point(self, idx):
        if not self.path:
            return (self.x,self.y), idx
        L=self.L
        start_s=self.cumlen[idx]
        target_s=start_s+L
        j=idx
        while j+1<len(self.path) and self.cumlen[j]<target_s:
            j+=1
        return self.path[j], j

    def segment_yaw(self,j):
        if not self.path:
            return self.yaw
        if j<=0:
            a=self.path[0]; b=self.path[min(1,len(self.path)-1)]
        else:
            a=self.path[j-1]; b=self.path[j]
        return math.atan2(b[1]-a[1],b[0]-a[0])

    def wrap(self,a):
        while a>math.pi: a-=2*math.pi
        while a<-math.pi: a+=2*math.pi
        return a

    def build_auto_joy(self):
        if not(self.have_pose and self.path):
            j=Joy(); j.axes=[0.0]*8; j.buttons=[0]*8; return j
        if math.hypot(self.goal[0]-self.x,self.goal[1]-self.y)<=self.goal_tol:
            j=Joy(); j.axes=[0.0]*8; j.buttons=[0]*8; return j
        i=self.nearest_index()
        tgt,jidx=self.target_point(i)
        dx=tgt[0]-self.x; dy=tgt[1]-self.y
        c=math.cos(-self.yaw); s=math.sin(-self.yaw)
        vx_b=c*dx - s*dy
        vy_b=s*dx + c*dy
        n=math.hypot(vx_b,vy_b)
        if n>1e-6:
            vx_b/=n; vy_b/=n
        seg_dir=self.segment_yaw(jidx)
        yaw_err=self.wrap(seg_dir - self.yaw)
        wz=clamp(self.yaw_kp*yaw_err,-self.wz_lim,self.wz_lim)
        ax1=clamp(-vx_b,-1.0,1.0)
        ax0=clamp(-vy_b,-1.0,1.0)
        ax3=clamp(-wz/self.wz_lim,-1.0,1.0)
        out=Joy()
        out.axes=[0.0]*8
        out.buttons=[0]*8
        out.axes[1]=ax1
        out.axes[0]=ax0
        out.axes[3]=ax3
        out.buttons[7]=1
        return out

    def loop(self):
        now=time.time()
        use_manual=self.last_manual is not None and (not self.auto_enabled or (now-self.t_last_manual)<self.man_win)
        if use_manual:
            self.pub.publish(self.last_manual)
            return
        if self.auto_enabled:
            aj=self.build_auto_joy()
            self.pub.publish(aj)
            return
        if self.last_manual:
            self.pub.publish(self.last_manual)

def main(args=None):
    rclpy.init(args=args)
    n=JoyMux()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
