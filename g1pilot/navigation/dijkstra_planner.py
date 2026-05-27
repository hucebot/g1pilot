#!/usr/bin/env python3
import math, heapq
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Header

def _dist(a,b):
    dx=a[0]-b[0]; dy=a[1]-b[1]
    return math.hypot(dx,dy)

def yaw_from_quat(x, y, z, w):
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)

def goal_yaw_from_pose(pose):
    q = pose.orientation
    if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) <= 1e-6:
        return None
    return yaw_from_quat(q.x, q.y, q.z, q.w)

def _catmull_rom_centripetal(points, samples_per_seg=8, closed=False):
    if len(points)<2: return points[:]
    P=points[:]
    P=([P[-1]]+P+[P[0],P[1]]) if closed else ([P[0]]+P+[P[-1]])
    out=[]
    for i in range(1,len(P)-2):
        p0,p1,p2,p3=P[i-1],P[i],P[i+1],P[i+2]
        t0=0.0
        t1=t0+math.sqrt(_dist(p0,p1))
        t2=t1+math.sqrt(_dist(p1,p2))
        t3=t2+math.sqrt(_dist(p2,p3))
        if t1==t0 or t2==t1 or t3==t2:
            if not out or _dist(out[-1],p1)>1e-6: out.append(p1)
            if i==len(P)-3 and (not out or _dist(out[-1],p2)>1e-6): out.append(p2)
            continue
        for s in range(samples_per_seg):
            t=t1+(t2-t1)*s/float(samples_per_seg)
            A1=((t1-t)/(t1-t0))*p0[0]+((t-t0)/(t1-t0))*p1[0], ((t1-t)/(t1-t0))*p0[1]+((t-t0)/(t1-t0))*p1[1]
            A2=((t2-t)/(t2-t1))*p1[0]+((t-t1)/(t2-t1))*p2[0], ((t2-t)/(t2-t1))*p1[1]+((t-t1)/(t2-t1))*p2[1]
            A3=((t3-t)/(t3-t2))*p2[0]+((t-t2)/(t3-t2))*p3[0], ((t3-t)/(t3-t2))*p2[1]+((t-t2)/(t3-t2))*p3[1]
            B1=((t2-t)/(t2-t0))*A1[0]+((t-t0)/(t2-t0))*A2[0], ((t2-t)/(t2-t0))*A1[1]+((t-t0)/(t2-t0))*A2[1]
            B2=((t3-t)/(t3-t1))*A2[0]+((t-t1)/(t3-t1))*A3[0], ((t3-t)/(t3-t1))*A2[1]+((t-t1)/(t3-t1))*A3[1]
            C=((t2-t)/(t2-t1))*B1[0]+((t-t1)/(t2-t1))*B2[0], ((t2-t)/(t2-t1))*B1[1]+((t-t1)/(t2-t1))*B2[1]
            if not out or _dist(out[-1],C)>1e-6: out.append(C)
        if i==len(P)-3:
            if not out or _dist(out[-1],p2)>1e-6: out.append(p2)
    return out

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')
        self.declare_parameter('map_topic','/map')
        self.declare_parameter('odom_topic','/lidar_odometry/pose_fixed')
        self.declare_parameter('goal_topic','/g1pilot/goal')
        self.declare_parameter('path_topic','/g1pilot/path')
        self.declare_parameter('occ_threshold',50)
        self.declare_parameter('allow_diagonal',True)
        self.declare_parameter('straight_steps',50)
        self.declare_parameter('inflation_radius_m',0.40)
        self.declare_parameter('smooth_enable',True)
        self.declare_parameter('smooth_samples_per_segment',8)
        self.declare_parameter('smooth_closed',False)
        self.declare_parameter('simplify_min_dist',0.02)
        self.declare_parameter('shortcut_enable',True)
        self.declare_parameter('turn_cost_gain',2.0)
        qos=QoSProfile(depth=10)
        self.sub_map=self.create_subscription(OccupancyGrid,self.get_parameter('map_topic').value,self.cb_map,qos)
        self.sub_odom=self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.cb_odom,qos)
        self.sub_goal=self.create_subscription(PoseStamped,self.get_parameter('goal_topic').value,self.cb_goal,qos)
        self.pub_path=self.create_publisher(Path,self.get_parameter('path_topic').value,qos)
        self.map=None
        self.map_frame='map'
        self.res=self.ox=self.oy=0.0
        self.w=self.h=0
        self.occ=[]; self.occ_inf=[]
        self.inf_radius_cells=0
        self.have_pose=False
        self.px=self.py=self.pyaw=0.0

    def cb_map(self,msg):
        self.map=msg
        self.map_frame=msg.header.frame_id or 'map'
        self.res=float(msg.info.resolution)
        self.ox=float(msg.info.origin.position.x)
        self.oy=float(msg.info.origin.position.y)
        self.w=int(msg.info.width)
        self.h=int(msg.info.height)
        self.occ=list(msg.data)
        self.inf_radius_cells=int(math.ceil(0.40/self.res)) if self.res>0.0 else 0
        self.occ_inf=self.inflate_occupancy(self.occ,self.w,self.h,self.inf_radius_cells,50)

    def cb_odom(self,msg):
        self.px=float(msg.pose.pose.position.x)
        self.py=float(msg.pose.pose.position.y)
        q=msg.pose.pose.orientation
        siny_cosp=2*(q.w*q.z+q.x*q.y)
        cosy_cosp=1-2*(q.y*q.y+q.z*q.z)
        self.pyaw=math.atan2(siny_cosp,cosy_cosp)
        self.have_pose=True

    def cb_goal(self,msg):
        if not self.have_pose:
            self.get_logger().warn("No odom pose yet.")
            return
        gx=float(msg.pose.position.x); gy=float(msg.pose.position.y)
        goal_yaw=goal_yaw_from_pose(msg.pose)
        frame_id=msg.header.frame_id or 'map'
        if self.map is None:
            self.publish_path(self.line_points(self.px,self.py,gx,gy,frame_id),frame_id,goal_yaw)
            return
        sx,sy=self.world_to_grid(self.px,self.py)
        gx_i,gy_i=self.world_to_grid(gx,gy)
        if not self.in_bounds(sx,sy) or not self.in_bounds(gx_i,gy_i):
            self.publish_path(self.line_points(self.px,self.py,gx,gy,self.map_frame),self.map_frame,goal_yaw); return
        if self.is_occ(sx,sy) or self.is_occ(gx_i,gy_i):
            self.publish_path(self.line_points(self.px,self.py,gx,gy,self.map_frame),self.map_frame,goal_yaw); return
        path_idx=self.dijkstra((sx,sy,self.pyaw),(gx_i,gy_i))
        if not path_idx:
            self.publish_path(self.line_points(self.px,self.py,gx,gy,self.map_frame),self.map_frame,goal_yaw); return
        pts=[self.grid_to_world(ix,iy) for ix,iy in path_idx]
        pts=self.simplify_spacing(pts,0.02)
        pts=self.shortcut_path(pts)
        pts=_catmull_rom_centripetal(pts,8,False)
        self.publish_path(pts,self.map_frame,goal_yaw)

    def world_to_grid(self,x,y):
        return int(math.floor((x-self.ox)/self.res)), int(math.floor((y-self.oy)/self.res))
    def grid_to_world(self,ix,iy):
        return self.ox+(ix+0.5)*self.res, self.oy+(iy+0.5)*self.res
    def in_bounds(self,ix,iy): return 0<=ix<self.w and 0<=iy<self.h
    def is_occ(self,ix,iy):
        v=self.occ_inf[iy*self.w+ix]
        return v>=50 and v!=255

    def neighbors(self,ix,iy):
        n=[(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0)]
        rt2=math.sqrt(2)
        n+= [(-1,-1,rt2),(1,-1,rt2),(-1,1,rt2),(1,1,rt2)]
        for dx,dy,c in n:
            nx,ny=ix+dx,iy+dy
            if self.in_bounds(nx,ny) and not self.is_occ(nx,ny):
                yield nx,ny,c

    def dijkstra(self,start,goal):
        sx,sy,syaw=start; gx,gy=goal
        dist={(sx,sy):0.0}; prev={}
        pq=[(0.0,sx,sy,syaw)]
        vis=set()
        k_turn=float(self.get_parameter('turn_cost_gain').value)
        while pq:
            d,x,y,yaw_prev=heapq.heappop(pq)
            if (x,y) in vis: continue
            vis.add((x,y))
            if (x,y)==(gx,gy): break
            for nx,ny,c in self.neighbors(x,y):
                yaw_next=math.atan2(ny-y,nx-x)
                delta=abs(math.atan2(math.sin(yaw_next - yaw_prev), math.cos(yaw_next - yaw_prev)))
                turn_cost=1.0 + k_turn * delta
                nd=d + c * turn_cost
                if nd < dist.get((nx,ny),float('inf')):
                    dist[(nx,ny)]=nd
                    prev[(nx,ny)]=(x,y,yaw_next)
                    heapq.heappush(pq,(nd,nx,ny,yaw_next))
        if (gx,gy) not in dist: return None
        path=[]; cur=(gx,gy)
        while cur in prev or cur==(sx,sy):
            path.append(cur)
            if cur==(sx,sy): break
            cur=(prev[cur][0],prev[cur][1])
        path.reverse()
        return path

    def publish_path(self,pts,frame_id,goal_yaw=None):
        path=Path()
        path.header=Header()
        path.header.stamp=self.get_clock().now().to_msg()
        path.header.frame_id=frame_id
        path.poses=[]
        prev_yaw=self.pyaw
        for i,(x,y) in enumerate(pts):
            p=PoseStamped()
            p.header=path.header
            p.pose.position.x=x; p.pose.position.y=y
            if i == len(pts)-1 and goal_yaw is not None:
                yaw=goal_yaw
            else:
                if i < len(pts)-1:
                    nx,ny=pts[i+1]
                    yaw=math.atan2(ny-y,nx-x)
                else:
                    yaw=prev_yaw
                alpha=0.3
                yaw=prev_yaw+alpha*math.atan2(math.sin(yaw-prev_yaw),math.cos(yaw-prev_yaw))
            prev_yaw=yaw
            p.pose.orientation.z=math.sin(yaw/2.0)
            p.pose.orientation.w=math.cos(yaw/2.0)
            path.poses.append(p)
        self.pub_path.publish(path)

    def line_points(self,sx,sy,gx,gy,frame_id):
        pts=[]
        for i in range(50+1):
            a=i/50.0
            x=(1-a)*sx+a*gx; y=(1-a)*sy+a*gy
            pts.append((x,y))
        return _catmull_rom_centripetal(pts,8,False)

    def inflate_occupancy(self,occ,w,h,r_cells,occ_th):
        if r_cells<=0: return occ[:]
        inflated=[0]*(w*h)
        occ_cells=[(i%w,i//w) for i,v in enumerate(occ) if v>=occ_th and v!=255]
        for ox,oy in occ_cells:
            xmin=max(0,ox-r_cells); xmax=min(w-1,ox+r_cells)
            ymin=max(0,oy-r_cells); ymax=min(h-1,oy+r_cells)
            r2=r_cells*r_cells
            for y in range(ymin,ymax+1):
                dy=y-oy; dy2=dy*dy
                base=y*w
                for x in range(xmin,xmax+1):
                    dx=x-ox
                    if dx*dx+dy2<=r2:
                        inflated[base+x]=max(inflated[base+x],100)
        for i,v in enumerate(occ):
            if v==255: inflated[i]=255
        return inflated

    def simplify_spacing(self,pts,min_d):
        if not pts: return pts
        out=[pts[0]]
        for p in pts[1:]:
            if _dist(out[-1],p)>=min_d: out.append(p)
        if out[-1]!=pts[-1]: out.append(pts[-1])
        return out

    def shortcut_path(self,pts):
        if len(pts)<=2: return pts
        grid_pts=[self.world_to_grid(x,y) for x,y in pts]
        out=[pts[0]]; i=0
        while i<len(grid_pts)-1:
            j=len(grid_pts)-1
            while j>i+1 and not self.grid_line_clear(grid_pts[i],grid_pts[j]): j-=1
            out.append(pts[j]); i=j
        return out

    def grid_line_clear(self,a,b):
        x0,y0=a; x1,y1=b
        dx=abs(x1-x0); dy=abs(y1-y0)
        sx=1 if x0<x1 else -1
        sy=1 if y0<y1 else -1
        err=dx-dy; x,y=x0,y0
        while True:
            if not self.in_bounds(x,y) or self.is_occ(x,y): return False
            if x==x1 and y==y1: break
            e2=2*err
            if e2>-dy: err-=dy; x+=sx
            if e2<dx: err+=dx; y+=sy
        return True

def main(args=None):
    rclpy.init(args=args)
    node=DijkstraPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
