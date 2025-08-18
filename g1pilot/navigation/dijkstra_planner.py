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

class DijkstraPlannerSmooth(Node):
    def __init__(self):
        super().__init__('dijkstra_planner_smooth')
        self.declare_parameter('map_topic','/map')
        self.declare_parameter('odom_topic','/g1pilot/odometry')
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
        self.map_topic=self.get_parameter('map_topic').value
        self.odom_topic=self.get_parameter('odom_topic').value
        self.goal_topic=self.get_parameter('goal_topic').value
        self.path_topic=self.get_parameter('path_topic').value
        self.occ_th=int(self.get_parameter('occ_threshold').value)
        self.allow_diag=bool(self.get_parameter('allow_diagonal').value)
        self.straight_steps=int(self.get_parameter('straight_steps').value)
        self.inflation_radius_m=float(self.get_parameter('inflation_radius_m').value)
        self.smooth_enable=bool(self.get_parameter('smooth_enable').value)
        self.sps=int(self.get_parameter('smooth_samples_per_segment').value)
        self.smooth_closed=bool(self.get_parameter('smooth_closed').value)
        self.simplify_min_dist=float(self.get_parameter('simplify_min_dist').value)
        self.shortcut_enable=bool(self.get_parameter('shortcut_enable').value)
        qos=QoSProfile(depth=10)
        self.sub_map=self.create_subscription(OccupancyGrid,self.map_topic,self.cb_map,qos)
        self.sub_odom=self.create_subscription(Odometry,self.odom_topic,self.cb_odom,qos)
        self.sub_goal=self.create_subscription(PointStamped,self.goal_topic,self.cb_goal,qos)
        self.pub_path=self.create_publisher(Path,self.path_topic,qos)
        self.map=None
        self.map_frame='map'
        self.res=0.0
        self.ox=0.0
        self.oy=0.0
        self.w=0
        self.h=0
        self.occ=[]
        self.occ_inf=[]
        self.inf_radius_cells=0
        self.have_pose=False
        self.px=0.0
        self.py=0.0

    def cb_map(self,msg:OccupancyGrid):
        self.map=msg
        self.map_frame=msg.header.frame_id if msg.header.frame_id else 'map'
        self.res=float(msg.info.resolution)
        self.ox=float(msg.info.origin.position.x)
        self.oy=float(msg.info.origin.position.y)
        self.w=int(msg.info.width)
        self.h=int(msg.info.height)
        self.occ=list(msg.data)
        self.inf_radius_cells=int(math.ceil(self.inflation_radius_m/self.res)) if self.res>0.0 else 0
        self.occ_inf=self.inflate_occupancy(self.occ,self.w,self.h,self.inf_radius_cells,self.occ_th)

    def cb_odom(self,msg:Odometry):
        self.px=float(msg.pose.pose.position.x)
        self.py=float(msg.pose.pose.position.y)
        self.have_pose=True

    def cb_goal(self,msg:PointStamped):
        if not self.have_pose:
            self.px=0.0; self.py=0.0
        gx=float(msg.point.x); gy=float(msg.point.y)
        if self.map is None:
            pts=self.line_points(self.px,self.py,gx,gy,msg.header.frame_id or 'world')
            self.publish_path(pts,msg.header.frame_id or 'world'); return
        sx,sy=self.world_to_grid(self.px,self.py)
        gx_i,gy_i=self.world_to_grid(gx,gy)
        if not self.in_bounds(sx,sy) or not self.in_bounds(gx_i,gy_i):
            pts=self.line_points(self.px,self.py,gx,gy,self.map_frame)
            self.publish_path(pts,self.map_frame); return
        if self.is_occ(sx,sy) or self.is_occ(gx_i,gy_i):
            pts=self.line_points(self.px,self.py,gx,gy,self.map_frame)
            self.publish_path(pts,self.map_frame); return
        path_idx=self.dijkstra((sx,sy),(gx_i,gy_i))
        if not path_idx:
            pts=self.line_points(self.px,self.py,gx,gy,self.map_frame)
            self.publish_path(pts,self.map_frame); return
        pts=[self.grid_to_world(ix,iy) for ix,iy in path_idx]
        pts=self.simplify_spacing(pts,self.simplify_min_dist)
        if self.shortcut_enable:
            pts=self.shortcut_path(pts)
        if self.smooth_enable and len(pts)>=2:
            pts=_catmull_rom_centripetal(pts,self.sps,self.smooth_closed)
        self.publish_path(pts,self.map_frame)

    def world_to_grid(self,x,y):
        return int(math.floor((x-self.ox)/self.res)), int(math.floor((y-self.oy)/self.res))

    def grid_to_world(self,ix,iy):
        return self.ox+(ix+0.5)*self.res, self.oy+(iy+0.5)*self.res

    def in_bounds(self,ix,iy):
        return 0<=ix<self.w and 0<=iy<self.h

    def is_occ(self,ix,iy):
        v=self.occ_inf[iy*self.w+ix]
        return v>=self.occ_th and v!=255

    def neighbors(self,ix,iy):
        n=[(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0)]
        if self.allow_diag:
            rt2=math.sqrt(2)
            n+= [(-1,-1,rt2),(1,-1,rt2),(-1,1,rt2),(1,1,rt2)]
        for dx,dy,c in n:
            nx,ny=ix+dx,iy+dy
            if self.in_bounds(nx,ny) and not self.is_occ(nx,ny):
                yield nx,ny,c

    def dijkstra(self,start,goal):
        sx,sy=start; gx,gy=goal
        dist={(sx,sy):0.0}; prev={}
        pq=[(0.0,sx,sy)]; vis=set()
        while pq:
            d,x,y=heapq.heappop(pq)
            if (x,y) in vis: continue
            vis.add((x,y))
            if (x,y)==(gx,gy): break
            for nx,ny,c in self.neighbors(x,y):
                nd=d+c
                if nd<dist.get((nx,ny),float('inf')):
                    dist[(nx,ny)]=nd; prev[(nx,ny)]=(x,y)
                    heapq.heappush(pq,(nd,nx,ny))
        if (gx,gy) not in dist: return None
        path=[]; cur=(gx,gy)
        while cur in prev or cur==(sx,sy):
            path.append(cur)
            if cur==(sx,sy): break
            cur=prev[cur]
        path.reverse()
        return path

    def publish_path(self,pts,frame_id):
        path=Path()
        path.header=Header()
        path.header.stamp=self.get_clock().now().to_msg()
        path.header.frame_id=frame_id
        path.poses=[]
        for x,y in pts:
            p=PoseStamped()
            p.header=path.header
            p.pose.position.x=x
            p.pose.position.y=y
            p.pose.orientation.w=1.0
            path.poses.append(p)
        self.pub_path.publish(path)

    def line_points(self,sx,sy,gx,gy,frame_id):
        pts=[]
        for i in range(self.straight_steps+1):
            a=i/self.straight_steps
            x=(1-a)*sx+a*gx
            y=(1-a)*sy+a*gy
            pts.append((x,y))
        if self.smooth_enable and len(pts)>=2:
            pts=_catmull_rom_centripetal(pts,self.sps,False)
        return pts

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
            if _dist(out[-1],p)>=min_d:
                out.append(p)
        if out[-1]!=pts[-1]:
            out.append(pts[-1])
        return out

    def shortcut_path(self,pts):
        if len(pts)<=2: return pts
        grid_pts=[self.world_to_grid(x,y) for x,y in pts]
        out=[pts[0]]
        i=0
        while i<len(grid_pts)-1:
            j=len(grid_pts)-1
            while j>i+1 and not self.grid_line_clear(grid_pts[i],grid_pts[j]):
                j-=1
            out.append(pts[j])
            i=j
        return out

    def grid_line_clear(self,a,b):
        x0,y0=a; x1,y1=b
        dx=abs(x1-x0); dy=abs(y1-y0)
        sx=1 if x0<x1 else -1
        sy=1 if y0<y1 else -1
        err=dx-dy
        x,y=x0,y0
        while True:
            if not self.in_bounds(x,y) or self.is_occ(x,y):
                return False
            if x==x1 and y==y1:
                break
            e2=2*err
            if e2>-dy:
                err-=dy; x+=sx
            if e2<dx:
                err+=dx; y+=sy
        return True

def main(args=None):
    rclpy.init(args=args)
    node=DijkstraPlannerSmooth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
