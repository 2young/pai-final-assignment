"""Shared oriented vehicle geometry and conservative continuous-motion checks."""
import math
from pose_provider import angle_difference


def integrate(pose, v, w, dt):
    x,y,yaw=pose
    if abs(w)<1e-10:
        return x+v*math.cos(yaw)*dt,y+v*math.sin(yaw)*dt,yaw+w*dt
    return (x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),
            y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)),yaw+w*dt)


def rectangle(center, size, yaw=0.):
    c,s=math.cos(yaw),math.sin(yaw)
    return (*center,size[0]/2,size[1]/2,c,s)


def separation(a,b):
    """SAT separation lower bound (positive only for disjoint rectangles)."""
    ax,ay,ah,aw,ac,ass=a;bx,by,bh,bw,bc,bs=b
    dx,dy=bx-ax,by-ay
    best=-math.inf
    for ux,uy in ((ac,ass),(-ass,ac),(bc,bs),(-bs,bc)):
        ra=ah*abs(ux*ac+uy*ass)+aw*abs(-ux*ass+uy*ac)
        rb=bh*abs(ux*bc+uy*bs)+bw*abs(-ux*bs+uy*bc)
        best=max(best,abs(dx*ux+dy*uy)-ra-rb)
    return best


class FootprintScene:
    def __init__(self,config,budget=None):
        self.config=config;self.budget=budget
        self.bounds=config['bounds']
        left,bottom,right,top=config['robot']['footprint_bounds']
        if not all(math.isfinite(v) for v in (left,bottom,right,top)) or not left<right or not bottom<top:
            raise ValueError('Invalid robot footprint_bounds')
        self.offset=((left+right)/2,(bottom+top)/2)
        self.size=(right-left,top-bottom)
        self.margin=config['robot']['margin']
        if not math.isfinite(self.margin) or self.margin<0:
            raise ValueError('Invalid robot margin')
        self.radius=max(math.hypot(x,y) for x in (left,right) for y in (bottom,top))+2*self.margin
        self.obstacles=[]
        for obj in config['walls']+config['objects']:
            # Runtime metadata retains both the conservative AABB and the
            # original oriented box. Static configs have zero-yaw boxes.
            box=rectangle(obj['center'],obj.get('body_size',obj['size']),obj.get('yaw',0.))
            _,_,h,w,c,s=box
            self.obstacles.append((obj.get('id'),box,h*abs(c)+w*abs(s),h*abs(s)+w*abs(c)))

    def check_budget(self,expansion=False):
        if self.budget is not None:
            self.budget.expand() if expansion else self.budget.check()

    def box(self,pose,extra=0.):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        dx,dy=self.offset;p=self.margin+extra
        return x+c*dx-s*dy,y+s*dx+c*dy,self.size[0]/2+p,self.size[1]/2+p,c,s

    def clearance(self,pose,extra=0.,ignore=None):
        if not all(math.isfinite(v) for v in pose):return -math.inf
        box=self.box(pose,extra);x,y,h,w,c,s=box
        hx,hy=h*abs(c)+w*abs(s),h*abs(s)+w*abs(c)
        x0,y0,x1,y1=self.bounds
        gap=min(x-hx-x0,x1-x-hx,y-hy-y0,y1-y-hy)
        for name,other,ohx,ohy in self.obstacles:
            if ignore is not None and name==ignore:continue
            # Cheap world-axis separation is itself a safe lower bound.
            ox,oy,oh,ow,oc,os=other
            broad=max(abs(x-ox)-hx-ohx,abs(y-oy)-hy-ohy)
            if broad>=gap:continue
            gap=min(gap,separation(box,other))
        return gap

    def free(self,pose,extra=0.,ignore=None):
        if not all(math.isfinite(v) for v in pose):return False
        box=self.box(pose,extra);x,y,h,w,c,s=box
        hx,hy=h*abs(c)+w*abs(s),h*abs(s)+w*abs(c)
        x0,y0,x1,y1=self.bounds
        if not (x0+hx<x<x1-hx and y0+hy<y<y1-hy):return False
        for name,other,ohx,ohy in self.obstacles:
            if ignore is not None and name==ignore:continue
            if abs(x-other[0])>hx+ohx or abs(y-other[1])>hy+ohy:continue
            if separation(box,other)<=1e-9:return False
        return True

    def sweep(self,a,b,extra=0.,ignore=None):
        """Certify linear position/yaw interpolation, including between samples.

        Each interval is covered by its midpoint rectangle expanded by a bound
        on the maximum motion of any corner. This is not point sampling alone.
        """
        if not all(math.isfinite(v) for v in (*a,*b)):return False
        angle=angle_difference(b[2],a[2]);distance=math.dist(a[:2],b[:2])
        travel=distance+(self.radius+2*extra)*abs(angle)
        n=max(1,math.ceil(travel/.08));pad=travel/(2*n)
        for i in range(n):
            if i%16==0:self.check_budget()
            t=(i+.5)/n
            pose=(a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,a[2]+angle*t)
            if not self.free(pose,extra+pad,ignore):return False
        return True

    def motion_free(self,pose,v,w,duration,extra=0.,ignore=None):
        if not all(math.isfinite(x) for x in (*pose,v,w,duration)) or duration<0:return False
        travel=(abs(v)+(self.radius+2*extra)*abs(w))*duration
        n=max(1,math.ceil(travel/.08));pad=travel/(2*n)
        for i in range(n):
            if i%16==0:self.check_budget()
            if not self.free(integrate(pose,v,w,duration*(i+.5)/n),extra+pad,ignore):return False
        return True


def footprint_free(config,pose,target_id=None):
    return FootprintScene(config).free(pose,ignore=target_id)
