"""Collision-certified corner rounding and monotonically tracked arc length."""
import bisect
import math


def triangle_free(grid,triangle):
    """Test the entire Bezier convex hull against closed occupied grid cells."""
    lo=grid.cell(tuple(min(p[k] for p in triangle) for k in (0,1)))
    hi=grid.cell(tuple(max(p[k] for p in triangle) for k in (0,1)))
    axes=[(1,0),(0,1)]
    for a,b in zip(triangle,triangle[1:]+triangle[:1]):
        axes.append((a[1]-b[1],b[0]-a[0]))
    for i in range(lo[0]-1,hi[0]+2):
        for j in range(lo[1]-1,hi[1]+2):
            if grid.free((i,j)):continue
            x,y=grid.cell_center((i,j));h=grid.resolution/2
            square=[(x-h,y-h),(x+h,y-h),(x+h,y+h),(x-h,y+h)]
            separated=False
            for ax,ay in axes:
                if abs(ax)+abs(ay)<1e-12:continue
                a=[x*ax+y*ay for x,y in triangle];b=[x*ax+y*ay for x,y in square]
                if max(a)<min(b)-1e-10 or max(b)<min(a)-1e-10:
                    separated=True;break
            if not separated:return False
    return True


def rounded_path(grid,route,distance=.9):
    """Only replace a corner if its whole quadratic curve hull is free."""
    points=[tuple(route[0])];stops=[];rounded=0
    for a,b,c in zip(route,route[1:],route[2:]):
        before,after=math.dist(a,b),math.dist(b,c)
        if min(before,after)<1e-8:continue
        u=((b[0]-a[0])/before,(b[1]-a[1])/before)
        v=((c[0]-b[0])/after,(c[1]-b[1])/after)
        turn=math.acos(max(-1,min(1,u[0]*v[0]+u[1]*v[1])))
        if turn<.02:continue
        accepted=False
        if turn<math.radians(150):
            for scale in (1.,.75,.5,.25):
                radius=min(distance,before*.35,after*.35)*scale
                if radius<.12:continue
                entry=(b[0]-u[0]*radius,b[1]-u[1]*radius)
                exit=(b[0]+v[0]*radius,b[1]+v[1]*radius)
                if not triangle_free(grid,[entry,tuple(b),exit]):continue
                curve=[((1-t)**2*entry[0]+2*(1-t)*t*b[0]+t*t*exit[0],
                        (1-t)**2*entry[1]+2*(1-t)*t*b[1]+t*t*exit[1])
                       for t in [i/max(4,math.ceil(2*radius/.08))
                                 for i in range(max(4,math.ceil(2*radius/.08))+1)]]
                if not all(grid.segment_free(p,q) for p,q in zip([points[-1]]+curve,curve)):
                    continue
                points.extend(curve);rounded+=1;accepted=True;break
        if not accepted:
            points.append(tuple(b));stops.append(len(points)-1)
    points.append(tuple(route[-1]))
    if not all(grid.segment_free(p,q) for p,q in zip(points,points[1:])):
        raise ValueError('Unsafe processed path')
    return points,stops,rounded


class ArcPath:
    def __init__(self,points,stops=()):
        self.points=[tuple(points[0])];remap={0:0}
        for index,point in enumerate(points[1:],1):
            if math.dist(self.points[-1],point)>1e-8:self.points.append(tuple(point))
            remap[index]=len(self.points)-1
        self.stations=[0.]
        for a,b in zip(self.points,self.points[1:]):self.stations.append(self.stations[-1]+math.dist(a,b))
        self.length=self.stations[-1]
        self.stops=[self.stations[remap[i]] for i in stops]
        self.curvatures=[]
        for i,(a,b,c) in enumerate(zip(self.points,self.points[1:],self.points[2:]),1):
            ab,bc,ac=math.dist(a,b),math.dist(b,c),math.dist(a,c)
            cross=abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
            kappa=2*cross/(ab*bc*ac) if min(ab,bc,ac)>1e-8 else 0.
            if kappa>.01:self.curvatures.append((self.stations[i],kappa))

    def point(self,station):
        if len(self.points)==1:return self.points[0]
        station=max(0.,min(station,self.length))
        i=min(len(self.points)-2,max(0,bisect.bisect_right(self.stations,station)-1))
        t=(station-self.stations[i])/(self.stations[i+1]-self.stations[i])
        a,b=self.points[i:i+2]
        return a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])

    def project(self,point,previous=0.,limit=None):
        if len(self.points)==1:return 0.,math.dist(point,self.points[0])
        limit=self.length if limit is None else limit
        best=(math.inf,previous)
        for i,(a,b) in enumerate(zip(self.points,self.points[1:])):
            if self.stations[i+1]<previous-.15 or self.stations[i]>min(limit,previous+2.):continue
            length=self.stations[i+1]-self.stations[i]
            t=max(0.,min(1.,((point[0]-a[0])*(b[0]-a[0])+(point[1]-a[1])*(b[1]-a[1]))/length**2))
            station=min(limit,max(previous,self.stations[i]+t*length))
            distance=math.dist(point,self.point(station))
            if (distance,station)<best:best=distance,station
        return best[1],best[0]
