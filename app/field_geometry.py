"""Operating-area validation independent of the temporary planning window."""
import math


def intersects_operating_area(center,size,yaw,bounds):
    cx,cy=center;sx,sy=size[:2]
    c,s=math.cos(yaw),math.sin(yaw)
    box=[(cx+c*x-s*y,cy+s*x+c*y)
         for x,y in [(-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)]]
    x0,y0,x1,y1=bounds
    area=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    for ax,ay in [(1,0),(0,1),(c,s),(-s,c)]:
        a=[x*ax+y*ay for x,y in box];b=[x*ax+y*ay for x,y in area]
        if max(a)<=min(b)+1e-10 or max(b)<=min(a)+1e-10:
            return False
    return True
