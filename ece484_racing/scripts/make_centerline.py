import sys, numpy as np
from PIL import Image
from scipy import ndimage
img=np.array(Image.open('/home/sid2404/ece484/f1tenth_simulator/f1tenth_gym_ros/maps/levine.png').convert('L')); r=0.05; o=-51.224998; H=img.shape[0]
white=img>250
edt=ndimage.distance_transform_edt(white)*r     # clearance inside the white corridors
def clear(p):
    c=np.clip(np.round((p[:,0]-o)/r).astype(int),0,H-1); rw=np.clip(np.round(H-1-(p[:,1]-o)/r).astype(int),0,H-1); return edt[rw,c]
# corridor centers measured from the map
yb,yt,xl,xr=-0.17,8.63,-13.70,9.71
R=float(sys.argv[1]) if len(sys.argv)>1 else 1.4
# counter-clockwise rounded rectangle, starting at (0, yb) heading +x
def arc(cx,cy,a0,a1,n=40):
    t=np.linspace(a0,a1,n); return np.column_stack([cx+R*np.cos(t),cy+R*np.sin(t)])
def line(p0,p1): 
    n=max(2,int(np.hypot(*(np.subtract(p1,p0)))/0.02)); return np.linspace(p0,p1,n)
pts=np.vstack([
 line((0,yb),(xr-R,yb)), arc(xr-R,yb+R,-np.pi/2,0),
 line((xr,yb+R),(xr,yt-R)), arc(xr-R,yt-R,0,np.pi/2),
 line((xr-R,yt),(xl+R,yt)), arc(xl+R,yt-R,np.pi/2,np.pi),
 line((xl,yt-R),(xl,yb+R)), arc(xl+R,yb+R,np.pi,1.5*np.pi),
 line((xl+R,yb),(0,yb))])
# resample at 0.1 m
s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(pts,axis=0),axis=1))]; keep=np.r_[True,np.diff(s)>1e-9]; pts,s=pts[keep],s[keep]
ss=np.arange(0,s[-1]-0.05,0.1); res=np.column_stack([np.interp(ss,s,pts[:,j]) for j in range(2)])
cl=clear(res)
print(f"R={R}: {len(res)} pts, length {s[-1]:.1f} m, clearance min {cl.min():.2f} m mean {cl.mean():.2f} m, steer at R {np.degrees(np.arctan(0.33/R)):.1f} deg")
if len(sys.argv)>2:
    np.savetxt(sys.argv[2],res,delimiter=',',header='x,y',comments='',fmt='%.4f'); print("wrote",sys.argv[2])
