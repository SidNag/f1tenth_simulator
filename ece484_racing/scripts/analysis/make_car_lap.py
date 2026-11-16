import numpy as np, yaml, datetime as dt
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree
P=np.loadtxt('map_1351_pose.csv',delimiter=',',skiprows=1); t,x,y=P[:,0],P[:,1],P[:,2]; XY=np.column_stack([x,y])
v=np.r_[0,np.hypot(np.diff(x),np.diff(y))/np.maximum(np.diff(t),1e-3)]
T=lambda s: dt.datetime.fromtimestamp(s).strftime('%H:%M:%S')
# mapping drive: moving samples before the map save (13:54:40)
tsave=t[0]+190
mv=np.where((t<tsave))[0]
seg=XY[mv]; s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(seg,axis=0),axis=1))]
# lap 1 of the outer loop: leave the left vertex at ~20-40 s, come back near it at ~130-170 s
ts=t[mv]-t[0]
A=np.where((ts>15)&(ts<45)&(v[mv]>0.3))[0]; B=np.where((ts>125)&(ts<175)&(v[mv]>0.3))[0]
D=np.linalg.norm(seg[A][:,None,:]-seg[B][None,:,:],axis=2); ia,ib=np.unravel_index(np.argmin(D),D.shape)
istart,iend=A[ia],B[ib]
lap=seg[istart:iend+1]
print(f"lap: {T(t[mv][istart])} -> {T(t[mv][iend])}, raw length {s[iend]-s[istart]:.1f} m, closure gap {np.linalg.norm(lap[0]-lap[-1]):.3f} m")
# thin to >= 2 cm, light circular smoothing (~0.6 m window), resample 0.1 m
keep=[0]
for i in range(1,len(lap)):
    if np.linalg.norm(lap[i]-lap[keep[-1]])>=0.02: keep.append(i)
lap=lap[keep]
ss=np.r_[0,np.cumsum(np.linalg.norm(np.diff(lap,axis=0),axis=1))]
u=np.arange(0,ss[-1],0.02); L=np.column_stack([np.interp(u,ss,lap[:,j]) for j in range(2)])
w=31; ext=np.vstack([L[-w:],L,L[:w]]); k=np.ones(w)/w
S=np.column_stack([np.convolve(ext[:,j],k,'same')[w:-w] for j in range(2)])
C=np.vstack([S,S[:1]]); cs=np.r_[0,np.cumsum(np.linalg.norm(np.diff(C,axis=0),axis=1))]
q=np.arange(0,cs[-1]-0.05,0.1); W=np.column_stack([np.interp(q,cs,C[:,j]) for j in range(2)])
print(f"waypoints: {len(W)} pts @0.1 m, length {cs[-1]:.1f} m, max smoothing shift {np.max(np.linalg.norm(S-L,axis=1)):.3f} m")
np.savetxt('/home/sid2404/ece484/car_data/analysis/lap1_slam.csv',W,delimiter=',',header='x,y',comments='',fmt='%.4f')
# clearance vs saved map
my=yaml.safe_load(open('/home/sid2404/ece484/car_data/sid/maps/track.yaml')); img=np.array(Image.open('/home/sid2404/ece484/car_data/sid/maps/track.pgm'))
r=my['resolution']; ox,oy=my['origin'][:2]; H=img.shape[0]
occ=img<50; dist=ndimage.distance_transform_edt(~occ)*r
def clr(p):
    c=np.clip(np.round((p[:,0]-ox)/r).astype(int),0,img.shape[1]-1); rw=np.clip(np.round(H-1-(p[:,1]-oy)/r).astype(int),0,H-1); return dist[rw,c]
cl=clr(W); print(f"clearance to occupied cells: min {cl.min():.2f} m, 5th pct {np.percentile(cl,5):.2f} m, mean {cl.mean():.2f} m")
# consistency of the post-save waypoint pass with the lap
wp_pass=XY[(t>t[0]+290)&(t<t[0]+342)&(v>0.3)]
d,_=cKDTree(W).query(wp_pass); print(f"post-save waypoint pass: {len(wp_pass)} pts, distance to lap: median {np.median(d):.2f} m, max {d.max():.2f} m")
# plot
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,9),dpi=110)
ax.imshow(img,cmap='gray',extent=[ox,ox+img.shape[1]*r,oy,oy+H*r],origin='upper')
ax.plot(XY[t<tsave][:,0],XY[t<tsave][:,1],color='0.65',lw=0.6,label='mapping drive (map_1351)')
ax.plot(wp_pass[:,0],wp_pass[:,1],color='tab:blue',lw=1.0,label='post-save pass (partial, drifted)')
ax.plot(W[:,0],W[:,1],color='tab:red',lw=1.6,label=f'car_track.csv ({len(W)} pts, {cs[-1]:.0f} m)')
ax.plot(*W[0],'go',ms=8,label='start'); ax.annotate('',xy=W[20],xytext=W[0],arrowprops=dict(arrowstyle='->',color='g',lw=2))
pad=3; ax.set_xlim(W[:,0].min()-pad,W[:,0].max()+pad); ax.set_ylim(W[:,1].min()-pad,W[:,1].max()+pad)
ax.set_aspect('equal'); ax.legend(loc='lower left',fontsize=8); ax.set_title(f'Car track waypoints (map frame) - min wall clearance {cl.min():.2f} m')
fig.tight_layout(); fig.savefig('/home/sid2404/ece484/car_data/analysis/lap1_slam.png')
h=np.degrees(np.arctan2(W[3,1]-W[0,1],W[3,0]-W[0,0])); print(f"start ({W[0,0]:.2f},{W[0,1]:.2f}) heading {h:.1f} deg")
