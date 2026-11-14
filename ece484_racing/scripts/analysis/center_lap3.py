import numpy as np, yaml, sys
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree
my=yaml.safe_load(open('/home/sid2404/ece484/car_data/sid/maps/track.yaml')); img=np.array(Image.open('/home/sid2404/ece484/car_data/sid/maps/track.pgm'))
r=my['resolution']; ox,oy=my['origin'][:2]; H,Wd=img.shape
free=img>250; occ=img<50
W=np.loadtxt('lap1_slam.csv',delimiter=',',skiprows=1)
def resample(P,step):
    C=np.vstack([P,P[:1]]); s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(C,axis=0),axis=1))]
    q=np.arange(0,s[-1]-step/2,step); return np.column_stack([np.interp(q,s,C[:,j]) for j in range(2)])
def smooth(P,w):
    ext=np.vstack([P[-w:],P,P[:w]]); k=np.ones(w)/w; return np.column_stack([np.convolve(ext[:,j],k,'same')[w:-w] for j in range(2)])
ref=resample(smooth(resample(W,0.1),31),0.1)                     # smooth reference loop
# world coords of every pixel centre
rows,cols=np.mgrid[0:H,0:Wd]; PX=np.column_stack([ox+cols.ravel()*r, oy+(H-1-rows.ravel())*r])
tree=cKDTree(ref); d,idx=tree.query(PX,distance_upper_bound=2.2)
near=(d<2.0).reshape(H,Wd)
corr=ndimage.binary_opening(free&near,iterations=4)   # keep the solid white band, drop speckle and ray streaks
lab,n=ndimage.label(corr); big=lab==np.argmax(np.bincount(lab.ravel())[1:])+1
sp=np.column_stack([ox+np.nonzero(big)[1]*r, oy+(H-1-np.nonzero(big)[0])*r])   # all corridor pixels
_,si=tree.query(sp)
# per reference index window, median of skeleton points
M=len(ref); C=np.full((M,2),np.nan)
for i in range(M):
    m=np.abs(((si-i+M//2)%M)-M//2)<=3
    if m.sum()>=10: C[i]=sp[m].mean(axis=0)            # centroid of the corridor cross-section
ok=~np.isnan(C[:,0]); print(f"corridor px {len(sp)}, ref pts {M}, covered {ok.mean()*100:.0f}%")
# fill gaps by interpolation along index
ii=np.arange(M)
for j in range(2): C[:,j]=np.interp(ii,ii[ok],C[ok,j],period=M)
P=resample(smooth(C,41),0.1); P=resample(smooth(P,31),0.1); P=resample(smooth(P,21),0.1)
E=ndimage.distance_transform_edt(~occ)*r
def sample(F,p):
    c=np.clip(np.round((p[:,0]-ox)/r).astype(int),0,Wd-1); rw=np.clip(np.round(H-1-(p[:,1]-oy)/r).astype(int),0,H-1); return F[rw,c]
co=sample(E,P); cf=sample(ndimage.distance_transform_edt(big)*r,P)   # cf: clearance inside the solid white band
a=np.roll(P,-5,axis=0)-P; b=P-np.roll(P,5,axis=0); ang=np.arctan2(a[:,0]*b[:,1]-a[:,1]*b[:,0],(a*b).sum(1)); kap=np.abs(ang)/0.5; Rmin=0.34/np.tan(0.26)
print(f"centerline {len(P)} pts {len(P)*0.1:.1f} m | to occupied: min {co.min():.2f} p5 {np.percentile(co,5):.2f} mean {co.mean():.2f} | to band edge: min {cf.min():.2f} mean {cf.mean():.2f} | min radius {1/kap.max():.2f} m (car {Rmin:.2f}), tighter pts {(kap>1/Rmin).sum()}")
np.savetxt('car_track_centerline.csv',P,delimiter=',',header='x,y',comments='',fmt='%.4f')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,9),dpi=110); ax.imshow(img,cmap='gray',extent=[ox,ox+Wd*r,oy,oy+H*r],origin='upper')
ax.plot(W[:,0],W[:,1],color='tab:orange',lw=0.8,alpha=0.8,label='SLAM-estimated lap 1 (offset from final map)')
ax.plot(P[:,0],P[:,1],color='tab:red',lw=1.8,label=f'car_track.csv: corridor centerline ({len(P)} pts, {len(P)*0.1:.0f} m)')
ax.plot(*P[0],'go',ms=8,label='start'); ax.annotate('',xy=P[20],xytext=P[0],arrowprops=dict(arrowstyle='->',color='g',lw=2))
ax.set_xlim(P[:,0].min()-3,P[:,0].max()+3); ax.set_ylim(P[:,1].min()-3,P[:,1].max()+3); ax.set_aspect('equal'); ax.legend(loc='lower left',fontsize=8)
ax.set_title(f'Car track waypoints - clearance to white-band edge min {cf.min():.2f} m, mean {cf.mean():.2f} m'); fig.tight_layout(); fig.savefig('car_waypoints.png')
h=np.degrees(np.arctan2(P[3,1]-P[0,1],P[3,0]-P[0,0])); print(f"start ({P[0,0]:.2f},{P[0,1]:.2f}) heading {h:.1f} deg")
