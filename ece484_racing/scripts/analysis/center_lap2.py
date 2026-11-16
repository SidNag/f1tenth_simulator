import numpy as np, yaml
from PIL import Image
from scipy import ndimage
my=yaml.safe_load(open('/home/sid2404/ece484/car_data/sid/maps/track.yaml')); img=np.array(Image.open('/home/sid2404/ece484/car_data/sid/maps/track.pgm'))
r=my['resolution']; ox,oy=my['origin'][:2]; H,Wd=img.shape
free=img>250; occ=img<50
E=ndimage.gaussian_filter(ndimage.distance_transform_edt(free)*r,2)   # smoothed clearance field
gy,gx=np.gradient(E)            # per pixel (rows go down = -y)
Eocc=ndimage.distance_transform_edt(~occ)*r
def pix(p): return np.clip(np.round(H-1-(p[:,1]-oy)/r).astype(int),0,H-1), np.clip(np.round((p[:,0]-ox)/r).astype(int),0,Wd-1)
def sample(F,p): rw,c=pix(p); return F[rw,c]
raw=np.loadtxt('lap1_raw.csv',delimiter=',',skiprows=1) if False else None
W=np.loadtxt('/home/sid2404/ece484/car_data/analysis/lap1_slam.csv',delimiter=',',skiprows=1)
def resample(P,step):
    C=np.vstack([P,P[:1]]); s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(C,axis=0),axis=1))]
    q=np.arange(0,s[-1]-step/2,step); return np.column_stack([np.interp(q,s,C[:,j]) for j in range(2)])
def smooth(P,w):
    ext=np.vstack([P[-w:],P,P[:w]]); k=np.ones(w)/w; return np.column_stack([np.convolve(ext[:,j],k,'same')[w:-w] for j in range(2)])
P=resample(smooth(resample(W,0.1),21),0.1)
def smooth1(v,w):
    ext=np.r_[v[-w:],v,v[:w]]; return np.convolve(ext,np.ones(w)/w,'same')[w:-w]
for it in range(300):
    T=np.roll(P,-1,axis=0)-np.roll(P,1,axis=0); T/=np.linalg.norm(T,axis=1)[:,None]; N=np.column_stack([-T[:,1],T[:,0]])
    rw,c=pix(P); g=np.column_stack([gx[rw,c],-gy[rw,c]])
    dn=smooth1((g*N).sum(1),15)                       # smooth the normal push, not the positions
    P=P+np.clip(dn/ (np.abs(dn).max()+1e-9)*0.03,-0.03,0.03)[:,None]*N
    if it%30==29: P=resample(P,0.1)
P=resample(smooth(P,13),0.1)
co=sample(Eocc,P); cf=sample(ndimage.distance_transform_edt(free)*r,P)
a=np.roll(P,-5,axis=0)-P; b=P-np.roll(P,5,axis=0)
ang=np.arctan2(a[:,0]*b[:,1]-a[:,1]*b[:,0],(a*b).sum(1)); kap=np.abs(ang)/0.5; Rmin=0.34/np.tan(0.26)
print(f"centerline: {len(P)} pts, {len(P)*0.1:.1f} m | clearance to occupied min {co.min():.2f} p5 {np.percentile(co,5):.2f} mean {co.mean():.2f} | to non-free min {cf.min():.2f} | min radius {1/kap.max():.2f} m (car {Rmin:.2f}), pts tighter: {(kap>1/Rmin).sum()}")
np.savetxt('/home/sid2404/ece484/f1tenth_simulator/ece484_racing/waypoints/car_track.csv',P,delimiter=',',header='x,y',comments='',fmt='%.4f')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,9),dpi=110); ax.imshow(img,cmap='gray',extent=[ox,ox+Wd*r,oy,oy+H*r],origin='upper')
ax.plot(W[:,0],W[:,1],color='tab:orange',lw=0.8,alpha=0.8,label='SLAM-estimated lap 1 (offset from final map)')
ax.plot(P[:,0],P[:,1],color='tab:red',lw=1.8,label=f'car_track.csv: corridor centerline ({len(P)} pts, {len(P)*0.1:.0f} m)')
ax.plot(*P[0],'go',ms=8,label='start'); ax.annotate('',xy=P[20],xytext=P[0],arrowprops=dict(arrowstyle='->',color='g',lw=2))
ax.set_xlim(P[:,0].min()-3,P[:,0].max()+3); ax.set_ylim(P[:,1].min()-3,P[:,1].max()+3); ax.set_aspect('equal'); ax.legend(loc='lower left',fontsize=8)
ax.set_title(f'Car track waypoints - clearance to walls min {co.min():.2f} m, mean {co.mean():.2f} m'); fig.tight_layout()
fig.savefig('/mnt/c/Vaults/Obsidian Vault/ECE484/FinalProject/car_waypoints.png')
h=np.degrees(np.arctan2(P[3,1]-P[0,1],P[3,0]-P[0,0])); print(f"start ({P[0,0]:.2f},{P[0,1]:.2f}) heading {h:.1f} deg")
