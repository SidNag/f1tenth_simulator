import numpy as np, yaml
from PIL import Image
from scipy import ndimage
my=yaml.safe_load(open('/home/sid2404/ece484/car_data/sid/maps/track.yaml')); img=np.array(Image.open('/home/sid2404/ece484/car_data/sid/maps/track.pgm'))
r=my['resolution']; ox,oy=my['origin'][:2]; H,Wd=img.shape
free=img>250; occ=img<50
edt_free=ndimage.distance_transform_edt(free)*r      # distance to nearest non-free (wall or unknown)
edt_occ=ndimage.distance_transform_edt(~occ)*r       # distance to nearest occupied cell
def sample(E,p):
    c=np.clip(np.round((p[:,0]-ox)/r).astype(int),0,Wd-1); rw=np.clip(np.round(H-1-(p[:,1]-oy)/r).astype(int),0,H-1); return E[rw,c]
W=np.loadtxt('/home/sid2404/ece484/car_data/analysis/lap1_slam.csv',delimiter=',',skiprows=1)
def resample(P,step,closed=True):
    C=np.vstack([P,P[:1]]) if closed else P; s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(C,axis=0),axis=1))]
    q=np.arange(0,s[-1]-(step/2 if closed else 0),step); return np.column_stack([np.interp(q,s,C[:,j]) for j in range(2)])
def smooth(P,w):
    ext=np.vstack([P[-w:],P,P[:w]]); k=np.ones(w)/w; return np.column_stack([np.convolve(ext[:,j],k,'same')[w:-w] for j in range(2)])
P=resample(W,0.1)
for it in range(4):
    T=np.roll(P,-1,axis=0)-np.roll(P,1,axis=0); T/=np.linalg.norm(T,axis=1)[:,None]; N=np.column_stack([-T[:,1],T[:,0]])
    offs=np.arange(-1.5,1.51,0.05)
    cand=P[:,None,:]+offs[None,:,None]*N[:,None,:]
    sc=sample(edt_free,cand.reshape(-1,2)).reshape(len(P),len(offs)) - 0.15*np.abs(offs)[None,:]   # prefer small moves
    best=offs[np.argmax(sc,axis=1)]
    best=ndimage.median_filter(best,size=41,mode='wrap')            # reject outliers (spikes)
    best=ndimage.uniform_filter1d(best,size=31,mode='wrap')         # smooth offsets
    P=P+best[:,None]*N
    P=resample(smooth(P,9),0.1)
P=resample(smooth(P,9),0.1)
cf=sample(edt_free,P); co=sample(edt_occ,P)
# curvature check vs steering limit (wheelbase 0.34, max_steer 0.26 -> R_min = 0.34/tan(0.26) = 1.28 m)
a=np.roll(P,-5,axis=0)-P; b=P-np.roll(P,5,axis=0)
ang=np.arctan2(a[:,0]*b[:,1]-a[:,1]*b[:,0],(a*b).sum(1)); kappa=np.abs(ang)/0.5
print(f"centered lap: {len(P)} pts, length {len(P)*0.1:.1f} m")
print(f"clearance to occupied: min {co.min():.2f} m, 5th pct {np.percentile(co,5):.2f}, mean {co.mean():.2f}")
print(f"clearance to any non-free (incl. unknown): min {cf.min():.2f} m, mean {cf.mean():.2f}")
print(f"min turn radius {1/kappa.max():.2f} m (car min ~{0.34/np.tan(0.26):.2f} m); points tighter than that: {(kappa>np.tan(0.26)/0.34).sum()}")
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
