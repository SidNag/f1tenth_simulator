import sys, numpy as np
from PIL import Image
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
img=np.array(Image.open('/home/sid2404/ece484/f1tenth_simulator/f1tenth_gym_ros/maps/levine.png').convert('L')); r=0.05; o=-51.224998; H=img.shape[0]
wp=np.loadtxt(sys.argv[1],delimiter=',',skiprows=1)
fig,ax=plt.subplots(figsize=(11,5.5),dpi=110)
ax.imshow(img,cmap='gray',extent=[o,o+H*r,o,o+H*r],origin='upper')
for f,lab,c in zip(sys.argv[3::2],sys.argv[4::2],['tab:blue','tab:orange','tab:green']):
    tr=np.loadtxt(f,delimiter=',',skiprows=1); ax.plot(tr[:,0],tr[:,1],color=c,lw=1,label=lab)
ax.plot(wp[:,0],wp[:,1],'--',color='tab:red',lw=1.2,label=f'levine_lap.csv ({len(wp)} pts, 0.1 m)')
ax.plot(*wp[0],'go',ms=7,label='start (0, -0.17)'); ax.annotate('',xy=wp[25],xytext=wp[0],arrowprops=dict(arrowstyle='->',color='g',lw=2))
ax.set_xlim(-16.5,12.5); ax.set_ylim(-2.5,11); ax.set_aspect('equal'); ax.legend(loc='center',fontsize=8)
ax.set_title('Levine pure pursuit waypoints (map frame, counter-clockwise)'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
fig.tight_layout(); fig.savefig(sys.argv[2])
