"""Extract map->base_link poses from a car bag by composing /tf map->odom and odom->base_link."""
import sys, math, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
bag, out = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/tf']))
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
mo = []; ob = []
while r.has_next():
    _, data, _ = r.read_next()
    for t in deserialize_message(data, TFMessage).transforms:
        st = t.header.stamp.sec + t.header.stamp.nanosec*1e-9
        tr = t.transform
        rec = (st, tr.translation.x, tr.translation.y, yaw(tr.rotation))
        if t.header.frame_id == 'map' and t.child_frame_id == 'odom': mo.append(rec)
        elif t.header.frame_id == 'odom' and t.child_frame_id == 'base_link': ob.append(rec)
mo = np.array(sorted(mo)); ob = np.array(sorted(ob))
print(f"map->odom msgs {len(mo)}, odom->base_link msgs {len(ob)}")
# for each odom->base_link sample use the latest map->odom at or before it
idx = np.searchsorted(mo[:, 0], ob[:, 0], side='right') - 1
ok = idx >= 0; ob = ob[ok]; m = mo[idx[ok]]
c, s = np.cos(m[:, 3]), np.sin(m[:, 3])
x = m[:, 1] + c*ob[:, 1] - s*ob[:, 2]; y = m[:, 2] + s*ob[:, 1] + c*ob[:, 2]; th = m[:, 3] + ob[:, 3]
np.savetxt(out, np.column_stack([ob[:, 0], x, y, th]), delimiter=',', header='t,x,y,yaw', comments='', fmt='%.4f')
print(f"poses {len(x)}, t {ob[0,0]:.1f}..{ob[-1,0]:.1f} ({ob[-1,0]-ob[0,0]:.0f} s)")
