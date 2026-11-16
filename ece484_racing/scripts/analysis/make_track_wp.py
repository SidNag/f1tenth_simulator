"""Clockwise centerline waypoints for the car track (track_fixed map).

The corridor is a ring around a rectangular island. Fit the island's min-area
rectangle, measure per-side corridor width by ray-casting outward from the island
(median over the side, so side openings/rooms don't count), and build a rounded
rectangle on the corridor centerline with corner radius R (> car min radius 1.27 m).
Works for any map of this track (rerun on a new map):
usage: python make_track_wp.py <map.yaml> <out.csv> <out.png> [pose.csv|-] [R|auto] [bias] [min_clear] [max_steer]
  R: corner radius (default 2.0; 'auto' = largest R in 1.4..3.0 with wall clearance >= min_clear (0.35 m)),
  bias: shift all straights toward the outer wall (default 0.15 m; 'auto' picks the smallest that works),
  so corners enter wide / cut the inner corner / exit wide. Corners 0.7 m/s, straights 1.0 m/s.
  pose.csv: optional t,x,y,yaw SLAM pose (scripts/analysis/extract_pose.py) for a cross-check overlay
Also writes <map>.start next to the map: AMCL start pose "x:=0.0 y:=0.0 yaw:=0.0" = exactly where and how
the car sat when the mapping run started (slam_toolbox map origin).
"""
import sys, numpy as np, yaml, cv2
Y = yaml.safe_load(open(sys.argv[1])); im = cv2.imread(sys.argv[1].rsplit('/', 1)[0] + '/' + Y['image'], -1)
r = Y['resolution']; ox, oy = Y['origin'][:2]; H, W = im.shape
out_csv, out_png = sys.argv[2], sys.argv[3]
pose = np.loadtxt(sys.argv[4], delimiter=',', skiprows=1) if len(sys.argv) > 4 and sys.argv[4] != '-' else None
Rarg = sys.argv[5] if len(sys.argv) > 5 else '2.0'
Barg = sys.argv[6] if len(sys.argv) > 6 else '0.15'
MIN_CLEAR = float(sys.argv[7]) if len(sys.argv) > 7 else 0.35
MAX_STEER = float(sys.argv[8]) if len(sys.argv) > 8 else 0.23
WB = 0.34
def w2p(P): return np.column_stack([(P[:, 0] - ox) / r, H - 1 - (P[:, 1] - oy) / r])
def p2w(P): return np.column_stack([ox + P[:, 0] * r, oy + (H - 1 - P[:, 1]) * r])
# island = largest non-free component not touching the border
notfree = (im < 250).astype(np.uint8)
n, lab, st, _ = cv2.connectedComponentsWithStats(notfree, connectivity=4)
cand = [i for i in range(1, n) if st[i, 0] > 0 and st[i, 1] > 0 and st[i, 0] + st[i, 2] < W and st[i, 1] + st[i, 3] < H]
isl = max(cand, key=lambda i: st[i, 4]); m = (lab == isl).astype(np.uint8)
c = max(cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0], key=cv2.contourArea)
box = p2w(cv2.boxPoints(cv2.minAreaRect(c)))
ctr = box.mean(0)
# axes in world: u along the long side
e1, e2 = box[1] - box[0], box[2] - box[1]
u = e1 if np.linalg.norm(e1) > np.linalg.norm(e2) else e2
u = u / np.linalg.norm(u); v = np.array([-u[1], u[0]])
Q = (box - ctr) @ np.column_stack([u, v]); hl, hw = np.abs(Q[:, 0]).max(), np.abs(Q[:, 1]).max()
print(f"island {2*hl:.2f} x {2*hw:.2f} m, center {ctr.round(3)}, long-axis angle {np.degrees(np.arctan2(u[1], u[0])):.1f} deg")
occ = im < 50
nf = (im < 250).astype(np.uint8)
k_, lab_, st_, _ = cv2.connectedComponentsWithStats(nf, connectivity=8)
speck = np.isin(lab_, np.nonzero(st_[:, 4] <= 3)[0]) & (lab_ > 0)   # single-pixel specks (people, noise)
dt = cv2.distanceTransform((~(nf.astype(bool) & ~speck)).astype(np.uint8), cv2.DIST_L2, 5) * r  # clearance
def ray(p, d, maxd=3.0):
    for t in np.arange(0.05, maxd, r / 2):
        q = w2p((p + t * d)[None])[0]; ci, ri = int(round(q[0])), int(round(q[1]))
        if not (0 <= ri < H and 0 <= ci < W) or occ[ri, ci]: return t
    return np.nan
# per side: (normal, along, half-length along, half-depth)
sides = {'+v': (v, u, hl, hw), '-v': (-v, u, hl, hw), '+u': (u, v, hw, hl), '-u': (-u, v, hw, hl)}
off = {}
for k, (nrm, al, half, dep) in sides.items():
    ws = []
    for s in np.linspace(-half + 0.5, half - 0.5, 60):
        p = ctr + nrm * dep + al * s
        # island wall: step inward until occupied, then outward to the outer wall
        tin = ray(p, -nrm, 1.0); tout = ray(p, nrm, 3.0)
        if not np.isnan(tin) and not np.isnan(tout): ws.append((tout + tin, tin))
    ws = np.array(ws); wid = np.median(ws[:, 0]); tin = np.median(ws[:, 1])
    off[k] = wid / 2 - tin   # centerline offset from the box side
    print(f"side {k}: corridor width median {wid:.2f} m (p10 {np.percentile(ws[:,0],10):.2f}, p90 {np.percentile(ws[:,0],90):.2f}), center offset {off[k]:.2f} m from box")
def build(R, bias):
    xr, xl, yt, yb = hl + off['+u'] + bias, -hl - off['-u'] - bias, hw + off['+v'] + bias, -hw - off['-v'] - bias
    def arc(cx, cy, a0, a1, k=80):
        t = np.linspace(a0, a1, k); return np.column_stack([cx + R * np.cos(t), cy + R * np.sin(t)])
    def line(p0, p1):
        k = max(2, int(np.hypot(*np.subtract(p1, p0)) / 0.02)); return np.linspace(p0, p1, k)
    pts = np.vstack([line((0, yb), (xr - R, yb)), arc(xr - R, yb + R, -np.pi/2, 0),
                     line((xr, yb + R), (xr, yt - R)), arc(xr - R, yt - R, 0, np.pi/2),
                     line((xr - R, yt), (xl + R, yt)), arc(xl + R, yt - R, np.pi/2, np.pi),
                     line((xl, yt - R), (xl, yb + R)), arc(xl + R, yb + R, np.pi, 1.5*np.pi),
                     line((xl + R, yb), (0, yb))])
    P = ctr + pts[:, :1] * u + pts[:, 1:] * v
    area = 0.5 * np.sum(P[:-1, 0] * P[1:, 1] - P[1:, 0] * P[:-1, 1])
    if area > 0: P = P[::-1]                    # clockwise
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]; keep = np.r_[True, np.diff(s) > 1e-9]; P, s = P[keep], s[keep]
    ss = np.arange(0, s[-1] - 0.05, 0.1)
    return np.column_stack([np.interp(ss, s, P[:, j]) for j in range(2)]), s[-1]
def clearance(WP):
    q = w2p(WP); return dt[np.clip(q[:, 1].round().astype(int), 0, H-1), np.clip(q[:, 0].round().astype(int), 0, W-1)]
Rs = [float(Rarg)] if Rarg != 'auto' else np.round(np.arange(3.0, 1.39, -0.05), 2)
Bs = [float(Barg)] if Barg != 'auto' else np.round(np.arange(0.0, 0.41, 0.02), 2)
best = None
for R in Rs:
    for bias in Bs:
        WP, L = build(R, bias); cl = clearance(WP)
        if cl.min() >= MIN_CLEAR or (len(Rs) == 1 and len(Bs) == 1):
            best = (R, bias); break
    if best: break
if best is None:
    raise SystemExit(f"no R/bias with clearance >= {MIN_CLEAR} m")
R, bias = best; WP, L = build(R, bias); s = np.r_[0.0, L]
print(f"chosen R={R:.2f} m, bias {bias:.2f} m toward outer wall; steer needed {np.arctan(WB/R):.3f} rad (limit {MAX_STEER}, margin {100*(1-np.arctan(WB/R)/MAX_STEER):.0f}%)")
# curvature -> speed column: 1.0 straight, 0.8 in/near corners
nx = np.roll(WP, -1, 0); pv = np.roll(WP, 1, 0)
a, b = nx - WP, WP - pv
curv = np.abs(np.cross(b, a)) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(nx - pv, axis=1) / 2 + 1e-9)
corner = np.convolve(np.r_[curv[-15:], curv, curv[:15]] > 0.2, np.ones(31), 'same')[15:-15] > 0
vel = np.where(corner, 0.7, 1.0)
q = w2p(WP); cl = clearance(WP)
hd = np.arctan2(WP[1, 1] - WP[0, 1], WP[1, 0] - WP[0, 0])
rmin = 1 / max(curv.max(), 1e-9)
print(f"R={R}: {len(WP)} wps, lap {L:.1f} m, min radius (discrete) {rmin:.2f} m -> steer {np.arctan(WB/rmin):.3f} rad; clearance min {cl.min():.2f} mean {cl.mean():.2f} m (corners min {cl[corner].min():.2f}, straights min {cl[~corner].min():.2f})")
print(f"start wp0 = ({WP[0,0]:.3f}, {WP[0,1]:.3f}) heading {hd:.3f} rad ({np.degrees(hd):.1f} deg)")
np.savetxt(out_csv, np.column_stack([WP, vel]), delimiter=',', header='x,y,v', comments='', fmt='%.4f')
i0 = int(np.argmin(np.linalg.norm(WP, axis=1))); j0 = (i0 + 1) % len(WP)
yaw0 = float(np.arctan2(WP[j0, 1] - WP[i0, 1], WP[j0, 0] - WP[i0, 0]))
start_file = sys.argv[1][:-5] + '.start'
open(start_file, 'w').write("x:=0.0 y:=0.0 yaw:=0.0\n")  # exact pose where the mapping run started
print(f"map origin (= mapping start pose, yaw 0) is {np.linalg.norm(WP[i0]):.2f} m from the path, path heading there {yaw0:.3f} rad -> {start_file}")
if pose is not None:
    from scipy.spatial import cKDTree
    d, _ = cKDTree(WP).query(pose[:, 1:3]); t = pose[:, 0] - pose[0, 0]
    for lo, hi in [(0, 1e9), (296, 346)]:
        mm = (t >= lo) & (t <= hi); print(f"SLAM pose t {lo}-{min(hi, t[-1]):.0f}s: dist to wps median {np.median(d[mm]):.2f} p90 {np.percentile(d[mm],90):.2f} max {d[mm].max():.2f} m")
# overlay
img = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR); img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
if pose is not None:
    cv2.polylines(img, [(w2p(pose[:, 1:3]) * 2).astype(np.int32)], False, (200, 160, 0), 1)
cv2.polylines(img, [(q * 2).astype(np.int32)], True, (0, 0, 220), 2)
for i in range(0, len(WP), 40):  # direction arrows
    j = (i + 3) % len(WP); cv2.arrowedLine(img, tuple((q[i]*2).astype(int)), tuple((q[j]*2).astype(int)), (0, 140, 0), 2, tipLength=0.6)
cv2.circle(img, tuple((q[0]*2).astype(int)), 7, (0, 200, 0), -1)
cv2.putText(img, 'START wp0', tuple((q[0]*2).astype(int) + [10, 25]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 120, 0), 2)
cv2.imwrite(out_png, img); print("wrote", out_csv, out_png)
