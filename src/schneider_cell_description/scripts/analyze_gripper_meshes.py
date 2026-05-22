"""V23 gripper STL analysis.

Bbox each new_gripper_*.STL to decide which mesh is the closed housing
and which is the movable jaw. Same binary-STL parser as the fixture
analyzer.
"""
import struct, os, glob

MESH_DIR = r"C:\tmp\v23work\ros_ws\src\schneider_cell_description\meshes\gripper_new"

def parse_stl(path):
    with open(path, "rb") as f:
        d = f.read()
    n = struct.unpack("<I", d[80:84])[0]
    xs=[]; ys=[]; zs=[]
    for i in range(n):
        off = 84 + i*50 + 12
        for v in range(3):
            x,y,z = struct.unpack("<fff", d[off+v*12:off+v*12+12])
            xs.append(x); ys.append(y); zs.append(z)
    return {
        "tris": n,
        "x":(min(xs),max(xs)), "y":(min(ys),max(ys)), "z":(min(zs),max(zs)),
        "cx":(min(xs)+max(xs))/2, "cy":(min(ys)+max(ys))/2, "cz":(min(zs)+max(zs))/2,
    }

if __name__ == "__main__":
    files = sorted(glob.glob(os.path.join(MESH_DIR, "*.STL")))
    print(f"[V23 gripper] {len(files)} STLs in '{MESH_DIR}'\n")
    info = {}
    for p in files:
        s = parse_stl(p); name = os.path.basename(p); info[name] = s
        sx = s['x'][1]-s['x'][0]; sy = s['y'][1]-s['y'][0]; sz = s['z'][1]-s['z'][0]
        vol = sx*sy*sz
        print(f"{name}")
        print(f"  tris {s['tris']:5d}   bbox X[{s['x'][0]:+7.2f},{s['x'][1]:+7.2f}] "
              f"Y[{s['y'][0]:+7.2f},{s['y'][1]:+7.2f}] "
              f"Z[{s['z'][0]:+7.2f},{s['z'][1]:+7.2f}]")
        print(f"  size {sx:6.2f} x {sy:6.2f} x {sz:6.2f}   vol={vol:.0f} mm^3")
        print(f"  centroid ({s['cx']:+7.2f},{s['cy']:+7.2f},{s['cz']:+7.2f})\n")

    print("=" * 70)
    print("DIAGNOSIS")
    print("=" * 70)
    if "new_gripper_fixture.STL" in info and "new_gripper_fixture1.STL" in info:
        a = info["new_gripper_fixture.STL"]
        b = info["new_gripper_fixture1.STL"]
        print(f"fixture  X span = {a['x'][1]-a['x'][0]:.2f} mm")
        print(f"fixture1 X span = {b['x'][1]-b['x'][0]:.2f} mm")
        print(f"fixture  Y span = {a['y'][1]-a['y'][0]:.2f} mm")
        print(f"fixture1 Y span = {b['y'][1]-b['y'][0]:.2f} mm")
        print(f"fixture  Z span = {a['z'][1]-a['z'][0]:.2f} mm")
        print(f"fixture1 Z span = {b['z'][1]-b['z'][0]:.2f} mm")
        print()
        print(f"X overlap: fixture[{a['x'][0]:.2f},{a['x'][1]:.2f}] vs "
              f"fixture1[{b['x'][0]:.2f},{b['x'][1]:.2f}]")
        x_overlap = (max(a['x'][0], b['x'][0]) < min(a['x'][1], b['x'][1]))
        y_overlap = (max(a['y'][0], b['y'][0]) < min(a['y'][1], b['y'][1]))
        z_overlap = (max(a['z'][0], b['z'][0]) < min(a['z'][1], b['z'][1]))
        print(f"  overlap X={x_overlap}  Y={y_overlap}  Z={z_overlap}")
        print()
        # The jaw (movable plate) is the thinner mesh
        if (a['x'][1]-a['x'][0]) > (b['x'][1]-b['x'][0]) * 2:
            print(" -> 'fixture' is the WIDE housing, 'fixture1' is the THIN jaw blade")
            print("    fixture1 sits at X={:.2f}..{:.2f} which is adjacent to "
                  "fixture X={:.2f}..{:.2f}".format(
                    b['x'][0], b['x'][1], a['x'][0], a['x'][1]))
            gap = b['x'][0] - a['x'][1] if b['x'][0] > a['x'][1] else a['x'][0] - b['x'][1]
            print(f"    gap between them = {gap:+.2f} mm  (negative = overlap)")
        print()
        # Check whether fixture1 covers the full Y range of fixture
        cov_y = (b['y'][0] >= a['y'][0] - 1) and (b['y'][1] <= a['y'][1] + 1)
        print(f"fixture1 Y span fits inside fixture Y span: {cov_y}")
        if not cov_y:
            print(f"  -> fixture1 only covers Y=[{b['y'][0]:.2f},{b['y'][1]:.2f}]; "
                  f"fixture is wider Y=[{a['y'][0]:.2f},{a['y'][1]:.2f}]")
            print(f"  -> the housing has an OPENING along the Y axis that the jaw "
                  f"only partially closes.")
