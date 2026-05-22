"""V23 fixture CAD analysis.

Parses the 5 binary STL files of the user's SolidWorks assembly and
prints bbox/centroid/triangle-count in the COMMON assembly frame.

If the STLs were exported with 'keep assembly origin' (typical when
exporting parts from an assembly one-at-a-time in SolidWorks), each
file's vertices are already expressed in the assembly origin's
coordinates and we can place every mesh into the URDF with xyz=0 rpy=0
(then translate the whole fixture group together).

That hypothesis is tested here by checking whether the centroids span a
range that makes sense for an assembled fixture (~tens of mm), instead
of all sitting at origin.
"""
import struct, sys, os, glob

CAD_DIR = r"C:\Users\santi\Desktop\Planta premier tech\planta Schneider\fixture para remache final"

def parse_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    header = data[:80]
    n = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + n * 50
    if expected_size != len(data):
        print(f"  !! file size mismatch (got {len(data)}, expected {expected_size}) -- probably ASCII STL")
        return None
    xs = []; ys = []; zs = []
    for i in range(n):
        off = 84 + i * 50
        # skip 12 bytes of normal
        for v in range(3):
            voff = off + 12 + v * 12
            x, y, z = struct.unpack("<fff", data[voff:voff+12])
            xs.append(x); ys.append(y); zs.append(z)
    return {
        "tris": n,
        "x": (min(xs), max(xs)),
        "y": (min(ys), max(ys)),
        "z": (min(zs), max(zs)),
        "cx": (min(xs)+max(xs))/2,
        "cy": (min(ys)+max(ys))/2,
        "cz": (min(zs)+max(zs))/2,
    }

if __name__ == "__main__":
    if not os.path.isdir(CAD_DIR):
        print(f"missing CAD dir: {CAD_DIR}"); sys.exit(1)
    files = sorted(glob.glob(os.path.join(CAD_DIR, "*.STL")))
    print(f"[V23 CAD] {len(files)} STLs in '{CAD_DIR}'\n")
    summaries = {}
    for p in files:
        name = os.path.basename(p)
        print(name)
        s = parse_stl(p)
        if s is None: continue
        summaries[name] = s
        print(f"  tris      {s['tris']}")
        print(f"  X (mm)    [{s['x'][0]:+9.3f}, {s['x'][1]:+9.3f}]   span {s['x'][1]-s['x'][0]:9.3f}")
        print(f"  Y (mm)    [{s['y'][0]:+9.3f}, {s['y'][1]:+9.3f}]   span {s['y'][1]-s['y'][0]:9.3f}")
        print(f"  Z (mm)    [{s['z'][0]:+9.3f}, {s['z'][1]:+9.3f}]   span {s['z'][1]-s['z'][0]:9.3f}")
        print(f"  centroid  ({s['cx']:+9.3f}, {s['cy']:+9.3f}, {s['cz']:+9.3f})")
        print()

    print("=" * 70)
    print("ASSEMBLY HYPOTHESIS CHECK")
    print("=" * 70)
    if not summaries:
        sys.exit(0)
    cxs = [s["cx"] for s in summaries.values()]
    cys = [s["cy"] for s in summaries.values()]
    czs = [s["cz"] for s in summaries.values()]
    print(f"centroid X range: [{min(cxs):+8.3f}, {max(cxs):+8.3f}]   span {max(cxs)-min(cxs):8.3f}")
    print(f"centroid Y range: [{min(cys):+8.3f}, {max(cys):+8.3f}]   span {max(cys)-min(cys):8.3f}")
    print(f"centroid Z range: [{min(czs):+8.3f}, {max(czs):+8.3f}]   span {max(czs)-min(czs):8.3f}")
    print()
    print("Interpretation:")
    if max(max(cxs)-min(cxs), max(cys)-min(cys), max(czs)-min(czs)) > 5:
        print("  centroids span > 5 units -> meshes are in ASSEMBLY frame (baked offsets).")
        print("  -> URDF: each mesh visual xyz=0 rpy=0, all under one fixture root.")
    else:
        print("  centroids all near origin -> meshes are in PART frame.")
        print("  -> URDF needs explicit xyz/rpy for each part (measure from CAD).")
