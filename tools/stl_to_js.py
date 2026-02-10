"""
stl_to_js.py — Convert a binary STL file to compact JS vertex/face arrays
for use in the 3DScanner's browser-based orientation visualization.

Reads the GY-BNO085 breakout board STL model and outputs board.js containing:
  - BOARD_V: flat array of vertex coordinates [x0,y0,z0, x1,y1,z1, ...]
  - BOARD_F: flat array of face indices [a0,b0,c0, a1,b1,c1, ...]

The conversion pipeline:
  1. Parse binary STL (80-byte header + 50 bytes per triangle)
  2. Compute bounding box and center for normalization
  3. Scale so the longest planar dimension = TARGET_SIZE model units
  4. Remap axes from STL space to Y-up visualization frame:
       STL Y (long edge, 25.5mm)  → Model X (long edge)
       STL Z (thickness, 2.25mm)  → Model Y (up, IC side positive)
       STL X (short edge, 15.8mm) → Model -Z (short edge)
  5. Deduplicate vertices (rounded to 1 decimal) via hash map
  6. Remove degenerate and duplicate triangles
  7. Write compact JS with minimal whitespace

Usage: python tools/stl_to_js.py
"""

import struct
import os

# --- Configuration ---
STL_PATH = r"C:\projects\3dscanner\3dmodels\GY-BNO085 IMU.stl"
JS_PATH  = r"C:\projects\3dscanner\data\board.js"
TARGET_SIZE = 192.0  # longest planar dimension in model units (controls on-screen size)

def read_binary_stl(path):
    """Parse a binary STL file. Returns a list of triangles, each a list of 3 (x,y,z) tuples."""
    with open(path, "rb") as f:
        header = f.read(80)  # 80-byte header (ignored)
        num_triangles = struct.unpack("<I", f.read(4))[0]
        triangles = []
        for _ in range(num_triangles):
            # 12 floats: normal(3) + vertex1(3) + vertex2(3) + vertex3(3)
            data = struct.unpack("<12f", f.read(48))
            # Skip the normal (first 3 floats), grab vertices
            v1 = (data[3], data[4], data[5])
            v2 = (data[6], data[7], data[8])
            v3 = (data[9], data[10], data[11])
            triangles.append([v1, v2, v3])
            f.read(2)  # 2-byte attribute byte count (ignored)
    return triangles

def main():
    # 1. Parse the binary STL
    triangles = read_binary_stl(STL_PATH)
    original_count = len(triangles)
    print(f"Parsed {original_count} triangles from STL")

    # Collect all raw vertices (STL coords)
    all_verts = []
    for tri in triangles:
        for v in tri:
            all_verts.append(v)

    # 2. Compute bounding box in STL space to find center and scale
    xs = [v[0] for v in all_verts]
    ys = [v[1] for v in all_verts]
    zs = [v[2] for v in all_verts]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    center_z = (min_z + max_z) / 2.0

    span_x = max_x - min_x  # STL X
    span_y = max_y - min_y  # STL Y

    print(f"STL bounding box: X={span_x:.3f}  Y={span_y:.3f}  Z={max_z - min_z:.3f}")
    print(f"STL center: ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})")

    # 3. Scale so longest planar dimension (X or Y in STL) = TARGET_SIZE
    longest_planar = max(span_x, span_y)
    scale = TARGET_SIZE / longest_planar
    print(f"Longest planar dimension: {longest_planar:.3f} -> scale factor: {scale:.4f}")

    # 4. Remap axes to Y-up visualization frame and round
    #    viz_x =  (stl_y - center_y) * scale
    #    viz_y =  (stl_z - center_z) * scale
    #    viz_z = -(stl_x - center_x) * scale
    def transform(v):
        vx = round((v[1] - center_y) * scale, 1)
        vy = round((v[2] - center_z) * scale, 1)
        vz = round(-(v[0] - center_x) * scale, 1)
        return (vx, vy, vz)

    # 5-6. Build deduplicated vertex list
    vertex_map = {}  # (vx, vy, vz) -> index
    vertices = []    # list of unique vertex tuples

    def get_vertex_index(v):
        tv = transform(v)
        if tv not in vertex_map:
            vertex_map[tv] = len(vertices)
            vertices.append(tv)
        return vertex_map[tv]

    # Build face index list
    faces = []
    for tri in triangles:
        i0 = get_vertex_index(tri[0])
        i1 = get_vertex_index(tri[1])
        i2 = get_vertex_index(tri[2])

        # 7. Remove degenerate triangles
        if i0 == i1 or i1 == i2 or i0 == i2:
            continue

        faces.append((i0, i1, i2))

    # 8. Remove duplicate face entries
    unique_faces = []
    seen = set()
    for face in faces:
        key = tuple(sorted(face))
        if key not in seen:
            seen.add(key)
            unique_faces.append(face)

    faces = unique_faces

    # 9. Write output JS file
    # Build flat arrays
    vert_flat = []
    for v in vertices:
        vert_flat.extend(v)

    face_flat = []
    for f in faces:
        face_flat.extend(f)

    # Format numbers: strip trailing zeros for compactness
    def fmt(n):
        if isinstance(n, int):
            return str(n)
        s = f"{n:.1f}"
        # 1.0 -> 1, 2.5 -> 2.5
        if s.endswith(".0"):
            return s[:-2]
        return s

    vert_str = ",".join(fmt(v) for v in vert_flat)
    face_str = ",".join(str(i) for i in face_flat)

    js_content = f"var BOARD_V=[{vert_str}];\nvar BOARD_F=[{face_str}];\n"

    os.makedirs(os.path.dirname(JS_PATH), exist_ok=True)
    with open(JS_PATH, "w") as f:
        f.write(js_content)

    # 10. Print stats
    file_size = os.path.getsize(JS_PATH)
    print(f"")
    print(f"--- Stats ---")
    print(f"Original triangles : {original_count}")
    print(f"Final vertices     : {len(vertices)}")
    print(f"Final faces        : {len(faces)}")
    print(f"JS file size       : {file_size / 1024:.1f} KB")
    print(f"Output written to  : {JS_PATH}")

if __name__ == "__main__":
    main()
