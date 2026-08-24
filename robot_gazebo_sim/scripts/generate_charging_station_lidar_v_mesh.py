import os
import math
import numpy as np

mesh_dir = '/home/tetra/tetra_integration/ros/src/tetra_lab/robot_gazebo_sim/models/charging_station_lidar_v/meshes'
os.makedirs(mesh_dir, exist_ok=True)

mtl_content = """# Materials for LiDAR V-Shape Charging Station
newmtl StationBody
Kd 0.18 0.18 0.20
Ks 0.3 0.3 0.3
Ns 30.0

newmtl ChargingTerminal
Kd 0.85 0.65 0.15
Ks 0.8 0.8 0.8
Ns 80.0

newmtl VWingPlate
Kd 0.88 0.88 0.90
Ks 0.1 0.1 0.1
Ns 10.0

newmtl IndicatorLED
Kd 0.1 0.5 0.95
Ks 0.5 0.5 0.5
Ns 50.0
"""

with open(os.path.join(mesh_dir, 'charging_station_lidar_v.mtl'), 'w') as f:
    f.write(mtl_content)

v_list = []
vt_list = []
vn_list = []
f_groups = {}

def add_box(min_x, max_x, min_y, max_y, min_z, max_z, mat_name):
    start_v = len(v_list) + 1
    verts = [
        (min_x, min_y, min_z), # 0: ---
        (max_x, min_y, min_z), # 1: +--
        (max_x, max_y, min_z), # 2: ++-
        (min_x, max_y, min_z), # 3: -+-
        (min_x, min_y, max_z), # 4: --+
        (max_x, min_y, max_z), # 5: +-+
        (max_x, max_y, max_z), # 6: +++
        (min_x, max_y, max_z), # 7: -++
    ]
    for v in verts:
        v_list.append(v)
    
    faces = [
        ('front', [1, 2, 6, 5], (1, 0, 0)),
        ('back', [3, 0, 4, 7], (-1, 0, 0)),
        ('left', [2, 3, 7, 6], (0, 1, 0)),
        ('right', [0, 1, 5, 4], (0, -1, 0)),
        ('top', [4, 5, 6, 7], (0, 0, 1)),
        ('bottom', [0, 3, 2, 1], (0, 0, -1)),
    ]

    for face_name, idxs, normal in faces:
        vn_idx = len(vn_list) + 1
        vn_list.append(normal)
        
        vt_start = len(vt_list) + 1
        vt_list.append((0.0, 0.0))
        vt_list.append((1.0, 0.0))
        vt_list.append((1.0, 1.0))
        vt_list.append((0.0, 1.0))
        f1 = (f"{start_v + idxs[0]}/{vt_start}/{vn_idx}",
              f"{start_v + idxs[1]}/{vt_start + 1}/{vn_idx}",
              f"{start_v + idxs[2]}/{vt_start + 2}/{vn_idx}")
        f2 = (f"{start_v + idxs[0]}/{vt_start}/{vn_idx}",
              f"{start_v + idxs[2]}/{vt_start + 2}/{vn_idx}",
              f"{start_v + idxs[3]}/{vt_start + 3}/{vn_idx}")
        
        if mat_name not in f_groups:
            f_groups[mat_name] = []
        f_groups[mat_name].append(f1)
        f_groups[mat_name].append(f2)

def add_extruded_polygon(poly_2d, z_min, z_max, mat_name):
    """
    poly_2d: list of (x, y) coordinates forming a closed convex/simple polygon in CCW order.
    """
    start_v = len(v_list) + 1
    n = len(poly_2d)
    
    # Bottom vertices (0..n-1)
    for x, y in poly_2d:
        v_list.append((x, y, z_min))
    # Top vertices (n..2n-1)
    for x, y in poly_2d:
        v_list.append((x, y, z_max))
        
    # Side faces
    for i in range(n):
        next_i = (i + 1) % n
        # Vertices: bottom_i, bottom_next, top_next, top_i
        v0 = start_v + i
        v1 = start_v + next_i
        v2 = start_v + n + next_i
        v3 = start_v + n + i
        
        dx = poly_2d[next_i][0] - poly_2d[i][0]
        dy = poly_2d[next_i][1] - poly_2d[i][1]
        length = math.hypot(dx, dy)
        if length > 1e-6:
            normal = (dy / length, -dx / length, 0.0)
        else:
            normal = (0.0, 0.0, 1.0)
            
        vn_idx = len(vn_list) + 1
        vn_list.append(normal)
        
        vt_start = len(vt_list) + 1
        vt_list.append((0.0, 0.0))
        vt_list.append((1.0, 0.0))
        vt_list.append((1.0, 1.0))
        vt_list.append((0.0, 1.0))
        
        f1 = (f"{v0}/{vt_start}/{vn_idx}",
              f"{v1}/{vt_start + 1}/{vn_idx}",
              f"{v2}/{vt_start + 2}/{vn_idx}")
        f2 = (f"{v0}/{vt_start}/{vn_idx}",
              f"{v2}/{vt_start + 2}/{vn_idx}",
              f"{v3}/{vt_start + 3}/{vn_idx}")
              
        if mat_name not in f_groups:
            f_groups[mat_name] = []
        f_groups[mat_name].append(f1)
        f_groups[mat_name].append(f2)
        
    # Bottom face (normal (0, 0, -1))
    vn_idx_bot = len(vn_list) + 1
    vn_list.append((0.0, 0.0, -1.0))
    for i in range(1, n - 1):
        f = (f"{start_v}/{1}/{vn_idx_bot}",
             f"{start_v + i + 1}/{1}/{vn_idx_bot}",
             f"{start_v + i}/{1}/{vn_idx_bot}")
        f_groups[mat_name].append(f)
        
    # Top face (normal (0, 0, 1))
    vn_idx_top = len(vn_list) + 1
    vn_list.append((0.0, 0.0, 1.0))
    for i in range(1, n - 1):
        f = (f"{start_v + n}/{1}/{vn_idx_top}",
             f"{start_v + n + i}/{1}/{vn_idx_top}",
             f"{start_v + n + i + 1}/{1}/{vn_idx_top}")
        f_groups[mat_name].append(f)

# 1. Main Base Body (40cm Width Y: -0.20 to +0.20, 20cm Depth X: -0.10 to +0.10, 15cm Height Z: 0.0 to 0.15)
add_box(-0.10, 0.10, -0.20, 0.20, 0.0, 0.15, 'StationBody')

# 2. Charging Terminals (+X face, 2 electrodes)
add_box(0.10, 0.11, 0.05, 0.09, 0.04, 0.11, 'ChargingTerminal')
add_box(0.10, 0.11, -0.09, -0.05, 0.04, 0.11, 'ChargingTerminal')

# 3. Top Status LED
add_box(0.06, 0.08, -0.015, 0.015, 0.15, 0.155, 'IndicatorLED')

# 4. V-Shape 130° Reflector Object
# Angle: 130 degrees interior angle.
# Apex located at X = -0.06, Y = 0.0.
# Left wing: angle = +25 deg from +X axis.
# Right wing: angle = -25 deg from +X axis.
# Vector 1 dot Vector 2 = cos(50 deg) => angle between lines is 180 - 50 = 130.0 degrees.
# Wing thickness = 0.012m (1.2cm), Wing length = 0.16m (16cm).
# Height: Z = 0.15m to 0.38m (23cm height, covers robot LiDAR height Z≈0.27m perfectly).

apex_x = -0.06
apex_y = 0.0
half_angle_rad = math.radians(25.0) # 25 degrees from forward X
wing_len = 0.16
thickness = 0.012

# Left Wing Polygon:
# Inner line: from (apex_x, apex_y) to (apex_x + wing_len * cos(25°), apex_y + wing_len * sin(25°))
p0 = (apex_x, apex_y)
p1 = (apex_x + wing_len * math.cos(half_angle_rad), apex_y + wing_len * math.sin(half_angle_rad))
# Outer normal for left wing: (-sin(25°), cos(25°))
nx_left = -math.sin(half_angle_rad) * thickness
ny_left = math.cos(half_angle_rad) * thickness
p2 = (p1[0] + nx_left, p1[1] + ny_left)
p3 = (p0[0] + nx_left, p0[1] + ny_left)
add_extruded_polygon([p0, p1, p2, p3], 0.15, 0.38, 'VWingPlate')

# Right Wing Polygon:
# Inner line: from (apex_x, apex_y) to (apex_x + wing_len * cos(-25°), apex_y + wing_len * sin(-25°))
q0 = (apex_x, apex_y)
q1 = (apex_x + wing_len * math.cos(-half_angle_rad), apex_y + wing_len * math.sin(-half_angle_rad))
# Outer normal for right wing: (sin(-25°), -cos(-25°)) = (-sin(25°), -cos(25°))
nx_right = -math.sin(half_angle_rad) * thickness
ny_right = -math.cos(half_angle_rad) * thickness
q2 = (q1[0] + nx_right, q1[1] + ny_right)
q3 = (q0[0] + nx_right, q0[1] + ny_right)
add_extruded_polygon([q0, q3, q2, q1], 0.15, 0.38, 'VWingPlate')

# Support back-post connecting V-wings to station base
add_box(-0.10, -0.06, -0.04, 0.04, 0.15, 0.38, 'StationBody')

# Write OBJ file
with open(os.path.join(mesh_dir, 'charging_station_lidar_v.obj'), 'w') as f:
    f.write("# Charging Station with LiDAR 130-deg V-Shape Object\n")
    f.write("mtllib charging_station_lidar_v.mtl\n\n")
    for v in v_list:
        f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
    f.write("\n")
    for vt in vt_list:
        f.write(f"vt {vt[0]:.6f} {vt[1]:.6f}\n")
    f.write("\n")
    for vn in vn_list:
        f.write(f"vn {vn[0]:.6f} {vn[1]:.6f}\n")
    f.write("\n")
    
    for mat_name, faces in f_groups.items():
        f.write(f"usemtl {mat_name}\n")
        for face in faces:
            f.write(f"f {' '.join(face)}\n")
        f.write("\n")

print(f"Generated charging_station_lidar_v.obj with {len(v_list)} vertices.")

# Verify Angle
v1 = np.array([p1[0] - p0[0], p1[1] - p0[1]])
v2 = np.array([q1[0] - q0[0], q1[1] - q0[1]])
cos_acute = abs(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
acute_deg = math.degrees(math.acos(cos_acute))
v_angle_deg = 180.0 - acute_deg
print(f"Verified V-angle between inner planes: {v_angle_deg:.2f} degrees (Target: 130.00 deg)")
