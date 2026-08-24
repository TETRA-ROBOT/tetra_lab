import os

mesh_dir = '/home/tetra/tetra_integration/ros/src/tetra_lab/robot_gazebo_sim/models/charging_station/meshes'
os.makedirs(mesh_dir, exist_ok=True)

mtl_content = """# Materials for Charging Station
newmtl StationBody
Kd 0.18 0.18 0.20
Ks 0.3 0.3 0.3
Ns 30.0

newmtl ChargingTerminal
Kd 0.85 0.65 0.15
Ks 0.8 0.8 0.8
Ns 80.0

newmtl Backplate
Kd 0.25 0.25 0.27
Ks 0.2 0.2 0.2
Ns 20.0

newmtl TagBoard
Kd 1.0 1.0 1.0
Ks 0.0 0.0 0.0
Ns 10.0
map_Kd ../materials/textures/tag36h11_0.png

newmtl IndicatorLED
Kd 0.1 0.9 0.2
Ks 0.5 0.5 0.5
Ns 50.0
"""

with open(os.path.join(mesh_dir, 'charging_station.mtl'), 'w') as f:
    f.write(mtl_content)

# Generate OBJ with precise boxes and UV mapped tag face
v_list = []
vt_list = []
vn_list = []
f_groups = {}

def add_box(min_x, max_x, min_y, max_y, min_z, max_z, mat_name, tag_front=False):
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
        
        target_mat = mat_name
        if face_name == 'front' and tag_front:
            target_mat = 'TagBoard'
            vt_start = len(vt_list) + 1
            vt_list.append((1.0, 0.0))
            vt_list.append((0.0, 0.0))
            vt_list.append((0.0, 1.0))
            vt_list.append((1.0, 1.0))
            
            f1 = (f"{start_v + idxs[0]}/{vt_start}/{vn_idx}",
                  f"{start_v + idxs[1]}/{vt_start + 1}/{vn_idx}",
                  f"{start_v + idxs[2]}/{vt_start + 2}/{vn_idx}")
            f2 = (f"{start_v + idxs[0]}/{vt_start}/{vn_idx}",
                  f"{start_v + idxs[2]}/{vt_start + 2}/{vn_idx}",
                  f"{start_v + idxs[3]}/{vt_start + 3}/{vn_idx}")
        else:
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
        
        if target_mat not in f_groups:
            f_groups[target_mat] = []
        f_groups[target_mat].append(f1)
        f_groups[target_mat].append(f2)

# 1. Main Base Body: 40cm width (Y: -0.20 to +0.20), 20cm depth (X: -0.10 to +0.10), 15cm height (Z: 0 to 0.15)
add_box(-0.10, 0.10, -0.20, 0.20, 0.0, 0.15, 'StationBody')

# 2. Charging Terminals (+X face, 2 electrodes)
add_box(0.10, 0.11, 0.05, 0.09, 0.04, 0.11, 'ChargingTerminal')
add_box(0.10, 0.11, -0.09, -0.05, 0.04, 0.11, 'ChargingTerminal')

# 3. Rear Backplate (후면 판):
add_box(-0.10, -0.085, -0.12, 0.12, 0.15, 0.38, 'Backplate')

# 4. AprilTag Board on Backplate (+X face of backplate):
# Tag center Z = 0.30m (matching camera height), size 0.12m x 0.12m
add_box(-0.085, -0.084, -0.06, 0.06, 0.24, 0.36, 'Backplate', tag_front=True)

# 5. Top Indicator LED (Status LED on station top):
add_box(0.06, 0.08, -0.015, 0.015, 0.15, 0.155, 'IndicatorLED')

with open(os.path.join(mesh_dir, 'charging_station.obj'), 'w') as f:
    f.write("# Charging Station OBJ\n")
    f.write("mtllib charging_station.mtl\n\n")
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
        for f1, f2, f3 in faces:
            f.write(f"f {f1} {f2} {f3}\n")
        f.write("\n")

print(f"Generated charging_station.obj with {len(v_list)} vertices and {sum(len(v) for v in f_groups.values())} triangles.")
