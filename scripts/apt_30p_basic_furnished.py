import omni.usd
import omni.kit.commands
import numpy as np
import os
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Usd, UsdGeom, Gf, UsdPhysics

stage = omni.usd.get_context().get_stage()
group_path = "/World/Apt_30P_Basic_Furnished"

if stage.GetPrimAtPath(group_path).IsValid():
    omni.kit.commands.execute("DeletePrims", paths=[group_path])
omni.kit.commands.execute("CreatePrim", prim_type="Xform", prim_path=group_path)

# ================================================================
# Constants & Shift Variables (5-Fold Nav Protection)
# ================================================================
WALL_H = 2.4
WALL_T = 0.2

# Coordinates are absolute grid coordinates. 
# We visually shift the entire apartment so that robot spawn (0,0) is cleanly in the middle of the Living Room.
OFFSET_X = -1.95  # Living room center X 
OFFSET_Y = -6.60  # Living room center Y

# PBR Realistic Albedo Colors
COLOR_WALL      = np.array([0.70, 0.70, 0.70]) 
COLOR_FLOOR     = np.array([0.60, 0.45, 0.25]) 
COLOR_EXT_FLOOR = np.array([0.18, 0.20, 0.25]) 

# ================================================================
# Core Geometry Functions
# ================================================================
def spawn_slab(name, x_start, x_end, y_start, y_end, is_ext=False):
    color = COLOR_EXT_FLOOR if is_ext else COLOR_FLOOR
    pos = [(x_start+x_end)/2.0 + OFFSET_X, (y_start+y_end)/2.0 + OFFSET_Y, -0.05]
    scale = [x_end-x_start, y_end-y_start, 0.1]
    
    prim_path = f"{group_path}/Floor_{name}"
    FixedCuboid(prim_path=prim_path, name=f"Floor_{name}",
                position=np.array(pos, dtype=float),
                scale=np.array(scale, dtype=float),
                color=np.array(color, dtype=float),
                orientation=euler_angles_to_quat(np.array([0,0,0])))

def wall_seg(name, const_val, start, end, is_x_axis, j_start="CORNER", j_end="CORNER"):
    s, e = start, end
    if is_x_axis:
        thickness = WALL_T
        if j_start == "CORNER": s -= (WALL_T / 2.0)
        if j_end == "CORNER":   e += (WALL_T / 2.0)
    else:
        thickness = WALL_T - 0.002
        if j_start == "CORNER": s += (WALL_T / 2.0) - 0.05
        if j_end == "CORNER":   e -= (WALL_T / 2.0) - 0.05

    if e <= s: return
    mid, length = (s+e)/2.0, e-s
    
    pos = [mid + OFFSET_X, const_val + OFFSET_Y, WALL_H/2] if is_x_axis else [const_val + OFFSET_X, mid + OFFSET_Y, WALL_H/2]
    scale = [length, thickness, WALL_H] if is_x_axis else [thickness, length, WALL_H]
    
    prim_path = f"{group_path}/Wall_{name}"
    FixedCuboid(prim_path=prim_path, name=f"Wall_{name}",
                position=np.array(pos, dtype=float),
                scale=np.array(scale, dtype=float),
                color=np.array(COLOR_WALL, dtype=float),
                orientation=euler_angles_to_quat(np.array([0,0,0])))

# ================================================================
# Structural Topology: 30-Pyeong Basic Type
# ================================================================
print("--- Generating 30P Basic Footprint & Walls ---")

# Floor Slabs
spawn_slab("Apt_LBed",    0.0,  3.90, 0.0, 4.50)
spawn_slab("Apt_Living",  0.0,  3.90, 4.50, 8.70)
spawn_slab("Apt_Baths",   3.90, 6.20, 0.0, 3.30)
spawn_slab("Apt_MRBed",   6.20, 9.50, 0.0, 3.30)
spawn_slab("Apt_BRBalc",  9.50, 10.60,0.0, 3.30, is_ext=True)
spawn_slab("Apt_KitHall", 3.90, 10.60,3.30, 6.00)
spawn_slab("Apt_Ent",     3.90, 5.40, 6.00, 8.70, is_ext=True) # entrance shoe grid
spawn_slab("Apt_TRBed",   5.40, 8.70, 6.00, 8.70)
spawn_slab("Apt_TRBalc",  8.70, 10.60,6.00, 8.70, is_ext=True)
spawn_slab("Balc_Bot_L", -1.30, 0.0,  0.0, 4.50, is_ext=True)
spawn_slab("Balc_Top_L", -1.70, 0.0,  4.50, 8.70, is_ext=True)
spawn_slab("Pub_Core",   -1.70, 10.60,8.70, 11.50,is_ext=True)

# Outer Perimeter (Ext)
wall_seg("Ext_B_LBalc", 0.0,  -1.30, 0.0, True, j_end="DOOR") # Joint meets inner balc 
wall_seg("Ext_B_Main",  0.0,  0.0, 10.60, True)
wall_seg("Ext_R_Main", 10.60, 0.0, 8.70,  False)
wall_seg("Ext_R_Pub",  10.60, 8.70, 11.50,False)
wall_seg("Ext_T_Pub",  11.50,-1.70, 10.60,True)
wall_seg("Ext_L_Pub",  -1.70, 8.70, 11.50,False)
wall_seg("Ext_L_LBal_T",-1.70,4.50, 8.70, False)
wall_seg("Ext_L_Jog",   4.50, -1.70, -1.30,True)
wall_seg("Ext_L_LBal_B",-1.30,0.0,  4.50, False)

# Inner Balcony Wall (X=0.0) -> Doors
wall_seg("BalcIn_B1", 0.0, 0.0, 0.5, False, j_end="DOOR")
wall_seg("BalcIn_B2", 0.0, 2.5, 4.5, False, j_start="DOOR") # 2m LBed door
wall_seg("BalcIn_T1", 0.0, 4.5, 5.0, False, j_end="DOOR")
wall_seg("BalcIn_T2", 0.0, 7.0, 8.7, False, j_start="DOOR") # 2m Living door

# L.Bed/Living Divider (Y=4.50)
wall_seg("LBed_Liv", 4.50, 0.0, 3.90, True)

# Apt/Public Divider (Y=8.70)
wall_seg("AptPub_L", 8.70, -1.70, 4.0, True, j_end="DOOR")
wall_seg("AptPub_R", 8.70, 5.2, 10.60, True, j_start="DOOR") # Main Ap Entrance

# L.Bed Right Wall / Entrance Left Wall (X=3.90)
wall_seg("Vert_39_B", 3.90, 0.0, 3.30, False, j_end="DOOR")
wall_seg("Vert_39_T", 3.90, 6.00, 8.70, False, j_start="DOOR") # Notice: 3.3 to 6.0 is perfectly open Hall

# Entrance Right Wall (X=5.40)
wall_seg("Vert_54_T", 5.40, 6.00, 8.70, False)

# Bath Right Wall (X=6.20)
wall_seg("Vert_62_B", 6.20, 0.0, 3.30, False)

# Bath Divider (X=5.05)
wall_seg("Vert_50_B", 5.05, 0.0, 3.30, False)

# T.R.Bed Right Wall (X=8.70)
wall_seg("Vert_87_T1", 8.70, 6.00, 6.50, False, j_end="DOOR")
wall_seg("Vert_87_T2", 8.70, 8.50, 8.70, False, j_start="DOOR") # 2m TRBalc Door

# M.R.Bed Right Wall (X=9.50)
wall_seg("Vert_95_B1", 9.50, 0.0, 0.50, False, j_end="DOOR")
wall_seg("Vert_95_B2", 9.50, 2.50, 3.30, False, j_start="DOOR") # 2m BRBalc Door

# Baths & M.R.Bed Top (Y=3.30)
wall_seg("Horiz_33_B1", 3.30, 3.90, 4.00, True, j_end="DOOR")
wall_seg("Horiz_33_B2", 3.30, 4.90, 5.20, True, j_start="DOOR", j_end="DOOR") # doors
wall_seg("Horiz_33_B3", 3.30, 6.00, 6.20, True, j_start="DOOR")
wall_seg("Horiz_33_M1", 3.30, 6.20, 6.40, True, j_end="DOOR")
wall_seg("Horiz_33_M2", 3.30, 7.90, 9.50, True, j_start="DOOR") # 1.5m door
wall_seg("Horiz_33_Bal",3.30, 9.50, 10.60,True) # Solid kitchen separator

# Kitchen & T.R.Bed Top (Y=6.00)
# Entrance opening Y=6.0 is mostly gap (X=3.9 to 5.4). No wall there.
wall_seg("Horiz_60_T1", 6.00, 5.40, 5.60, True, j_end="DOOR")
wall_seg("Horiz_60_T2", 6.00, 7.60, 8.70, True, j_start="DOOR") # 2m door
wall_seg("Horiz_60_Bal",6.00, 8.70, 10.60,True) # Solid kitchen separator


# ================================================================
# Phase 3: Semantic Asset Allocation & Solidification
# ================================================================
print("--- Injecting Furniture & Resolving Physics ---")

ASSETS_DIR = "/home/jnu/go2_sim/assets/objects"
furniture_dict = {
    "Sofa": "sofa_02_4k.usdc",
    "Television": "Television_01_4k.usdc",
    "Dining_Table": "wooden_table_02_4k.usdc",
    "Chair_L": "SchoolChair_01_4k.usdc",
    "Chair_R": "SchoolChair_01_4k.usdc",
    "Bed_Drawer": "vintage_wooden_drawer_01_4k.usdc",
    "Steel_Shelf": "steel_frame_shelves_01_4k.usdc",
    "Commode": "chinese_commode_4k.usdc",
    "Ottoman": "Ottoman_01_4k.usdc"
}

placements = {
    # Name: ([Grid_X, Grid_Y, Z], Rot_Z)
    "Sofa": ([2.0, 5.0, 0.0], 180),           # Facing -Y in Living Room
    "Television": ([2.5, 8.4, 0.0], 0),       # Against Top wall of Living Room
    "Ottoman": ([2.0, 6.0, 0.0], 0),          # Center of Living Room space
    "Dining_Table": ([8.0, 4.5, 0.0], 0),     # Middle of Kitchen area
    "Chair_L": ([7.2, 4.5, 0.0], 90),         # Left of table
    "Chair_R": ([8.8, 4.5, 0.0], -90),        # Right of table
    "Bed_Drawer": ([1.0, 4.0, 0.0], -90),     # Inside L.Bed facing right
    "Steel_Shelf": ([8.0, 8.2, 0.0], 180),    # Top wall of T.R.Bed
    "Commode": ([8.0, 0.5, 0.0], 0)           # Bottom wall of M.R.Bed
}

for name, folder_name in furniture_dict.items():
    file_path = os.path.join(ASSETS_DIR, folder_name, folder_name)
    prim_path = f"{group_path}/{name}"
    
    if os.path.exists(file_path):
        if name in placements:
            grid_pos, rot_z = placements[name]
            world_pos = [grid_pos[0] + OFFSET_X, grid_pos[1] + OFFSET_Y, grid_pos[2]]
            
            # Memory Cleanup
            if stage.GetPrimAtPath(prim_path).IsValid():
                omni.kit.commands.execute("DeletePrims", paths=[prim_path])
                
            # Bind Bundle
            prim = stage.DefinePrim(prim_path, "Xform")
            prim.GetReferences().AddReference(file_path)
            
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            
            xform.AddTranslateOp().Set(Gf.Vec3d(world_pos[0], world_pos[1], world_pos[2]))
            xform.AddRotateZOp().Set(float(rot_z))
            xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
            
            # Explicit Solid Colliders (for LiDAR scans)
            for child in Usd.PrimRange(prim):
                if child.IsA(UsdGeom.Mesh):
                    if not child.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(child)
                    if not child.HasAPI(UsdPhysics.MeshCollisionAPI):
                        mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(child)
                        mesh_coll.CreateApproximationAttr().Set("boundingCube")
            
            print(f"[OK] Instantiated {name} at World({world_pos[0]:.2f}, {world_pos[1]:.2f})")
    else:
        print(f"[WARNING] Local asset missing: {file_path}")

print("--- Scene Construction Fully Completed ---")
