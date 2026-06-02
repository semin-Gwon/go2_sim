import omni.usd
import omni.kit.commands
import numpy as np
import os
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Usd, UsdGeom, Gf, UsdPhysics

stage = omni.usd.get_context().get_stage()
group_path = "/World/Apt_4Bay_Furnished"

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
# Living Room center is around X=5.85, Y=2.25
OFFSET_X = -5.85
OFFSET_Y = -2.25

# PBR Realistic Albedo Colors (From Global Workflow)
COLOR_WALL      = np.array([0.70, 0.70, 0.70]) 
COLOR_FLOOR     = np.array([0.60, 0.45, 0.25]) 
COLOR_EXT_FLOOR = np.array([0.18, 0.20, 0.25]) 

# ================================================================
# Core Geometry Functions (Universal Joint & Crack Prevention)
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
# Phase 2: Structural Topology - 4-Bay Apartment
# ================================================================
print("--- Generating 4-Bay Footprint & Walls ---")

# Floor Slabs
spawn_slab("Living", 3.6, 8.1, 0.0, 4.5)
spawn_slab("Kitchen", 3.6, 8.1, 4.5, 8.5)
spawn_slab("MBed", 0.0, 3.6, 0.0, 4.5)
spawn_slab("Bed1", 8.1, 10.8, 0.0, 3.5)
spawn_slab("Bed2", 10.8, 13.8, 0.0, 3.5)
spawn_slab("Hall", 8.1, 12.0, 3.5, 4.5)
spawn_slab("Bed3", 8.1, 10.8, 4.5, 8.5)
spawn_slab("Entrance", 10.8, 12.0, 4.5, 6.5, is_ext=True)
spawn_slab("Bath1", 12.0, 13.8, 3.5, 6.0)
spawn_slab("Bath2", 2.0, 3.6, 4.5, 6.5)
spawn_slab("DressRoom", 0.0, 2.0, 4.5, 8.5)

# Outer Balconies
spawn_slab("Balc_MBed", 0.0, 3.6, -1.5, 0.0, is_ext=True)
spawn_slab("Balc_Liv", 3.6, 8.1, -1.5, 0.0, is_ext=True)
spawn_slab("Balc_Beds", 8.1, 13.8, -1.5, 0.0, is_ext=True)

# Exterior Walls
wall_seg("Ext_Bot", -1.5, 0.0, 13.8, True)
wall_seg("Ext_Top_M", 8.5, 0.0, 10.8, True)
wall_seg("Ext_Top_R", 6.5, 10.8, 12.0, True) # top of entrance
wall_seg("Ext_Top_Bath", 6.0, 12.0, 13.8, True) # top of bath1
wall_seg("Ext_L", 0.0, -1.5, 8.5, False)
wall_seg("Ext_R", 13.8, -1.5, 6.0, False)

# Main bottom wall separating balconies (with 2m sliding doors)
wall_seg("Horiz_Y0_MBed1", 0.0, 0.0, 1.0, True, j_end="DOOR")
wall_seg("Horiz_Y0_MBed2", 0.0, 3.0, 3.6, True, j_start="DOOR")
wall_seg("Horiz_Y0_Liv1",  0.0, 3.6, 5.0, True, j_end="DOOR")
wall_seg("Horiz_Y0_Liv2",  0.0, 7.0, 8.1, True, j_start="DOOR")
wall_seg("Horiz_Y0_B1_1",  0.0, 8.1, 8.5, True, j_end="DOOR")
wall_seg("Horiz_Y0_B1_2",  0.0, 10.5, 10.8, True, j_start="DOOR")
wall_seg("Horiz_Y0_B2_1",  0.0, 10.8, 11.5, True, j_end="DOOR")
wall_seg("Horiz_Y0_B2_2",  0.0, 13.5, 13.8, True, j_start="DOOR")

# Horizontal Dividers
# Top of Bed1/Bed2 (Y=3.5)
wall_seg("Horiz_Y35_B1L", 3.5, 8.1, 8.5, True, j_end="DOOR")
wall_seg("Horiz_Y35_B1R", 3.5, 10.5, 10.8, True, j_start="DOOR")
wall_seg("Horiz_Y35_B2L", 3.5, 10.8, 11.0, True, j_end="DOOR")
wall_seg("Horiz_Y35_B2R", 3.5, 13.0, 13.8, True, j_start="DOOR")

# Top of MBed (Y=4.5)
wall_seg("Horiz_Y45_MBed", 4.5, 0.0, 2.0, True, j_end="DOOR") # Bath2 door is 2.0~3.6

# Top of Hallway (Y=4.5)
wall_seg("Horiz_Y45_B3L", 4.5, 8.1, 8.5, True, j_end="DOOR")
wall_seg("Horiz_Y45_B3R", 4.5, 10.5, 12.0, True, j_start="DOOR")

# Horizontal above Bath2
wall_seg("Horiz_Y65_Bath2", 6.5, 2.0, 3.6, True)

# Vertical Dividers
wall_seg("Vert_X36_B", 3.6, 0.0, 4.5, False) # MBed/Living
wall_seg("Vert_X36_T", 3.6, 4.5, 6.5, False) # Bath2/Kitchen

wall_seg("Vert_X20", 2.0, 4.5, 8.5, False) # DressRoom/Bath2

wall_seg("Vert_X81_B", 8.1, 0.0, 3.5, False) # Living/Bed1
wall_seg("Vert_X81_T", 8.1, 4.5, 8.5, False) # Kitchen/Bed3

wall_seg("Vert_X108_B", 10.8, 0.0, 3.5, False) # Bed1/Bed2
wall_seg("Vert_X108_T", 10.8, 4.5, 6.5, False) # Bed3/Entrance

wall_seg("Vert_X120", 12.0, 5.5, 6.0, False, j_start="DOOR") # Entrance/Bath1

# ================================================================
# Phase 3: Semantic Asset Allocation & Solidification
# ================================================================
print("--- Injecting Furniture & Resolving Physics ---")

ASSETS_DIR = "/home/jnu/go2_sim/assets/objects"
furniture_dict = {
    "Sofa": "sofa_02_4k.usdc",
    "Television": "Television_01_4k.usdc",
    "Dining_Table": "wooden_table_02_4k.usdc",
    "Bed_MBed": "vintage_wooden_drawer_01_4k.usdc", # Placeholder for bed
    "Bed_B1": "vintage_wooden_drawer_01_4k.usdc",
    "Bed_B2": "vintage_wooden_drawer_01_4k.usdc",
    "Commode": "chinese_commode_4k.usdc",
    "Ottoman": "Ottoman_01_4k.usdc"
}

placements = {
    # Name: ([Grid_X, Grid_Y, Z], Rot_Z)
    "Sofa": ([5.8, 1.5, 0.0], 180),           # Living Room
    "Television": ([5.8, 4.0, 0.0], 0),       # Living Room Wall
    "Ottoman": ([5.0, 2.5, 0.0], 0),          # Center of Living Room
    "Dining_Table": ([5.8, 6.5, 0.0], 90),    # Kitchen
    "Bed_MBed": ([1.5, 1.0, 0.0], -90),       # Master Bed
    "Bed_B1": ([9.0, 1.0, 0.0], -90),         # Bed 1
    "Bed_B2": ([12.0, 1.0, 0.0], -90),        # Bed 2
    "Commode": ([11.4, 5.5, 0.0], 180)        # Entrance area
}

for name, folder_name in furniture_dict.items():
    file_path = os.path.join(ASSETS_DIR, folder_name, folder_name)
    prim_path = f"{group_path}/{name}"
    
    if os.path.exists(file_path):
        if name in placements:
            grid_pos, rot_z = placements[name]
            world_pos = [grid_pos[0] + OFFSET_X, grid_pos[1] + OFFSET_Y, grid_pos[2]]
            
            if stage.GetPrimAtPath(prim_path).IsValid():
                omni.kit.commands.execute("DeletePrims", paths=[prim_path])
                
            prim = stage.DefinePrim(prim_path, "Xform")
            prim.GetReferences().AddReference(file_path)
            
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            
            xform.AddTranslateOp().Set(Gf.Vec3d(world_pos[0], world_pos[1], world_pos[2]))
            xform.AddRotateZOp().Set(float(rot_z))
            xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
            
            for child in Usd.PrimRange(prim):
                if child.IsA(UsdGeom.Mesh):
                    if not child.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(child)
                    if not child.HasAPI(UsdPhysics.MeshCollisionAPI):
                        mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(child)
                        mesh_coll.CreateApproximationAttr().Set("boundingCube")
            
            print(f"[OK] Instantiated {name}")
    else:
        print(f"[WARNING] Local asset missing: {file_path}")

print("--- Scene Construction Fully Completed ---")
