# Core simulation
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # Set to False to see the robot

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
import numpy as np
import csv
# Set world
world = World(stage_units_in_meters=1.0)

# Adding ground plane 
world.scene.add_default_ground_plane() 

# Load Robot
robot_path = r"C:\Users\matek_yulq090\Desktop\Coding\School\challengeblockb\isaacsim\g1_levi.usd"
prim_path = "/World/G1"

# Adding G1
add_reference_to_stage(usd_path=robot_path, prim_path=prim_path)

# FIXED: Use the correct nested path for the articulation root
articulation_path = "/World/G1/g1_29dof_with_hand"  

robot = Articulation(prim_paths_expr=articulation_path, name="G1")
world.scene.add(robot)
world.reset()

# Verify it works
print(f"Number of DOFs: {robot.num_dof}")
print(f"Joint names: {robot.dof_names}")

###########################################################
import omni.usd
from pxr import Usd, UsdPhysics

# Add this line before calling save_joints_to_csv()
stage = omni.usd.get_context().get_stage()

def save_joints_to_csv(stage: Usd.Stage, output_path: str) -> list[dict]:
    """
    Collect joint information from a USD stage and save to CSV.
    
    Args:
        stage: The USD stage to analyze
        output_path: Path to save the CSV file
        
    Returns:
        List of dictionaries containing joint properties
    """
    joints_data = []
    joint_count = 0

    for prim in stage.Traverse():
        prim_type = prim.GetTypeName()
        
        if "Joint" not in prim_type:
            continue
            
        joint_count += 1
        
        joint_info = {
            "index": joint_count,
            "prim_path": str(prim.GetPath()),
            "joint_name": prim.GetName(),
            "joint_type": prim_type,
            "axis": "N/A",
            "lower_limit_deg": "N/A",
            "upper_limit_deg": "N/A",
            "lower_limit_rad": "N/A",
            "upper_limit_rad": "N/A",
            "stiffness": "N/A",
            "damping": "N/A",
            "max_force": "N/A",
            "target_position": "N/A",
            "target_velocity": "N/A",
            "parent_link": "N/A",
            "child_link": "N/A",
        }
        
        # Revolute Joint limits
        if "Revolute" in prim_type:
            rev_joint = UsdPhysics.RevoluteJoint(prim)
            if rev_joint:
                lower = rev_joint.GetLowerLimitAttr().Get()
                upper = rev_joint.GetUpperLimitAttr().Get()
                axis = rev_joint.GetAxisAttr().Get()
                
                if lower is not None:
                    joint_info["lower_limit_deg"] = lower
                    joint_info["lower_limit_rad"] = round(lower * 3.14159265359 / 180, 4)
                if upper is not None:
                    joint_info["upper_limit_deg"] = upper
                    joint_info["upper_limit_rad"] = round(upper * 3.14159265359 / 180, 4)
                if axis is not None:
                    joint_info["axis"] = axis
        
        # Prismatic Joint limits
        if "Prismatic" in prim_type:
            pris_joint = UsdPhysics.PrismaticJoint(prim)
            if pris_joint:
                lower = pris_joint.GetLowerLimitAttr().Get()
                upper = pris_joint.GetUpperLimitAttr().Get()
                axis = pris_joint.GetAxisAttr().Get()
                
                if lower is not None:
                    joint_info["lower_limit_deg"] = lower
                    joint_info["lower_limit_rad"] = lower
                if upper is not None:
                    joint_info["upper_limit_deg"] = upper
                    joint_info["upper_limit_rad"] = upper
                if axis is not None:
                    joint_info["axis"] = axis
        
        # Body relationships
        joint_api = UsdPhysics.Joint(prim)
        if joint_api:
            body0 = joint_api.GetBody0Rel().GetTargets()
            body1 = joint_api.GetBody1Rel().GetTargets()
            joint_info["parent_link"] = str(body0[0]).split('/')[-1] if body0 else "N/A"
            joint_info["child_link"] = str(body1[0]).split('/')[-1] if body1 else "N/A"
        
        # Drive API (control parameters)
        for drive_type in ["angular", "linear"]:
            drive_api = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive_api:
                stiff = drive_api.GetStiffnessAttr().Get()
                damp = drive_api.GetDampingAttr().Get()
                max_f = drive_api.GetMaxForceAttr().Get()
                target_pos = drive_api.GetTargetPositionAttr().Get()
                target_vel = drive_api.GetTargetVelocityAttr().Get()
                
                if stiff is not None:
                    joint_info["stiffness"] = stiff
                if damp is not None:
                    joint_info["damping"] = damp
                if max_f is not None:
                    joint_info["max_force"] = max_f
                if target_pos is not None:
                    joint_info["target_position"] = target_pos
                if target_vel is not None:
                    joint_info["target_velocity"] = target_vel
        
        joints_data.append(joint_info)
    
    # Save to CSV
    if joints_data:
        fieldnames = [
            "index", "joint_name", "joint_type", "axis",
            "lower_limit_deg", "upper_limit_deg",
            "lower_limit_rad", "upper_limit_rad",
            "stiffness", "damping", "max_force",
            "target_position", "target_velocity",
            "parent_link", "child_link", "prim_path"
        ]
        
        with open(output_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(joints_data)
        
        print(f"✓ Saved {len(joints_data)} joints to: {output_path}")
    
    return joints_data

joints = save_joints_to_csv(stage, "C:/Users/matek_yulq090/Documents/g1_levente.csv")

########################################

# Run simulation
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()