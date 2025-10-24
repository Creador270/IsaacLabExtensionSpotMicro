
import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="debugging script for SpotMicro spawning."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)

#conversion
import os
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import check_file_path
from isaaclab.utils.dict import print_dict
##
# Importing URDF robot
##

urdf_robot = "spotmicroaiean.urdf"
usd_name = "spotmicroaiean_inercia03.usd"

here = os.path.dirname(os.path.abspath(__file__))

# check valid file path
urdf_path = "{}/{}".format(f"{here}/URDF", urdf_robot)
if not os.path.isabs(urdf_path):
    urdf_path = os.path.abspath(urdf_path)
if not check_file_path(urdf_path):
    raise ValueError(f"Invalid file path: {urdf_path}")
# create destination path
usd_file = "{}/{}".format(f"{here}/SpotMicroUSD", usd_name)
if not os.path.isabs(usd_file):
    usd_file = os.path.abspath(usd_file)

if not check_file_path(usd_file):
    # Create config of Urdf converter 
    urdf_converter_cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=os.path.dirname(usd_file),
        usd_file_name=os.path.basename(usd_file),
        fix_base= False,
        merge_fixed_joints= False,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=1000.0,
            ),
            target_type="position",
        ),
    )
    
    # Print info
    print("-" * 80)
    print("-" * 80)
    print(f"Input URDF file: {urdf_path}")
    print("URDF importer config:")
    print_dict(urdf_converter_cfg.to_dict(), nesting=0)
    print("-" * 80)
    print("-" * 80)
    
    # Create Urdf converter and import the file
    urdf_converter = UrdfConverter(urdf_converter_cfg)
    # print output
    print("URDF importer output:")
    print(f"Generated USD file: {urdf_converter.usd_path}")
    print("-" * 80)
    print("-" * 80)

else:
    print("-" * 80)
    print(f"USD file already exists in: {usd_file}")
    print("-" * 80)
# launch omniverse app
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg

##
# Configurate SpotMicroAI
##

# Quadruped
QUAD_EAN = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/A1/a1.usd",
        usd_path=usd_file,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            #linear_damping=0.0,
            #angular_damping=0.0,
            #max_linear_velocity=1000.0,
            #max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        activate_contact_sensors=True,  # Add physics contact sensors
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # Agregar articulaciones de la unidad SpotMicroAI
        pos=(0.0, 0.0, 0.1),
        joint_pos={
            ".*_shoulder": 0.0,  # -0.548, 0.548, - up, + down
            # Articulations limits
            # - 'joint_front_left_shoulder': in [-0.548, 0.548]
            # - 'joint_front_right_shoulder': in [-0.548, 0.548]
            # - 'joint_rear_left_shoulder': in [-0.548, 0.548]
            # - 'joint_rear_right_shoulder': in [-0.548, 0.548]
            ".*_leg": -1.4922046,  #-1.559 - Ang_speed, + Ang_speed
            # Articulations limits
            # - 'joint_front_left_leg': in [-2.666, 1.548]
            # - 'joint_front_right_leg': in [-2.666, 1.548]
            # - 'joint_rear_left_leg': in [-2.666, 1.548]
            # - 'joint_rear_right_leg': in [-2.666, 1.548]
            ".*_foot": 2.580,  #2.589 - Ang_speed, + Ang_speed
            # Articulations limits
            # - 'joint_front_left_foot': in [-0.100, 2.590]
            # - 'joint_front_right_foot': in [-0.100, 2.590]
            # - 'joint_rear_left_foot': in [-0.100, 2.590]
            # - 'joint_rear_right_foot': in [-0.100, 2.590]
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "shoulders": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder"],
            velocity_limit=120.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_leg"],
            velocity_limit=120.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "foots": ImplicitActuatorCfg(
            joint_names_expr=[".*_foot"],
            velocity_limit=120.0,
            stiffness=10000.0,
            damping=100.0,
        )
    },
)

class SpawnSpotMicroAI(InteractiveSceneCfg):
    "sample scene"

    # simple ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # ligths
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0,
            color=(0.75, 0.75, 0.75),
        ),
    )

    # robot
    spotmicroaiean = QUAD_EAN.replace(prim_path="{ENV_REGEX_NS}/SpotMicroAI")

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveSceneCfg):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():
        # reset
        if count % 1000 == 0:
            # reset counters
            count = 0
            # reset the scene entities to the initial positions offset by the environment origins
            root_spotmicroaiean_state = scene["spotmicroaiean"].data.default_root_state.clone()
            root_spotmicroaiean_state[:, :3] += scene.env_origins
            
            # copy the default root state to the sim for the SpotMicroAI orientation and velocity
            scene["spotmicroaiean"].write_root_pose_to_sim(root_spotmicroaiean_state[:, :7])
            scene["spotmicroaiean"].write_root_velocity_to_sim(root_spotmicroaiean_state[:, 7:])
           
            # copy the default joint states to the sim
            joint_pos, joint_vel = (
                    scene["spotmicroaiean"].data.default_joint_pos.clone(),
                    scene["spotmicroaiean"].data.default_joint_vel.clone()
            )
            scene["spotmicroaiean"].write_joint_state_to_sim(joint_pos, joint_vel)

            # clear internal buffers
            scene.reset()
            print("[INFO]: Reseting SpotMicroAI state")

        # realize random joint targets after 300 steps
        if count >= 300:
            #print("SpotMicroAI joint positions:", scene["spotmicroaiean"].data.joint_pos)

            # apply random joint position targets
            joint_pos_target = scene["spotmicroaiean"].data.default_joint_pos + torch.randn_like(scene["spotmicroaiean"].data.default_joint_pos) * 0.01
        else:
            joint_pos_target = scene["spotmicroaiean"].data.joint_pos

        # set and write joint position targets
        scene["spotmicroaiean"].set_joint_position_target(joint_pos_target)
        scene.write_data_to_sim()

        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)

def main():
    """Main function to debug USD spawning"""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_cfg = SpawnSpotMicroAI(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    print("[INFO]: Setup complete")
    # Run the simulator
    run_simulator(sim, scene)

if __name__ == "__main__":
    main()
    simulation_app.close()
