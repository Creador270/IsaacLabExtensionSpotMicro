
import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

import contextlib
import os

import carb
import isaacsim.core.utils.stage as stage_utils
import omni.kit.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import check_file_path
from isaaclab.utils.dict import print_dict
##
# Importing URDF robot
##

urdf_robot = "spotmicroaiean.urdf"
root_path = "exts/MicroSpot_implementation/assets/URDF"
usd_path = "exts/MicroSpot_implementation/assets/SpotMicroUSD"
usd_name = "spotmicroaiean_inercia03.usd"

# check valid file path
urdf_path = "{}/{}".format(root_path, urdf_robot)
if not os.path.isabs(urdf_path):
    urdf_path = os.path.abspath(urdf_path)
if not check_file_path(urdf_path):
    raise ValueError(f"Invalid file path: {urdf_path}")
# create destination path
usd_file = "{}/{}".format(usd_path, usd_name)
if not os.path.isabs(usd_file):
    usd_file = os.path.abspath(usd_file)

# config of Urdf converter 
urdf_converter_cfg = UrdfConverterCfg(
    asset_path=urdf_path,
    usd_dir=os.path.dirname(usd_file),
    usd_file_name=os.path.basename(usd_file),
    fix_base= False,
    merge_fixed_joints= False,
    force_usd_conversion=True,
    joint_drive=UrdfConverterCfg.JointDriveCfg(
        gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
            stiffness=625.0,
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

##
# Configuracion SpotMicroAI
##

# QuadrupedEAN
QUAD_EAN = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/A1/a1.usd",
        usd_path=usd_file,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # Agregar articulaciones de la unidad SpotMicroAI
        pos=(0.0, 0.0, 0.3),
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
        "base_legs": DCMotorCfg(
            joint_names_expr=[".*_leg", ".*_foot", ".*_shoulder"],
            #effort_limit=33.5,
            saturation_effort=625.0 / 100.0,
            velocity_limit=1.0,
            stiffness=625.0 / 10.0,
            damping=0.0,
            #friction=0.0,
            # effort_limit=33.0,
            # saturation_effort=33.5,
            # velocity_limit=21.0,
            # stiffness=25.0,
            # damping=0.5,
            # friction=0.0,
            # effort_limit=21.0,
            # saturation_effort=20.0,
            # velocity_limit=6.98,
            # stiffness=25.0,
            # damping=0.5,
            # friction=0.0,
        ),
    },
)
