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

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configurate SpotMicroAI
##

# Quadruped
QUAD_EAN = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
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
