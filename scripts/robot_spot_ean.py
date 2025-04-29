
import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuracion SpotMicroAI
##

# QuadrupedEAN
QUAD_EAN = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/A1/a1.usd",
        usd_path="/home/user1/Omniverse/IsaacLabExtensionSpotMicro/exts/MicroSpot_implementation/assets/spotmicroaiean_inercia02.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
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
        pos=(0.0, 0.0, 0.26),
        joint_pos={
            ".*_shoulder": 0.0,  # -0.548, 0.548, - up, + down
            # Articulations limits
            # - 'joint_front_left_shoulder': in [-0.548, 0.548]
            # - 'joint_front_right_shoulder': in [-0.548, 0.548]
            # - 'joint_rear_left_shoulder': in [-0.548, 0.548]
            # - 'joint_rear_right_shoulder': in [-0.548, 0.548]
            ".*_leg": 0.0,  #-1.559 - Ang_speed, + Ang_speed
            # Articulations limits
            # - 'joint_front_left_leg': in [-2.666, 1.548]
            # - 'joint_front_right_leg': in [-2.666, 1.548]
            # - 'joint_rear_left_leg': in [-2.666, 1.548]
            # - 'joint_rear_right_leg': in [-2.666, 1.548]
            ".*_foot": 0.0,  #2.589 - Ang_speed, + Ang_speed
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
            effort_limit=33.5,
            saturation_effort=33.5,
            velocity_limit=21.0,
            stiffness=25.0,
            damping=0.5,
            friction=0.0,
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
