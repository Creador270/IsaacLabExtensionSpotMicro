import os
import sys
from isaaclab.utils import configclass

from velocity_env_cfg import LocomotionVelocityRoughEnvCfg

##
# Pre-defined configs
##
from assets.robot_spot_ean import QUAD_EAN


@configclass
class SpotMicroRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        self.sim.disable_contact_processing = True
        super().__post_init__()
        # switch robot to SpotMicro
        self.scene.robot = QUAD_EAN.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # rewards
        #self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_foot"
        #self.rewards.feet_air_time.weight = 0.01
        #self.rewards.undesired_contacts = None
        #self.rewards.dof_torques_l2.weight = -0.0002
        #self.rewards.track_lin_vel_xy_exp.weight = 1.5
        #self.rewards.track_ang_vel_z_exp.weight = 0.75
        #self.rewards.dof_acc_l2.weight = -2.5e-7

@configclass
class SpotMicroRoughEnvCfg_PLAY(SpotMicroRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 1.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.events.base_external_force_torque = None
        self.events.push_robot = None
