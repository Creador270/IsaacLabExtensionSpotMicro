# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different legged robots.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/demos/spot_quad.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher


# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates the SpotMicro.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""
import matplotlib.pyplot as plt
import numpy as np
import torch

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation

# from isaaclab.assets.articulation import ArticulationCfg

##
# Pre-defined configs

from robot_spot_ean import QUAD_EAN  # isort:skip
from isaaclab_assets.robots.unitree import UNITREE_A1_CFG  # isort:skip


def define_origins(num_origins: int, spacing: float) -> list[list[float]]:
    """Defines the origins of the the scene."""
    # create tensor based on number of environments
    env_origins = torch.zeros(num_origins, 3)
    # create a grid of origins
    num_cols = np.floor(np.sqrt(num_origins))
    num_rows = np.ceil(num_origins / num_cols)
    xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols), indexing="xy")
    env_origins[:, 0] = spacing * xx.flatten()[:num_origins] - spacing * (num_rows - 1) / 2
    env_origins[:, 1] = spacing * yy.flatten()[:num_origins] - spacing * (num_cols - 1) / 2
    env_origins[:, 2] = 0.0
    # return the origins
    return env_origins.tolist()


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2", "Origin3"
    # Each group will have a mount and a robot on top of it
    origins = define_origins(num_origins=2, spacing=1.25)

    # Origin EAN Quadruped
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    print("[INFO]: ************ROBOT DCMotorCfg Origin EAN ************")
    print("[INFO]: effort_limit: ", QUAD_EAN.actuators["base_legs"].effort_limit)
    print("[INFO]: saturation_effort: ", QUAD_EAN.actuators["base_legs"].saturation_effort)
    print("[INFO]: velocity_limit: ", QUAD_EAN.actuators["base_legs"].velocity_limit)
    print("[INFO]: stiffness: ", QUAD_EAN.actuators["base_legs"].stiffness)
    print("[INFO]: damping: ", QUAD_EAN.actuators["base_legs"].damping)
    print("[INFO]: friction: ", QUAD_EAN.actuators["base_legs"].friction)
    print("[INFO]: ****************************************")
    # -- Robot
    spotMicroAI = Articulation(QUAD_EAN.replace(prim_path="/World/Origin1/Robot1"))

    # Origin Untree Quadruped
    prim_utils.create_prim("/World/Origin2", "Xform", translation=origins[1])
    print("[INFO]: ************ROBOT DCMotorCfg Untree A1************")
    print("[INFO]: effort_limit: ", UNITREE_A1_CFG.actuators["base_legs"].effort_limit)
    print(
        "[INFO]: saturation_effort: ",
        UNITREE_A1_CFG.actuators["base_legs"].saturation_effort,
    )
    print("[INFO]: velocity_limit: ", UNITREE_A1_CFG.actuators["base_legs"].velocity_limit)
    print("[INFO]: stiffness: ", UNITREE_A1_CFG.actuators["base_legs"].stiffness)
    print("[INFO]: damping: ", UNITREE_A1_CFG.actuators["base_legs"].damping)
    print("[INFO]: friction: ", UNITREE_A1_CFG.actuators["base_legs"].friction)
    print("[INFO]: ****************************************")
    # -- Robot
    # untree_a1 = Articulation(UNITREE_A1_CFG.replace(prim_path="/World/Origin2/Robot2"))

    # return the scene information
    scene_entities = {
        "quadruped_ean": spotMicroAI,
        # "unitree_a1": untree_a1
    }
    return scene_entities, origins


def run_simulator(
    sim: sim_utils.SimulationContext,
    entities: dict[str, Articulation],
    origins: torch.Tensor,
):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    ep = 0
    fr_art = []  # Frecuency reguistry
    # Simulate physics
    while simulation_app.is_running():

        # reset
        if count % 1001 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            ep += 1
            # reset robots
            for index, robot in enumerate(entities.values()):
                # root state
                root_state = robot.data.default_root_state.clone()
                root_state[:, :3] += origins[index]
                robot.write_root_state_to_sim(root_state)
                #robot.write_root_velocity_to_sim(root_state[:, 7:])
                # joint state
                joint_pos, joint_vel = (
                    robot.data.default_joint_pos.clone(),
                    robot.data.default_joint_vel.clone(),
                )
                robot.write_joint_state_to_sim(joint_pos, joint_vel)
                # reset the internal state
                robot.reset()
            print(f"[INFO]: Resetting robots state...Ep{ep}")
        # apply default actions to the quadrupedal robot
        # torch.tensor([ shoulder,  shoulder,  shoulder,  shoulder,  leg,  leg,  leg,  leg, foot, foot, foot, foot], device=sim.device)
        for robot in entities.values():
            # if ep < 4:
            #     # print("[INFO]: Apend Robot joint position")
            #     fr_art.append(robot.data.joint_pos.cpu())
            # else:
            #     # print("[INFO]: Registry: ", len(fr_art))
            #     # Graficar los datos
            #     np_fr_art = np.array(fr_art)
            #     plt.figure()
            #     for i in range(12):
            #         if i == 8:
            #             plt.plot(np_fr_art[:, :, i], label=f"Joint{i}_Target", color="red")
            #         # else:
            #         # plt.plot(np_fr_art[:,i], label=f'Joint{i}')
            #     # plt.plot(np.full(len(fr_art), -0.09), label=f'Target: -0.09', color='red', linestyle='--')
            #     plt.xlabel("Steps")
            #     plt.ylabel("Joint Position (rad)")
            #     plt.title(f"Joint Positions Over Steps, Data size: {len(fr_art)}")
            #     plt.legend()
            #     plt.show()

            if count >= 300:
                # print("[INFO]: Step count: ", count)
                # generate random joint positions
                # joint_pos_target = robot.data.joint_pos.cpu()[0][5] + torch.tensor([0.1], device=sim.device)
                # joint_pos_target = robot.data.joint_pos.cpu()[0][4:8] + torch.randn_like(robot.data.default_joint_pos.cpu()[0][4:8]) * 0.1
                # joint_pos_target = joint_pos_target.to(sim.device)
                joint_pos_target = robot.data.joint_pos + 0.01
                #joint_pos_target = joint_pos_target.cpu()[0][8:]
                #joint_pos_target = joint_pos_target.to(sim.device)
                # joint_pos_target = torch.zeros(4, device=sim.device)
                # leg position
                # apply action to the robot
                robot.set_joint_position_target(joint_pos_target)#, joint_ids=[8, 9, 10, 11])
                # write data to sim
                robot.write_data_to_sim()
                # print("[INFO]: Step count: ", count)
                # a = torch.cat([joint_pos_target[:4], joint_pos_target[8:]])
                # print("robotTyp: ", robot.data.joint_pos.dtype, "robotTaryp", joint_pos_target.dtype)
                # print("robotNT: ", robot.data.joint_pos[8:])
                # Delay to analyze the simulation
                # print(robot.data.joint_pos[:])
                # time.sleep(0.5)
            else:

                # print("Quadruped EAN")
                joint_pos_target = robot.data.joint_pos  # + torch.randn_like(robot.data.joint_pos) * 0.01
                # apply action to the robot
                robot.set_joint_position_target(joint_pos_target)
                # write data to sim
                robot.write_data_to_sim()
                # print(robot.data.joint_pos)

        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1

        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)

        # Delay to analyze the simulation
        # time.sleep(0.01)


def main():
    """Main function."""

    # Initialize the simulation context
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01))
    # Set main camera
    sim.set_camera_view(eye=[0.7, 0.7, 0.7], target=[-0.7, 0.0, 0.3])
    # design scene
    # for r in range(2):
    #     if r == 1:
    #         scene_entities, scene_origins = design_scene(UNITREE_A1_CFG)

    #     else:
    #         scene_entities, scene_origins = design_scene(QUAD_EAN)
    #     scene_origins = torch.tensor(scene_origins, device=sim.device)
    #     # Play the simulator
    #     sim.reset()
    #     # Now we are ready!
    #     print("[INFO]: Setup complete...")
    #     # Run the simulator
    #     run_simulator(sim, scene_entities, scene_origins)

    scene_entities, scene_origins = design_scene()  # UNITREE_A1_CFG, QUAD_EAN
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
