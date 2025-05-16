from isaaclab.app import AppLauncher

app_launcher = AppLauncher()
simulation_app = app_launcher.app

import contextlib
import os

import carb
import isaacsim.core.utils.stage as stage_utils
import omni.kit.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import check_file_path
##
# Importing URDF robot
##


urdf_robot = "spotmicroaiean.urdf"
root_path = "exts/MicroSpot_implementation/assets/URDF"
usd_path = "exts/MicroSpot_implementation/assets/SpotMicroUSD"
usd_name = "spotmicroaiean_inercia03.usd"

def main():
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
    
    # Create Urdf converter and import the file
    urdf_converter = UrdfConverter(urdf_converter_cfg) 
if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
