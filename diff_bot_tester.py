import argparse
import os
import torch
import numpy as np
import logging

logging.basicConfig(
    level=logging.INFO, 
    format='[%(levelname)s]: %(message)s', 
)

from omni.isaac.lab.app import AppLauncher
# create argparser
parser = argparse.ArgumentParser(description="Example on creating an empty stage.")
parser.add_argument(
    "--num_envs", type=int, default=3, help="Number of environments to spawn for simulation"
)

# Appending AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.utils.math as math_utils
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.scene import InteractiveSceneCfg, InteractiveScene
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

from differential_bot import DIFF_BOT_CFG


@configclass
class DiffBotSceneCfg(InteractiveSceneCfg):
    # default ground plane
    ground_plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)) # size in square metres
    )

    lights = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # origins = [[0.25, 0.25, 0.0]]
    # for i, origin in enumerate(origins):
    #     prim_utils.create_prim(f"/World/Origin{i}", "Xform", translation=origin)

    # rigid cube prop model (for the environment)
    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="/World/Cube",
        spawn=sim_utils.CuboidCfg(
            mass_props=sim_utils.MassPropertiesCfg(
                mass=100.0
            ),
            size=[1.0, 1.0, 1.0],
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0))
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, -2.0, 1.0), 
            rot=(1.0, 0.0, 0.0, 0.0) # in quaternion [w,x,y,z]
        )
    )

    # bot model
    robot: ArticulationCfg = DIFF_BOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    


def run_simulator(sim: SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()
    
    count = 0
    root_state = robot.data.default_root_state.clone()
    root_state[:, :3] = scene.env_origins

    initial_states = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
    joint_pos, joint_vel = initial_states[0], initial_states[1]
    logging.info(f"{joint_pos}, {joint_vel}")
    
    while simulation_app.is_running():
        if count % 500 == 0:
            count = 0
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] = scene.env_origins
            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])

            initial_states = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            joint_pos, joint_vel = initial_states[0], initial_states[1]

            # logging.info(f"{joint_pos}, {joint_vel}")
            scene.reset()
            logging.info("Simulation Reset Completed... Physics Scene Initialized!")

        # logging.info(f"joint limits: {joint_limits}")
        # simple joint space control of bot (using velocity commands)
        joint_vel_cmd = torch.tensor([
            [0.0, 0.0, 0.0, 0.0],
            [5.0, 5.0, 5.0, 5.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        robot.set_joint_velocity_target(joint_vel_cmd)
        # joint_pos, joint_vel, joint_limits = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone(), robot.data.default_joint_limits.clone()
        joint_vel = robot.data.joint_vel
        joint_pos = robot.data.joint_pos
        robot_yaw = robot.data.heading_w # theta or phi value
        logging.info(f"joint velocities: {joint_vel[1]}")
        logging.info(f"yaw: {robot_yaw}")

        

        scene.write_data_to_sim()
        sim.step()
        count+=1
        scene.update(dt=sim_dt)
        

def main():
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    # Design scene
    scene_cfg = DiffBotSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    # Play the simulator
    # Now we are ready!
    logging.info("Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)

if __name__ == "__main__":
    main()