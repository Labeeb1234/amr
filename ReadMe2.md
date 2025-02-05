# Creating a Path Planner using PPO alogrithm

### Simulator ----> IsaacLab (

 > **Note**:  LIDAR scans are hard to get since the RTX-lidar sensor has not been ported over to the current(as of 2024) isaaclab version(4.2.0).

## Custom Bot Configuration
- created similar to the standard template based bot config file for a custom bot of my own making (for now the bot is a 4WD with an imu, lidar sensor(no isaaclab implementation yet)). [source code](https://github.com/Labeeb1234/amr/blob/isaac_lab/differential_bot/diff_bot.py)
- The diff bot configuration has all the articulation, imu and other rigid and collision properties setup for further simulation
> **Note**: There aren't any specific tutorials for this in the [IsaacLab Doc](), the entire template can be learnt via my source code, the link to which is given above in this section or just go through the IsaacLab source files [example assets](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_assets/isaaclab_assets/robots/cartpole.py) for pointers 
## Creating a SceneCfg file
- A SceneCfg file was created based on the Interactive Scene module; the template tutorial to make this is already given in the official doc [here](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/02_scene/create_scene.html)
- The custom SceneCfg created for this project is given [here]()
