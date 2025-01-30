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
            pos=(5.0, 0.0, 0.5), 
            rot=(1.0, 0.0, 0.0, 0.0) # in quaternion [w,x,y,z]
        )
    )

    # bot model
    robot: ArticulationCfg = DIFF_BOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")