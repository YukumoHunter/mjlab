"""Unitree G1 constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.entity import EntityCfg
from mjlab.entity.entity import EntityArticulationInfoCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import ActuatorCfg, CollisionCfg

##
# MJCF and assets.
##

K1_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "booster_k1" / "xmls" / "k1.xml"
)
assert K1_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, K1_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(K1_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.513),
  joint_pos={
    "Left_Shoulder_Roll": -1.4,
    "Left_Elbow_Yaw": -0.4,
    "Right_Shoulder_Roll": 1.4,
    "Right_Elbow_Yaw": 0.4,
    "Left_Hip_Pitch": -0.2,
    "Left_Knee_Pitch": 0.4,
    "Left_Ankle_Pitch": -0.2,
    "Right_Hip_Pitch": -0.2,
    "Right_Knee_Pitch": 0.4,
    "Right_Ankle_Pitch": -0.2,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# This enables all collisions, including self collisions.
# Self-collisions are given condim=1 while foot collisions
# are given condim=3.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=[".*_collision"],
  condim={".*_collision": 3},
  priority={r"^(left|right)_foot_collision$": 1},
  friction={r"^(left|right)_foot_collision$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=[".*_collision"],
  contype=0,
  conaffinity=0,
  condim={".*_collision": 3},
  priority={r"^(left|right)_foot_collision$": 1},
  friction={r"^(left|right)_foot_collision$": (0.6,)},
)

# This disables all collisions except the feet.
# Feet get condim=3, all other geoms are disabled.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=[r"^(left|right)_foot_collision$"],
  contype=0,
  conaffinity=0,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Actuator config.
##

# Effort limits are retrieved from the k1_serial urdf. Stiffness and
# damping values are taken from [`booster_gym`](https://github.com/BoosterRobotics/booster_gym/),
# and based on the values for the Booster T1. Armature is set to a small
# constant value based on the calculated values from the Unitree G1.

DEFAULT_ARMATURE = 0.01

K1_ACTUATOR_1 = ActuatorCfg(
  joint_names_expr=["Head_.*"],
  effort_limit=7.0,
  armature=DEFAULT_ARMATURE,
  stiffness=20,  # [N*m/rad]
  damping=0.2,  # [N*m*s/rad]
)

K1_ACTUATOR_2 = ActuatorCfg(
  joint_names_expr=[
    ".*_Shoulder_.*",
    ".*_Elbow_.*",
  ],
  effort_limit=10.0,
  armature=DEFAULT_ARMATURE,
  stiffness=20,
  damping=0.5,
)

K1_ACTUATOR_3 = ActuatorCfg(
  joint_names_expr=[
    ".*_Hip_Pitch",
  ],
  effort_limit=30.0,
  armature=DEFAULT_ARMATURE,
  stiffness=200,
  damping=5,
)

K1_ACTUATOR_4 = ActuatorCfg(
  joint_names_expr=[
    ".*_Hip_Roll",
    ".*_Hip_Yaw",
  ],
  effort_limit=20.0,
  armature=DEFAULT_ARMATURE,
  stiffness=200,
  damping=5,
)

K1_ACTUATOR_5 = ActuatorCfg(
  joint_names_expr=[
    ".*_Knee_Pitch",
  ],
  effort_limit=40.0,
  armature=DEFAULT_ARMATURE,
  stiffness=200,
  damping=5,
)

K1_ACTUATOR_6 = ActuatorCfg(
  joint_names_expr=[
    ".*_Ankle_Pitch",
  ],
  effort_limit=20.0,
  armature=DEFAULT_ARMATURE,
  stiffness=50,
  damping=3,
)

K1_ACTUATOR_7 = ActuatorCfg(
  joint_names_expr=[
    ".*_Ankle_Roll",
  ],
  effort_limit=15.0,
  armature=DEFAULT_ARMATURE,
  stiffness=50,
  damping=3,
)


K1_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    K1_ACTUATOR_1,
    K1_ACTUATOR_2,
    K1_ACTUATOR_3,
    K1_ACTUATOR_4,
    K1_ACTUATOR_5,
    K1_ACTUATOR_6,
    K1_ACTUATOR_7,
  ),
  soft_joint_pos_limit_factor=0.9,
)

##
# Final config.
##

K1_ROBOT_CFG = EntityCfg(
  init_state=HOME_KEYFRAME,
  collisions=(FULL_COLLISION,),
  spec_fn=get_spec,
  articulation=K1_ARTICULATION,
)


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(K1_ROBOT_CFG)

  viewer.launch(robot.spec.compile())
