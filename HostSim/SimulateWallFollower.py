# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import argparse
import sys

import carb
import numpy as np
import math
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.physx import RotatingLidarPhysX
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import WallFollower as wf
#import UmdWallFollower
#import RafaelWallFollower
from Fence import Fence


#from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from pxr import UsdLux, UsdGeom, Sdf, Gf


from omni.isaac.core.utils.prims import create_prim
from pxr import Gf, Sdf, UsdGeom

from omni.isaac.core.utils.prims import create_prim
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema

def create_rect_room(stage,
                     center=(0.0, 0.0, 0.0),
                     size_x=8.0,
                     size_y=6.0,
                     wall_height=2.5,
                     wall_thickness=0.1,
                     base_z=0.0):
    """
    Erzeugt 4 physikalisch feste Wände (statisch, kollidierbar).
    """

    cx, cy, cz = center
    z_mid = base_z + wall_height * 0.5

    room_root = "/World/Room"
    if not stage.GetPrimAtPath(room_root).IsValid():
        UsdGeom.Xform.Define(stage, Sdf.Path(room_root))

    def make_physics_wall(name, pos, scale):
        prim_path = f"{room_root}/{name}"

        # Wand als geometrisches Cube-Prim
        create_prim(
            prim_path=prim_path,
            prim_type="Cube",
            translation=Gf.Vec3d(*pos),
            scale=Gf.Vec3f(*scale),
            attributes={"size": 2.0},
        )

        prim = stage.GetPrimAtPath(prim_path)

        # Kollisions-API anwenden
        UsdPhysics.CollisionAPI.Apply(prim)

        # Physikalischer Body (statisch)
        body = UsdPhysics.RigidBodyAPI.Apply(prim)
        body.CreateRigidBodyEnabledAttr(False)  # static geometry

        # PhysX-Mesh / Collider
        collider = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        collider.CreateDisableGravityAttr(True)

    # Nord
    make_physics_wall(
        "Wall_N",
        pos=(cx, cy + size_y * 0.5, z_mid),
        scale=(size_x * 0.5, wall_thickness * 0.5, wall_height * 0.5),
    )

    # Süd
    make_physics_wall(
        "Wall_S",
        pos=(cx, cy - size_y * 0.5, z_mid),
        scale=(size_x * 0.5, wall_thickness * 0.5, wall_height * 0.5),
    )

    # Ost
    make_physics_wall(
        "Wall_E",
        pos=(cx + size_x * 0.5, cy, z_mid),
        scale=(wall_thickness * 0.5, size_y * 0.5, wall_height * 0.5),
    )

    # West
    make_physics_wall(
        "Wall_W",
        pos=(cx - size_x * 0.5, cy, z_mid),
        scale=(wall_thickness * 0.5, size_y * 0.5, wall_height * 0.5),
    )



parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
stage = my_world.stage


xs=4.0
ys=4.5
zs=2.0
#schuppen = my_world.scene.add(
#    DynamicCuboid(
#        prim_path="/World/Schuppen",
#        name="Schuppen",
#        position=np.array([0, 0, zs/2]),
#        scale=np.array([xs, ys, zs]),
#        #size=1.0,
#        color=np.array([255, 0, 0]),
#    )
#)
#schuppen.set_local_scale(Gf.Vec3f(xs, ys, zs))

xz=50.0
yz=0.05
zz=1.6
#zaun = my_world.scene.add(
#    DynamicCuboid(
#        prim_path="/World/Zaun",
#        name="Zaun",
#        position=np.array([0, ys/2+1.0, zz/2]),
#        scale=np.array([xz, yz, zz]),
#        #size=1.0,
#        color=np.array([0, 0, 255]),
#    )
#)


# Rechteck-Raum mit vier Wänden erzeugen
#create_rect_room(
#    stage,
#    center=(0.0, 0.0, 0.0),
#    size_x=20.0,          # Länge in X
#    size_y=5.0,          # Breite in Y
#    wall_height=1.6,
#    wall_thickness=0.05,
#    base_z=0.0
#)

# Zaun erzeugen 
fence = Fence(stage, "/World/Fence1")
f_xf = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/Fence1"))
f_xf.SetTranslate((0, ys/2+1.0, 0.1))    # 1m über Boden für klare Sicht


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Carter/carter_v1_physx_lidar.usd"

quat = euler_angles_to_quat([0, 0, np.pi])
my_carter = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Carter",
        name="my_carter",
        wheel_dof_names=["left_wheel", "right_wheel"],
        orientation=quat,
        create_robot=True,
        usd_path=asset_path,
        position=np.array([xs, 1.0, 0.3]),
    )
)



my_controller = DifferentialController(name="simple_control", wheel_radius=0.24, wheel_base=0.56)




# -------------------------------------------------------
# Sonnenlicht (Distant Light)
# -------------------------------------------------------
sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/SunLight"))
sun.CreateIntensityAttr(5000.0)
sun.CreateAngleAttr(0.53)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.8))

sun_prim = stage.GetPrimAtPath("/World/SunLight")
sun_xf = UsdGeom.XformCommonAPI(sun_prim)
#sun_xf.SetRotate((-45.0, 0.0, 0.0))  # leicht schräg


# -------------------------------------------------------
# Kamera auf Top-View ausrichten
# -------------------------------------------------------
cam_path = "/OmniverseKit_Persp"
cam_prim = stage.GetPrimAtPath(cam_path)
cam_xf = UsdGeom.XformCommonAPI(cam_prim)

cam_xf.SetTranslate((0.0, 0.0, 25.0))
cam_xf.SetRotate((90.0, 0.0, 0.0))     # Pitch 90° = nach unten

set_camera_view(
    (0.0, 0.0, 15.0),   # eye
    (0.0, 0.0, 0.0),    # target
    cam_path
)


lidar = my_world.scene.add(
            RotatingLidarPhysX(
                prim_path="/World/Carter/chassis_link/lidar", 
                name="lidar", 
                fov=(360.0, 1.0),
                resolution=(1.0, 1.0),
                translation=np.array([-0.06, 0, 0.38])
            )
        )

my_lidar = wf.Lidar(lidar)
follower = wf.WallFollowerFinal()

my_world.reset()
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        dist = my_lidar.GetDistArray()
        if len(dist) > 0: 
            v, omega = follower.step(dist)
            my_carter.apply_wheel_actions(my_controller.forward(command=[v, omega]))
    if args.test is True:
        break
simulation_app.close()
