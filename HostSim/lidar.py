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

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()


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
zaun = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/Zaun",
        name="Zaun",
        position=np.array([0, ys/2+1.0, zz/2]),
        scale=np.array([xz, yz, zz]),
        #size=1.0,
        color=np.array([0, 0, 255]),
    )
)
#zaun.set_local_scale(Gf.Vec3f(xz, yz, zz))


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


# -------------------------------------------------------------------
#  PARALELLE WANDVERFOLGUNG
# -------------------------------------------------------------------
# Ziel: Rechtswand im Abstand d_des halten
d_des = 0.5           # Wunschabstand zur Wand (m)
Kp    = 0.05           # Proportionalfaktor für Winkelgeschwindigkeit
v_fwd = 0.5           # konstante Vorwärtsgeschwindigkeit (m/s)
w = 0.0               # Winkelgeschwindigkeit


class Lidar():
    def __init__(self, world):
        self.frameCnt=-1
        self.last_step = -1
        self.dist = np.zeros(360)
        self.lidar = world.scene.add(
            RotatingLidarPhysX(
                prim_path="/World/Carter/chassis_link/lidar", 
                name="lidar", 
                fov=(360.0, 1.0),
                resolution=(1.0, 1.0),
                translation=np.array([-0.06, 0, 0.38])
            )
        )
        self.lidar.initialize()                   # wichtig
        self.lidar.add_point_cloud_data_to_frame()
        self.lidar.enable_visualization()
        
    def GetDistArray(self):
        # LiDAR-Frame holen
        frame = self.lidar.get_current_frame()
        if frame is None:
            return []

        new_step = frame["physics_step"]
        if new_step == self.last_step:
            return []
        self.last_step = new_step
        
        self.frameCnt += 1
        # skip first dummy frame
        if self.frameCnt == 0:
            return []
            
        pc = frame["point_cloud"].reshape(-1, 3)   # shape (N, 1, 3) – Punkte im Lidar/Robot-Frame

        numPoints = len(pc[:, 0])
        
        for i in range(numPoints):
            x = pc[i, 0]
            y = pc[i, 1]
            r = np.sqrt(x*x + y*y)
            theta = int(round((math.atan2(y, x)*180/math.pi)))
            if theta<0: theta += 360
            self.dist[theta] = r

        if self.frameCnt==3:
            self.frameCnt = 0
            return self.dist

        return []



my_lidar = Lidar(my_world)
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
            #v, omega = RafaelWallFollower.step(dist)
            my_carter.apply_wheel_actions(my_controller.forward(command=[v, omega]))
    if args.test is True:
        break
simulation_app.close()
