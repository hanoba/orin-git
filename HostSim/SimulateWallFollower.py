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
import omni.usd
import WallFollower as wf
from Fence import CreateFence, CreateWalls

from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.prims import XFormPrim
from pxr import UsdLux, UsdGeom, Sdf, Gf

from omni.isaac.core.utils.prims import create_prim
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, Usd

def get_robot_bounding_box(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    
    if not prim.IsValid():
        print(f"Fehler: Prim unter {prim_path} nicht gefunden!")
        return
    
    # Korrekte Initialisierung des BBoxCache
    # Wir nutzen Usd.TimeCode.Default() für die Standardzeit
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    
    # Berechnet die Bounding Box im Weltkoordinatensystem
    bbox = bbox_cache.ComputeWorldBound(prim)
    range_box = bbox.GetRange()
    
    # Min- und Max-Punkte
    min_pt = range_box.GetMin()
    max_pt = range_box.GetMax()
    
    # Dimensionen berechnen
    dims = max_pt - min_pt
    
    print(f"--- Bounding Box für {prim_path} ---")
    print(f"  Länge (X): {dims[0]:.3f} m")
    print(f"  Breite (Y): {dims[1]:.3f} m")
    print(f"  Höhe (Z):   {dims[2]:.3f} m")
    print(f"  Min Punkt:  {min_pt}")
    print(f"  Max Punkt:  {max_pt}")
    
    return dims


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
stage = my_world.stage


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


#asset_path = "/bin/Robots/NovaCarterScaled/nova_carter.usd"
asset_path = "/bin/Robots/NVIDIA/NovaCarter/nova_carter.usd"
add_reference_to_stage(asset_path, "/World/eKarren")

# 3. Skalierung & Position (Einfach und lesbar über XFormPrim)
# XFormPrim ist wie ein "Schweizer Taschenmesser" für Objekte
robot_xform = XFormPrim("/World/eKarren")
robot_xform.set_local_scale(np.array([1.6, 1.6, 1.6]))
robot_xform.set_world_pose(position=np.array([0.0, 0.0, 0.5]))

fenceLenX = 15.0
fenceLenY =  9.0
#CreateFence(stage, fenceLenX=fenceLenX, fenceLenY=fenceLenY)
CreateWalls(stage, size_x=fenceLenX, size_y=fenceLenY, wall_height=1.6)

posX= 5.97 
posY=-1.63 
yaw=-118.5*np.pi/180*0
#quat = euler_angles_to_quat([0, 0, 1*np.pi])
quat = euler_angles_to_quat([0, 0, 1*np.pi+0*yaw])
my_carter = WheeledRobot(
        prim_path="/World/eKarren",
        name="eKarren",
        #wheel_dof_names=["left_wheel", "right_wheel"],
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        orientation=quat,
        create_robot=True,
        usd_path=asset_path,
        position=np.array([fenceLenX/4*1, fenceLenY/4*1, 0.3*1]),
        #position=np.array([posX, posY, 0.3]),
    )
my_world.scene.add(my_carter)
#my_carter.set_local_scale(np.array([1.6, 1.6, 1.6]))
#get_robot_bounding_box("/World/eKarren")

my_controller = DifferentialController(name="simple_control", wheel_radius=0.218, wheel_base=0.68)

measPerDeg = 4
backWheelDrive = True
lidarX = -0.6 if backWheelDrive else 0
lidar = my_world.scene.add(
            RotatingLidarPhysX(
                prim_path="/World/eKarren/chassis_link/hbLidar", 
                name="lidar", 
                rotation_frequency = 10,
                fov=(360.0, 1.0),
                resolution=(1.0/measPerDeg, 1.0),
                translation=np.array([lidarX, 0, 0.50])   # 0.38
            )
        )
# 3. FIX: Die Welt-Skalierung des Lidars auf 1.0 zurücksetzen
from omni.isaac.core.prims import XFormPrim
lidar_xform = XFormPrim(lidar.prim_path)
# Berechne den Kehrwert der Roboter-Skalierung
inv_scale = 1.0 / 1.6  # ergibt 0.625

# Setze die lokale Skalierung des Lidars
# Da SingleXFormPrim die Methode 'set_local_scale' besitzt:
lidar_xform.set_local_scale(np.array([inv_scale, inv_scale, inv_scale]))

my_lidar = wf.Lidar(lidar, measPerDeg, backWheelDrive)
follower = wf.WallFollowerFinal(my_world, target_dist=1.5, max_speed=0.5)

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
            follower.Init()
            my_lidar.Init()
        dist, angles = my_lidar.GetDistArray()
        if len(dist) > 0: 
            v, omega = follower.step(dist, angles)
            v = -v if backWheelDrive else v
            my_carter.apply_wheel_actions(my_controller.forward(command=[v, omega]))
simulation_app.close()
