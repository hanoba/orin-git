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

import carb
import numpy as np
import math
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.physx import RotatingLidarPhysX
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, Usd, UsdLux

import omni.usd
import omni.kit.viewport.window as vp_win
import omni.kit.viewport.window as viewport_api
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim

import WallFollower as wf
from Fence import CreateFence, CreateWalls, CreateCylinder
from Garten import CreateGarten


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
stage = my_world.stage



# -------------------------------------------------------
# Kamera auf echte Top-View setzen
# -------------------------------------------------------
# 1. Wir zwingen die App kurz, das GUI zu aktualisieren
simulation_app.update()

# 2. Den Viewport über die Instanz-Suche holen
try:
    # 1. Alle Viewport-Instanzen holen
    all_viewports = list(vp_win.get_viewport_window_instances())
    
    if all_viewports:
        v_window = all_viewports[0]
        
        # 2. In 4.2 holt man die Kamera über das API-Objekt des Fensters
        v_api = v_window.viewport_api
        camera_path = v_api.get_active_camera()
        
        # 3. Die Kamera auf echte 2D-Draufsicht umstellen
        stage = omni.usd.get_context().get_stage()
        camera_prim = stage.GetPrimAtPath(camera_path)
        
        if camera_prim.IsValid():
            # 1. Kamera Setup
            usd_cam = UsdGeom.Camera(camera_prim)
            usd_cam.GetProjectionAttr().Set(UsdGeom.Tokens.orthographic)
            
            # 2. Sichtbereich (Zoom)
            # Wenn 200 noch zu nah ist, geh auf 500 oder 1000.
            # Dieser Wert definiert die Breite deines Sichtfensters in Welt-Einheiten.
            breite = 500.0
            usd_cam.GetHorizontalApertureAttr().Set(breite)
            
            # 4. Kamera-Position setzen (Draufsicht)
            from omni.isaac.core.utils.viewports import set_camera_view
            set_camera_view(eye=[0, 0, 20], target=[0, 0, 0])
            
            print(f"Echte 2D-Ansicht aktiv auf: {camera_path}")
except Exception as e:
    print(f"Letzter Versuch fehlgeschlagen: {e}")

# -------------------------------------------------------
# Sonnenlicht (Distant Light)
# -------------------------------------------------------
sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/SunLight"))
sun.CreateIntensityAttr(5000.0)
sun.CreateAngleAttr(0.53)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.8))

sun_prim = stage.GetPrimAtPath("/World/SunLight")
sun_xf = UsdGeom.XformCommonAPI(sun_prim)
sun_xf.SetRotate((-15.0, 0.0, 0.0))  # leicht schräg

# -------------------------------------------------------
# Garten simulieren
# -------------------------------------------------------
CreateGarten(stage)

# -------------------------------------------------------
# eKarren
# -------------------------------------------------------
eKarrenPath = "/World/eKarren"
asset_path = "/bin/Robots/NVIDIA/NovaCarter/nova_carter.usd"
add_reference_to_stage(asset_path, eKarrenPath)

# 3. Skalierung & Position (Einfach und lesbar über XFormPrim)
# XFormPrim ist wie ein "Schweizer Taschenmesser" für Objekte
robot_xform = XFormPrim(eKarrenPath)
scaleFactor = 1.5
robot_xform.set_local_scale(np.array([scaleFactor, scaleFactor, scaleFactor]))
robot_xform.set_world_pose(position=np.array([0.0, 0.0, 0.5]))
eKarrenWidth = 0.78
eKarrenLength = 1.1


posX= 5.97 
posY=-1.63 
yaw = np.pi
quat = euler_angles_to_quat([0, 0, yaw])
my_carter = WheeledRobot(
        prim_path=eKarrenPath,
        name="eKarren",
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        orientation=quat,
        create_robot=True,
        usd_path=asset_path,
        position=np.array([-5.00, 0, 0.3]),
    )
my_world.scene.add(my_carter)

my_controller = DifferentialController(name="simple_control", wheel_radius=0.218, wheel_base=0.68)

# -------------------------------------------------------
# Lidar
# -------------------------------------------------------
measPerDeg = 4
backWheelDrive = True
lidarX = -0.6 if backWheelDrive else 0
lidar = my_world.scene.add(
            RotatingLidarPhysX(
                prim_path=f"{eKarrenPath}/chassis_link/hbLidar", 
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
inv_scale = 1.0 / scaleFactor
lidar_xform.set_local_scale(np.array([inv_scale, inv_scale, inv_scale]))

my_lidar = wf.Lidar(lidar, measPerDeg, backWheelDrive)
follower = wf.WallFollowerFinal(my_world, target_dist=2.00, max_speed=0.5)

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
        dist = my_lidar.GetDistArray()
        if len(dist) > 0: 
            v, omega = follower.step(dist)
            v = -v if backWheelDrive else v
            my_carter.apply_wheel_actions(my_controller.forward(command=[v, omega]))
            
simulation_app.close()
