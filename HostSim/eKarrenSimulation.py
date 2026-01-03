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

# Zugriff auf die interne Sim-Uhr
from omni.timeline import get_timeline_interface

from WallFollower import WallFollower
import IsaacSimLib as isl
from Fence import CreateFence, CreateWalls, CreateCylinder
from Garten import CreateGarten

# Robot & Lidar parameters
backWheelDrive = True
posX = 15.00
posY = 7.50
yaw = -np.pi+3/4*np.pi
scaleFactor = 1.5
measPerDeg = 4

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
stage = my_world.stage
timeline = get_timeline_interface()


from omni.isaac.core.utils.prims import get_prim_at_path, get_all_matching_child_prims
from pxr import PhysxSchema

def stabilize_nova_carter(carter_root="/World/Nova_Carter",
                          chassis_damping=(4.0, 0.8),
                          wheel_damping=10.0):
    """
    Stabilisiert Nova Carter:
    - chassis_link: Angular + Linear Damping
    - alle Räder (Articulation Drives): Angular Damping
    """

    stage = get_prim_at_path("/World").GetStage()

    # 1️⃣ Chassis Damping
    chassis = get_prim_at_path(f"{carter_root}/chassis_link")
    if chassis:
        rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(chassis)
        rb_api.CreateAngularDampingAttr().Set(chassis_damping[0])
        rb_api.CreateLinearDampingAttr().Set(chassis_damping[1])
        print(f"[NovaCarter] Chassis damping gesetzt: Angular={chassis_damping[0]}, Linear={chassis_damping[1]}")
    else:
        print("[NovaCarter] chassis_link nicht gefunden!")

    # 2️⃣ Räder Damping
    all_prims = get_all_matching_child_prims(carter_root, lambda p: True)
    wheel_count = 0
    for prim in all_prims:
        # Wir suchen alle Articulation Joints (Revolute / Hinge)
        if prim.GetTypeName() in ["PhysxRevoluteJoint", "PhysxSphericalJoint"]:
            # Angular Damping direkt auf das Drive-Attribute setzen
            # Name des Attributes: "angularDrive.damping"
            attr = prim.GetAttribute("angularDrive.damping")
            if attr:
                attr.Set(wheel_damping)
                wheel_count += 1
                print(f"[NovaCarter] Wheel drive damping gesetzt: {prim.GetPath()} -> {wheel_damping}")

    if wheel_count == 0:
        print("[NovaCarter] Keine Wheel Drives gefunden!")

def GetPositionAndTime():
    #time = my_world.current_time
    time = timeline.get_current_time()
    prim = XFormPrim("/World/eKarren/chassis_link")
    pos, quat = prim.get_world_pose()
    posX = pos[0]
    posY = pos[1]
    from scipy.spatial.transform import Rotation as R
    # (w,x,y,z) → (x,y,z,w)
    quat_scipy = [quat[1], quat[2], quat[3], quat[0]]
    r = R.from_quat(quat_scipy)   # (x, y, z, w)
    yaw = r.as_euler("xyz", degrees=True)[2]
    if backWheelDrive: 
        yaw += 180
        if yaw > 360: yaw -= 360
    return posX, posY, yaw, time

def SetCamera():
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

    
def SetLight():
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
# eKarren
# -------------------------------------------------------
def CreateRobot(posX, posY, yaw):
    eKarrenPath = "/World/eKarren"
    asset_path = "/bin/Robots/NVIDIA/NovaCarter/nova_carter.usd"
    add_reference_to_stage(asset_path, eKarrenPath)

    stabilize_nova_carter(eKarrenPath)
    
    # 3. Skalierung & Position (Einfach und lesbar über XFormPrim)
    # XFormPrim ist wie ein "Schweizer Taschenmesser" für Objekte
    from omni.isaac.core.prims import XFormPrim
    robot_xform = XFormPrim(eKarrenPath)
    robot_xform.set_local_scale(np.array([scaleFactor, scaleFactor, scaleFactor]))
    #robot_xform.set_world_pose(position=np.array([0.0, 0.0, 0.5]))
    eKarrenWidth = 0.78
    eKarrenLength = 1.1
    posZ= 0.30
    quat = euler_angles_to_quat([0, 0, yaw])
    my_carter = WheeledRobot(
            prim_path=eKarrenPath,
            name="eKarren",
            wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
            orientation=quat,
            create_robot=True,
            usd_path=asset_path,
            position=np.array([posX, posY, posZ]),
        )
    my_world.scene.add(my_carter)
    return my_carter

# -------------------------------------------------------
# Create Lidar
# -------------------------------------------------------
def CreateLidar():
    lidarX = -0.40*scaleFactor if backWheelDrive else 0
    lidarZ = 0.34*scaleFactor
    lidar = my_world.scene.add(
                RotatingLidarPhysX(
                    prim_path="/World/eKarren/chassis_link/hbLidar", 
                    name="lidar", 
                    rotation_frequency = 10,
                    fov=(360.0, 1.0),
                    resolution=(1.0/measPerDeg, 1.0),
                    translation=np.array([lidarX, 0, lidarZ]) 
                )
            )
    # 3. FIX: Die Welt-Skalierung des Lidars auf 1.0 zurücksetzen
    from omni.isaac.core.prims import XFormPrim
    lidar_xform = XFormPrim(lidar.prim_path)
    # Berechne den Kehrwert der Roboter-Skalierung
    inv_scale = 1.0 / scaleFactor
    lidar_xform.set_local_scale(np.array([inv_scale, inv_scale, inv_scale]))
    return lidar


# -------------------------------------------------------
# Garten simulieren
# -------------------------------------------------------
SetCamera()
SetLight()
CreateGarten(stage)
my_carter = CreateRobot(posX, posY, yaw)
lidar = CreateLidar()

my_controller = DifferentialController(name="simple_control", wheel_radius=0.218, wheel_base=0.68)
my_lidar = isl.Lidar(lidar, measPerDeg, backWheelDrive, "127.0.0.1")
robotCtrl = isl.RobotCtrl()

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
            #follower.Init()
            my_lidar.Init()
        posX, posY, yaw, time = GetPositionAndTime()
        dist = my_lidar.GetDistArray(posX, posY, yaw, time)
        #if len(dist) > 0: 
            #v, omega = follower.step(dist)
        cmd, params = robotCtrl.GetCmd()
        if cmd == isl.CMD_VELOCITY:
            v, omega = params
            v = -v if backWheelDrive else v
            my_carter.apply_wheel_actions(my_controller.forward(command=[v, omega]))
            
simulation_app.close()
