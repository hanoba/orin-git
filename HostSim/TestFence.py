# TestFence.py
#
# Start:
#   C:\bin\isaac-sim\python.bat fence_demo.py

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.usd import get_context
from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Gf, Sdf
import math
from Fence import Fence


# ============================================================
# Szene erzeugen
# ============================================================
stage = get_context().get_stage()

UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# Heller Boden
ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/Ground"))
ground.CreatePointsAttr([(-10,-10,0), (10,-10,0), (10,10,0), (-10,10,0)])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0,1,2,3])
UsdGeom.Gprim(ground).CreateDisplayColorAttr([(0.9, 0.9, 0.9)])


# Sonnenlicht
sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Sun"))
sun.CreateIntensityAttr(700)
sun.CreateAngleAttr(0.3)


# Zaun erzeugen (in die Luft gehoben)
fence = Fence(stage, "/World/Fence1")
f_xf = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/Fence1"))
f_xf.SetTranslate((0, 0, 1.0))    # 1m über Boden für klare Sicht


# Kamera positionieren
cam = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/OmniverseKit_Persp"))
cam.SetTranslate((3, -4, 2))
cam.SetRotate((-20, 0, 30))


# ============================================================
# Simulation laufen lassen
# ============================================================
print("Simulation läuft – Fenster offen lassen. Oben auf STOP drücken zum Beenden.")
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
