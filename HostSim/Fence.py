# fence_demo.py
#
# Start:
#   C:\bin\isaac-sim\python.bat fence_demo.py

#from omni.isaac.kit import SimulationApp
#simulation_app = SimulationApp({"headless": False})

from omni.usd import get_context
from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Gf, Sdf, Vt
from omni.isaac.core.utils.prims import create_prim
import math
import numpy as np


class Fence:
    def __init__(self, stage, path, 
            width=10.0, 
            height=1.3,
            radius = 0.0015,    # sichtbar runde Drähte
            step = 0.1          # Maschenweite
        ):    
        # ============================================================
        # Zaunelement: +45° und −45° Drähte (in einer XZ-Ebene)
        # ============================================================
        root = Sdf.Path(path)
        UsdGeom.Xform.Define(stage, root)
    
        # +45° Linien (steigend)
        x = -width/2 - height + step
        i = 0
        while x <= width/2 - step:
            px = x
            pz = 0
            if px < -width/2:
                px = -width/2
                pz = -width/2 - x
            p0 = Gf.Vec3d(px, 0, pz)
            #p0 = Gf.Vec3d(x, 0, 0)

            px = x + height
            pz = min(width/2 - x, height)
            if pz < height: px = width/2
            p1 = Gf.Vec3d(px, 0, pz)
            #p1 = Gf.Vec3d(x + height, 0, height)
            
            self.create_cylinder_between(stage, root, f"P45_{i}", p0, p1, radius)
            x += step
            i += 1
    
        # −45° Linien (fallend)
        x = -width/2 - height + step
        j = 0
        while x <= width/2 - step:
            px = x
            pz = height
            if px < -width/2:
                px = -width/2
                pz = height + width/2 + x
            p0 = Gf.Vec3d(px, 0, pz)
            #p0 = Gf.Vec3d(x, 0, height)

            px = x + height
            pz = 0
            if px > width/2: 
                pz = px - width/2
                px = width/2
            p1 = Gf.Vec3d(px, 0, pz)
            #p1 = Gf.Vec3d(x + height, 0, 0)
            self.create_cylinder_between(stage, root, f"M45_{j}", p0, p1, radius)
            x += step
            j += 1
    
        print("Zaunelement erzeugt:", path)


    # ============================================================
    # Hilfsfunktion: Cylinder zwischen zwei Punkten erzeugen
    # ============================================================
    def create_cylinder_between(self, stage, parent, name, p0, p1,radius):
        """Erzeugt einen Z-orientierten Zylinder und richtet ihn so aus,
        dass er exakt zwischen p0 und p1 liegt."""
    
        prim_path = parent.AppendChild(name)
        cyl = UsdGeom.Cylinder.Define(stage, prim_path)
    
        # Länge
        d = p1 - p0
        length = d.GetLength()
        cyl.CreateHeightAttr(length)
        cyl.CreateRadiusAttr(radius)
        cyl.CreateAxisAttr("Z")        # Zylinder-Längsachse = Z
    
        # Mittelpunkt
        mid = (p0 + p1) * 0.5
    
        # Richtung normalisieren
        #dx, dy, dz = d
        n = d.GetNormalized()
        nx, ny, nz = n
    
        # Rotation:
        # yaw   = Drehung um Z
        # pitch = Neigung um X
        yaw = math.degrees(math.atan2(ny, nx))
        pitch = -math.degrees(math.atan2(nz, math.sqrt(nx*nx + ny*ny)))
        hb = math.degrees(math.atan2(nz, nx))
    
        xf = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(prim_path))
        xf.SetTranslate(mid)
        #xf.SetRotate((pitch, 0, yaw))
        xf.SetRotate((0, hb, 0))
    
        # Materialfarbe (schwarz)
        gprim = UsdGeom.Gprim(stage.GetPrimAtPath(prim_path))
        #gprim.CreateDisplayColorAttr([(0.75, 0.78, 0.82)])
        gprim.CreateDisplayColorAttr([(0.0, 0.0, 0.0)])
    
        # Physik
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(prim_path))
        rb = UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(prim_path))
        rb.CreateRigidBodyEnabledAttr(False)
        PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(prim_path))


def CreateFence(
    stage,
    fenceLenX   = 10.0,
    fenceLenY   =  6.0,
    fenceZ      =  0.1,    # 10 cm über dem Boden für klare Sicht
    fenceHeight =  1.3,    # Höhe des Zaunes
    S = 0.1,               # Maschenweite
    R = 0.05               # Zaun ==> Mauer (R=S/2)
    #R = 0.0015            # sichtbar runde Drähte
):
    # Zaun erzeugen 
    fenceNorth = Fence(stage, "/World/ZaunNord", width=fenceLenX, height=fenceHeight, radius=R, step=S)
    f = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/ZaunNord"))
    f.SetTranslate((0, fenceLenY/2, fenceZ))

    fenceWest = Fence(stage, "/World/ZaunWest", width=fenceLenY, height=fenceHeight, radius=R, step=S)
    f = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/ZaunWest"))
    f.SetRotate((0, 0, 90))
    f.SetTranslate((-fenceLenX/2, 0.0, fenceZ))

    fenceSouth = Fence(stage, "/World/ZaunSüd", width=fenceLenX, height=fenceHeight, radius=R, step=S)
    f = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/ZaunSüd"))
    f.SetTranslate((0, -fenceLenY/2, fenceZ))

    fenceOst = Fence(stage, "/World/ZaunOst", width=fenceLenY, height=fenceHeight, radius=R, step=S)
    f = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/ZaunOst"))
    f.SetRotate((0, 0, 90))
    f.SetTranslate((fenceLenX/2, 0.0, fenceZ))


def CreateCube(stage, prim_path, posX, posY, lenX, lenY, height=1.6):

    # Geometrisches Cube-Prim
    create_prim(
        prim_path=prim_path,
        prim_type="Cube",
        translation=Gf.Vec3d(posX+lenX/2, posY+lenY/2, height/2),
        scale=Gf.Vec3f(lenX, lenY, height),
        attributes={"size": 1.0},
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


def CreateRotatedCube(stage, prim_path, posX, posY, lenX, lenY, height=1.6, angle_deg=0):
    from omni.isaac.core.utils.prims import create_prim, set_prim_property
    from pxr import Gf
    import numpy as np

    # 1. Erstellen
    create_prim(
        prim_path=prim_path,
        prim_type="Cube",
        attributes={"size": 1.0}
    )

    prim = stage.GetPrimAtPath(prim_path)

    xformable = UsdGeom.Xformable(prim)
    
    # Zuerst alle alten Ops löschen, um Konflikte zu vermeiden
    xformable.ClearXformOpOrder()
    
    # Die Reihenfolge in der Liste bestimmt die Ausführung.
    # Für Skalieren -> Drehen -> Verschieben muss die Liste so aussehen:
    # [Translate, Rotate, Scale]
    # Wir erzwingen die Double-Präzision (PrecisionDouble), 
    # damit es zum existierenden "double3" passt:
    translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    rotate_op    = xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
    scale_op     = xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)    
    
    # Werte setzen
    translate_op.Set(Gf.Vec3d(float(posX), float(posY), float(height/2)))
    rotate_op.Set(Gf.Vec3f(0.0, 0.0, float(angle_deg)))
    scale_op.Set(Gf.Vec3f(float(lenX), float(lenY), float(height)))

    # Farbe als RGB definieren (Werte von 0.0 bis 1.0)
    # Hier ein helles Grau für den Zaun:
    farbe = Gf.Vec3f(1.0, 0.0, 0.0)

    # Das Attribut erstellen oder holen und setzen
    # Wir müssen es in eine Liste [] packen, da displayColor ein Array ist
    prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(Vt.Vec3fArray([farbe]))

    
    # Kollisions-API anwenden
    UsdPhysics.CollisionAPI.Apply(prim)

    # Physikalischer Body (statisch)
    body = UsdPhysics.RigidBodyAPI.Apply(prim)
    body.CreateRigidBodyEnabledAttr(False)  # static geometry

    # PhysX-Mesh / Collider
    collider = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    collider.CreateDisableGravityAttr(True)




def CreateCylinder(stage, prim_path, posX, posY, dm, height=1.6):

    # Geometrisches Cube-Prim
    # Create the cylinder
    cylinder_prim = create_prim(
        prim_path=prim_path,
        prim_type="Cylinder",
        position=np.array([posX, posY, 0.5*height]), # X=5, Y=0, Z=0.5 (stands on floor)
        attributes={
            "radius": dm/2, 
            "height": height,
            "axis": "Z"    # Orientation: "X", "Y", or "Z"
        }
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
        

def CreateWalls(
    stage,
    center=(0.0, 0.0, 0.0),
    size_x=8.0,
    size_y=6.0,
    wall_height=2.5,
    wall_thickness=0.1,
    base_z=0.0,
    name = "/World/Room"):
    """
    Erzeugt 4 physikalisch feste Wände (statisch, kollidierbar).
    """

    cx, cy, cz = center
    z_mid = base_z + wall_height * 0.5

    room_root = name
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
