# fence_demo.py
#
# Start:
#   C:\bin\isaac-sim\python.bat fence_demo.py

#from omni.isaac.kit import SimulationApp
#simulation_app = SimulationApp({"headless": False})

from omni.usd import get_context
from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Gf, Sdf
import math


class Fence:
    def __init__(self, stage, path, width=10.0, height=1.6):    
        # ============================================================
        # Parameter
        # ============================================================
        self.WIDTH = width
        self.HEIGHT = height
        self.RADIUS = 0.003       # sichtbar runde Drähte
        self.STEP = 0.1          # Maschenweite

        # ============================================================
        # Zaunelement: +45° und −45° Drähte (in einer XZ-Ebene)
        # ============================================================
        root = Sdf.Path(path)
        UsdGeom.Xform.Define(stage, root)
    
        # +45° Linien (steigend)
        x = -self.WIDTH/2 - self.HEIGHT + self.STEP
        i = 0
        while x <= self.WIDTH/2 - self.STEP:
            px = x
            pz = 0
            if px < -self.WIDTH/2:
                px = -self.WIDTH/2
                pz = -self.WIDTH/2 - x
            p0 = Gf.Vec3d(px, 0, pz)
            #p0 = Gf.Vec3d(x, 0, 0)

            px = x + self.HEIGHT
            pz = min(self.WIDTH/2 - x, self.HEIGHT)
            if pz < self.HEIGHT: px = self.WIDTH/2
            p1 = Gf.Vec3d(px, 0, pz)
            #p1 = Gf.Vec3d(x + self.HEIGHT, 0, self.HEIGHT)
            
            self.create_cylinder_between(stage, root, f"P45_{i}", p0, p1)
            x += self.STEP
            i += 1
    
        # −45° Linien (fallend)
        x = -self.WIDTH/2 - self.HEIGHT + self.STEP
        j = 0
        while x <= self.WIDTH/2 - self.STEP:
            px = x
            pz = self.HEIGHT
            if px < -self.WIDTH/2:
                px = -self.WIDTH/2
                pz = self.HEIGHT + self.WIDTH/2 + x
            p0 = Gf.Vec3d(px, 0, pz)
            #p0 = Gf.Vec3d(x, 0, self.HEIGHT)

            px = x + self.HEIGHT
            pz = 0
            if px > self.WIDTH/2: 
                pz = px - self.WIDTH/2
                px = self.WIDTH/2
            p1 = Gf.Vec3d(px, 0, pz)
            #p1 = Gf.Vec3d(x + self.HEIGHT, 0, 0)
            self.create_cylinder_between(stage, root, f"M45_{j}", p0, p1)
            x += self.STEP
            j += 1
    
        print("Zaunelement erzeugt:", path)


    # ============================================================
    # Hilfsfunktion: Cylinder zwischen zwei Punkten erzeugen
    # ============================================================
    def create_cylinder_between(self, stage, parent, name, p0, p1):
        """Erzeugt einen Z-orientierten Zylinder und richtet ihn so aus,
        dass er exakt zwischen p0 und p1 liegt."""
    
        prim_path = parent.AppendChild(name)
        cyl = UsdGeom.Cylinder.Define(stage, prim_path)
    
        # Länge
        d = p1 - p0
        length = d.GetLength()
        cyl.CreateHeightAttr(length)
        cyl.CreateRadiusAttr(self.RADIUS)
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
    
        # Materialfarbe (Metallisch hellgrau)
        gprim = UsdGeom.Gprim(stage.GetPrimAtPath(prim_path))
        gprim.CreateDisplayColorAttr([(0.75, 0.78, 0.82)])
    
        # Physik
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(prim_path))
        rb = UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(prim_path))
        rb.CreateRigidBodyEnabledAttr(False)
        PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(prim_path))
        
