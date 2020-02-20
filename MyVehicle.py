import bge
from collections import OrderedDict
import VehiclePhysics
import MathLib
import math

if not hasattr(bge, "__component__"):
    scene = bge.logic.getCurrentScene()

class MyVehicle(bge.types.KX_PythonComponent):
    args = OrderedDict([
    ])

    def start(self, args):
        self.Vehicle = VehiclePhysics.CavaloComRoda(self.object)
        self.Camera = [obj for obj in self.Vehicle.childrenRecursive if "Camera" in obj][0]
        self.CameraFollow = False

    def update(self):
        #self.Vehicle.PreUpdate()
        if self.CameraFollow:
            self.UpdateCamera()
    
    def UpdateCamera(self):
        #VehicleRotWorld = self.Vehicle.worldOrientation.to_euler()
        #VehicleRotWorld.rotate_axis('Z', math.radians(90))
        
        VehicleRotLocal = self.Vehicle.localOrientation.to_euler()
        VehicleRotLocal.rotate_axis('X', math.radians(-5.0))
        VehicleRotLocal[1] = 0.0
        #TargetWorld = VehicleRotWorld.to_matrix()
        TargetLocal = VehicleRotLocal.to_matrix()
        
        
        Speed = self.Vehicle.DeltaTime*5
        self.Camera.localOrientation = self.Camera.localOrientation.lerp(TargetLocal, Speed)
        #self.Camera.worldOrientation = TargetWorld#self.Camera.worldOrientation.lerp(TargetWorld, Speed)