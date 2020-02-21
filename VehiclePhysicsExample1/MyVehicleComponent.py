import bge
from collections import OrderedDict
import VehiclePhysics

if not hasattr(bge, "__component__"):
    scene = bge.logic.getCurrentScene()

class MyVehicleComponent(bge.types.KX_PythonComponent):
    args = OrderedDict([
    ])

    def start(self, args):
        self.Vehicle = VehiclePhysics.VehiclePhysics(self.object)

    def update(self):
        pass