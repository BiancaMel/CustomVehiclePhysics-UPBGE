import bge
from collections import OrderedDict
from numpy import clip as Clamp


if not hasattr(bge, "__component__"):
    events = bge.events
    k = bge.logic.keyboard.inputs
    logic = bge.logic

class MyPlayerController(bge.types.KX_PythonComponent):
    args = OrderedDict([
    ])
    
    def start(self, args):
        self.VehicleComponent = self.object.components[0]
    
    def update(self):        
        #Get keyboard input
        Throttle = (k[events.WKEY].values[-1]-k[events.SKEY].values[-1])
        self.MoveForward = Throttle
        
        Steering = (k[events.DKEY].values[-1]-k[events.AKEY].values[-1])
        self.MoveRight = Steering
        
        #Apply input
        self.VehicleComponent.Vehicle.SetThrottle(Clamp(self.MoveForward, -1.0, 1.0))
        self.VehicleComponent.Vehicle.SetBrake(abs(Clamp(self.MoveForward, -1.0, 0.0)))
        self.VehicleComponent.Vehicle.SetSteering(self.MoveRight)
        
        #Change Gear, by default have 2 forward gears, N and reverse
        if 1 in k[events.EKEY].queue:
            self.VehicleComponent.Vehicle.Transmission.SetGear(self.VehicleComponent.Vehicle.Transmission.CurrentGear+1)
        if 1 in k[events.QKEY].queue:
            self.VehicleComponent.Vehicle.Transmission.SetGear(self.VehicleComponent.Vehicle.Transmission.CurrentGear-1)
        
	#Reload wheels setup
        if 1 in k[events.F2KEY].queue:
            self.VehicleComponent.Vehicle.ReloadSetup()