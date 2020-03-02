import bge
from collections import OrderedDict
import time
from mathutils import Vector
from numpy import clip as Clamp, interp as Lerp
from copy import deepcopy
import math
import MathLib
import sys


if not hasattr(bge, "__component__"):
    pc = bge.constraints
    events = bge.events
    render = bge.render
    k = bge.logic.keyboard.inputs
    scene = bge.logic.getCurrentScene()
    #Input = bge.logic.Input
    logic = bge.logic
    #logic.addScene("HUD")

class PlayerController(bge.types.KX_PythonComponent):
    args = OrderedDict([
        ("Enable", True),
    ])
    
    def start(self, args):
        #bge.logic.setTimeScale(0.30)
        self.VehicleComponent = self.object.components[0]
        self.Cameras = [obj for obj in self.VehicleComponent.Vehicle.childrenRecursive if obj.__class__.__name__ == "KX_Camera"]
        self.SteerWheel = [obj for obj in self.VehicleComponent.Vehicle.childrenRecursive if "Volante" in obj][0]
        self.CamParent = [obj for obj in self.VehicleComponent.Vehicle.childrenRecursive if "CamParent" in obj][0]
        self.CamParentPos = self.CamParent.parent
        self.CamParent.removeParent()
        self.CurrentCamera = 0
        self.RotVolZero = self.SteerWheel.localOrientation.to_euler()
        scene.pre_draw_setup.append(self.PreUpdate)
        
    def PreUpdate(self):
        self.CamParent.worldPosition = self.CamParentPos.worldPosition
        self.CamParent.worldOrientation = self.CamParent.worldOrientation.lerp(self.CamParentPos.worldOrientation, self.VehicleComponent.Vehicle.DeltaTime*11)
    
    def update(self):
        #joystick
        Axis = [0,0,0,0,0,0]
        Buttons = []
        try:
                Axis = bge.logic.joysticks[0].axisValues
                Buttons = bge.logic.joysticks[0].activeButtons
        except:
                pass
        #print(Axis)
        
        RightRaw = Axis[0]
        SignRight = MathLib.Sign(RightRaw)
        DeadzoneRight = 0.0
        RightFinal = MathLib.MapRange(RightRaw, 0.0, SignRight, DeadzoneRight*SignRight, SignRight)
        RightFinal = RightFinal * MathLib.LerpF(abs(RightFinal), 1.0, 0.05)
        self.MoveRight = Clamp(RightFinal, -1.0, 1.0)
        
        
        ForwardRaw = -Axis[3]
        SignForward = MathLib.Sign(ForwardRaw)
        DeadzoneForward = -0.00
        self.MoveForward = MathLib.MapRange(ForwardRaw, 0.0, SignForward, DeadzoneForward*SignForward, SignForward)
        
        #Keyboard
        Throttle = (k[events.WKEY].values[-1]-k[events.SKEY].values[-1])
        if Throttle != 0:
            self.MoveForward = Throttle
        
        Steering = (k[events.DKEY].values[-1]-k[events.AKEY].values[-1])
        if Steering != 0:
            self.MoveRight = Steering
        
        #Apply input
        self.VehicleComponent.Vehicle.SetThrottle(Clamp(self.MoveForward, -1.0, 1.0))
        self.VehicleComponent.Vehicle.SetBrake(abs(Clamp(self.MoveForward, -1.0, 0.0)))
        self.VehicleComponent.Vehicle.SetSteering(self.MoveRight)
        
        #SteerWheel
        SteerV = self.VehicleComponent.Vehicle.Wheels[0].Steering.SteeringAngle
        RotVol = self.RotVolZero.copy()
        RotVol.rotate_axis('Y', math.radians(SteerV*90.0))
        TargetLocal = RotVol.to_matrix()
        self.SteerWheel.localOrientation = TargetLocal
        
        #Set Gear
        if 1 in k[events.EKEY].queue or 10 in Buttons:
            self.VehicleComponent.Vehicle.Transmission.SetGear(self.VehicleComponent.Vehicle.Transmission.CurrentGear+1)
        if 1 in k[events.QKEY].queue or 9 in Buttons:
            self.VehicleComponent.Vehicle.Transmission.SetGear(self.VehicleComponent.Vehicle.Transmission.CurrentGear-1)
        
        #Change camera
        if 1 in k[events.CKEY].queue:
            print(self.VehicleComponent.Vehicle.GetDebugValues())
            self.CurrentCamera += 1
            if self.CurrentCamera >= len(self.Cameras):
                self.CurrentCamera = 0
            scene.active_camera = self.Cameras[self.CurrentCamera]

        #Tests
        if 1 in k[events.PAD4].queue:
            Center = self.VehicleComponent.Vehicle.CenterOfMassOffset + Vector([0.0, -0.05, 0.0])
            self.VehicleComponent.Vehicle.SetCenterOfMassOffset(Center)
            print(Center)
        if 1 in k[events.PAD5].queue:
            Center = self.VehicleComponent.Vehicle.CenterOfMassOffset + Vector([0.0, 0.05, 0.0])
            self.VehicleComponent.Vehicle.SetCenterOfMassOffset(Center)
            print(Center)
        if 1 in k[events.PAD7].queue:
            Center = self.VehicleComponent.Vehicle.CenterOfMassOffset + Vector([0.0, 0.0, -0.05])
            self.VehicleComponent.Vehicle.SetCenterOfMassOffset(Center)
            print(Center)
        if 1 in k[events.PAD8].queue:
            Center = self.VehicleComponent.Vehicle.CenterOfMassOffset + Vector([0.0, 0.0, 0.05])
            self.VehicleComponent.Vehicle.SetCenterOfMassOffset(Center)
            print(Center)

        if 1 in k[events.F2KEY].queue:
            self.VehicleComponent.Vehicle.ReloadSetup()

        if 1 in k[events.F3KEY].queue:
            self.VehicleComponent.CameraFollow = not self.VehicleComponent.CameraFollow
            
        if k[events.PAD1].values[-1]:
            for Wheel in self.VehicleComponent.Vehicle.Wheels:
                Wheel.Tire.LatStiffnessFactor -= 0.01
                #Wheel.Tire.LatShapeFactor -= 0.01
                #Wheel.Tire.LatPeakValue  -= 0.01
                #Wheel.Tire.LatCurvatureFactor  -= 0.01
                #Wheel.Tire.LatTireLoad -= 0.01=
        if k[events.PAD2].values[-1]:
            for Wheel in self.VehicleComponent.Vehicle.Wheels:
                Wheel.Tire.LatStiffnessFactor += 0.01
                #Wheel.Tire.LatShapeFactor += 0.01
                #Wheel.Tire.LatPeakValue += 0.01
                #Wheel.Tire.LatCurvatureFactor  += 0.01
                #Wheel.Tire.LatTireLoad  += 0.01=
        if 1 in k[events.PAD0].queue:
            for Wheel in self.VehicleComponent.Vehicle.Wheels:
                print("\n Lateral")
                print(Wheel.Tire.LatStiffnessFactor)
                print(Wheel.Tire.LatShapeFactor)
                print(Wheel.Tire.LatPeakValue)
                print(Wheel.Tire.LatCurvatureFactor)
                print(Wheel.Tire.LatTireLoad)
                break


