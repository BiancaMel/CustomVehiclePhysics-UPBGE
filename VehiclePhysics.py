import bge
from collections import OrderedDict
import time
from mathutils import Vector
from numpy import clip as Clamp, interp as Lerp
from copy import deepcopy
import math
import MathLib
import sys
import VehicleWheel
import json

#if not hasattr(bge, "__component__"):

events = bge.events
render = bge.render
k = bge.logic.keyboard.inputs
scene = bge.logic.getCurrentScene()
#Input = bge.logic.Input
logic = bge.logic

class Gear():
    def __init__(self, Ratio, ShiftUp=0.9, ShiftDown=0.5):
        self.Ratio = Ratio
        self.ShiftUp = ShiftUp
        self.ShiftDown = ShiftDown

class Transmission():
    def __init__(self):
        #Costant Values
        self.ForwardGears = [Gear(4.06), Gear(3.5), Gear(2.2)]
        self.NeutralGear = Gear(0.0)
        self.ReverseGear = Gear(4.06)
        self.FinalDriveRatio = 3.70

        #Realtime Values
        self.CurrentGear = 1
        self.CurrentDriveRatio = 1.0
        self.TimeChange = time.time()
        self.SetGear(1, True)
    
    def SetGear(self, Number, Instant=False):
        if time.time() - self.TimeChange > 0.15 or Instant:
            self.TimeChange = time.time()
            self.CurrentGear = Clamp(Number, -1, 3)
            if self.CurrentGear > 0:
                self.CurrentDriveRatio = self.ForwardGears[self.CurrentGear-1].Ratio * self.FinalDriveRatio
            elif self.CurrentGear == -1:
                self.CurrentDriveRatio =  -self.ReverseGear.Ratio * self.FinalDriveRatio
            else:
                self.CurrentDriveRatio = self.NeutralGear.Ratio

class VehicleBase(bge.types.KX_GameObject):
    def __init__(self, oldOwner):
        self.MoveForward = 0.0
        self.MoveRight = 0.0
        self.Brake = 0.0
    
    def SetThrottle(self, Value):
        self.MoveForward = Clamp(Value, -1.0, 1.0)
    
    def SetBrake(self, Value):
        self.Brake = Clamp(Value, 0.0, 1.0)
    
    def SetSteering(self, Value):
        self.MoveRight = Clamp(Value, -1.0, 1.0)

class VehiclePhysics(VehicleBase):
    def __init__(self, Object):
        super().__init__(Object)
        self.Enable = True
        
        self.DeltaTime = 0.0
        self.EnableDebug = True

        self.Wheels = []
        self.NumDrivenWheels = 0
        self.CenterOfMassOffset = Vector([0.0, 0.0, -0.2])

        for ob in self.childrenRecursive:
            if "Collider" in ob:
                self.Collider = ob
                #print(self.Collider.localPosition)
                self.SetCenterOfMassOffset(self.CenterOfMassOffset)

            if "Wheel" in ob:
                child = ob.children[0]
                ob['wheelVisual'] = child
                child.removeParent()
                child.setParent(self)
                
                WheelObj = VehicleWheel.VehicleWheel(ob, self, self.LoadSetup(ob["Wheel"]))
                self.Wheels.append(WheelObj)
                if WheelObj.Wheel.bIsTraction:
                    self.NumDrivenWheels += 1
        
        #Constant Values
        self.EngineTorque = 600.0/self.NumDrivenWheels
        self.BrakeTorque = 3000
        self.MaxEngineTurnOver = 7500.0
        self.SteeringSensibility = 8.0
        self.Transmission = Transmission()
        
        #Realtime Values
        self.Speed = 0.0
        self.EngineTurnOver = 0.0
        self.EngineAngularVelocity = 0.0
        self.CurrentEngineTorque = 0.0
        self.CurrentBrakeTorque = 0.0
        
        
        
        
        self.LastForce = None
        self.LastTime = time.time()
        self.LastTimeTest = time.time()
        self.LastFixedUpdate = time.time()
        self.LastVelocity = self.localLinearVelocity.copy()
        self.PhysicsTime = time.time()
        scene.pre_draw_setup.append(self.PreUpdate)
    
    def SetThrottle(self, Value):
        self.MoveForward = Clamp(Value, 0.0, 1.0)
        #if Value >= 0.0 and MathLib.NearFloat(self.localLinearVelocity.y, 0.0, 1.0):
        #    self.Transmission.SetGear(1)
        #elif Value <= 0.0 and MathLib.NearFloat(self.localLinearVelocity.y, 0.0, 1.0):
        #    self.Transmission.SetGear(-1)


    def ReloadSetup(self):
        for i in self.Wheels:
            i.SetupWheelValues(self.LoadSetup(i["Wheel"]))
    
    def LoadSetup(self, SetupName):
        obj = None
        
        try:
            with open(logic.expandPath('//'+SetupName+'.json'), 'r') as myfile:
                data = myfile.read()
            obj = json.loads(data)
        except:
            print("Failed to open file: ", str(SetupName)+".json")
        
        return obj
    
    def DebugPhysics(self):
        for i in range(len(self.Wheels)):
            SuspensionName = "SuspensionVelocity{0}".format(i)
            
            if not SuspensionName in self:
                self.addDebugProperty(SuspensionName, True)
            
            self[SuspensionName] = self.Wheels[i].SuspensionVelocity
    
    def GetDebugValues(self):
        Values = ""
        Values += "Vehicle Velocity: {0:0.0f} KM/H\n".format(self.SpeedKMH)
        #Values += "\n"
        
        #Values += 'Gear: {0} \n'.format(self.Transmission.CurrentGear)
        #Values += '\n'
        #Values += 'PhysicsTime: {0:0.5f}ms \n'.format(self.PhysicsTime*10)
        #Values += '\n'

        """for i in range(len(self.Wheels)):
            Values += 'SuspensionVelocity[{0}]: {1:0.6f}\n'.format(i, self.Wheels[i].Suspension.CurrentVelocity)
        Values += '\n'
        
        for i in range(len(self.Wheels)):
            Values += 'SuspensionForceZ[{0}]: {1:0.2f}\n'.format(i, self.Wheels[i].Suspension.CurrentForceZ)
        Values += '\n'
        
        for i in range(len(self.Wheels)):
            Values += 'LongSlip[{0}]: {1:0.6f}\n'.format(i, self.Wheels[i].Tire.LongSlip)
        Values += '\n'
        
        for i in range(len(self.Wheels)):
            Values += 'LateralSlipAngle[{0}]: {1:0.6f}\n'.format(i, self.Wheels[i].Tire.mostra)
            #Values += 'LateralSlip[{0}]: X= {1:0.3f}, Y= {2:0.3f}, Z= {3:0.3f}\n'.format(i, self.Wheels[i].Tire.LateralSlip.x, self.Wheels[i].Tire.LateralSlip.y, self.Wheels[i].Tire.LateralSlip.z)
        Values += '\n'
        
        for i in range(len(self.Wheels)):
            #Values += 'LateralForce[{0}]: X= {1:0.3f}, Y= {2:0.3f}, Z= {3:0.3f}\n'.format(i, self.Wheels[i].Tire.LateralForce.x, self.Wheels[i].Tire.LateralForce.y, self.Wheels[i].Tire.LateralForce.z)
            #Values += 'LateralForce[{0}]: {1:0.3f}\n'.format(i, self.Wheels[i].Tire.LateralForce.length)
            pass
        Values += '\n'"""
        
    
        return Values
    
    def SetCenterOfMassOffset(self, Value):
        last = self.Collider.worldPosition.copy()
        self.CenterOfMassOffset = Value
        self.Collider.localPosition = -self.CenterOfMassOffset

        self.Collider.removeParent()
        self.Collider.setParent(self, True, False)
        self.worldPosition += (last-self.Collider.worldPosition)

    def PreUpdate(self):
        self.DeltaTime = Clamp(time.time() - self.LastTime, 0.0, 0.0333333)
        #PhyTime = time.time()
        
        
        self.Speed = self.worldLinearVelocity.magnitude
        self.SpeedKMH = self.worldLinearVelocity.magnitude * 3.6
        
        
        self.CurrentEngineTorque = self.MoveForward * self.EngineTorque
        self.CurrentBrakeTorque = self.Brake * self.BrakeTorque
        self.EngineTurnOver = MathLib.LerpF(self.EngineTurnOver, self.MoveForward*self.MaxEngineTurnOver, self.DeltaTime*3)
        self.EngineAngularVelocity = (6.28318530718*self.EngineTurnOver)/60
        #print(self.EngineTurnOver, self.EngineAngularVelocity)
        
        
        # TESTS ------------------------------------------------------
        if k[events.RKEY].values[-1]:
            self.applyForce([0, 0, -50000], False)
        if k[events.GKEY].values[-1]:
            self.applyTorque([0, 0, 10000], True)
        if k[events.HKEY].values[-1]:
            self.applyTorque([0, 0, -10000], True)
        if k[events.JKEY].values[-1]:
            self.worldLinearVelocity = Vector([0, 0, 0.00001])
        if k[events.TKEY].values[-1]:
            self.applyForce([0, 0, 100000], False)
        if k[events.LEFTARROWKEY].values[-1]:
            self.applyForce([-30000, 0, 0], True)
        if k[events.RIGHTARROWKEY].values[-1]:
            self.applyForce([30000, 0, 0], True)
        
        """if 1 in k[events.WKEY].queue:
            self.LastTimeTest = time.time()
        if not "speed" in self and self.SpeedKMH > 100.0:
            #sys.stdout.write("\r"+str(self.Speed*3.6))
            #sys.stdout.flush()
            self['speed'] =0
            print(time.time() - self.LastTimeTest)"""
        # ------------------------------------------------------

        
        UpdatePhysics = False
        FixedTimeUpdate = 0.02
        
        CurrentFixedTime = time.time() - self.LastFixedUpdate
        if CurrentFixedTime > 0.06:
            FixedTimeUpdate = self.DeltaTime
            self.LastFixedUpdate = time.time()
            UpdatePhysics = True
        elif CurrentFixedTime >= 0.010:
            FixedTimeUpdate = (time.time() - self.LastFixedUpdate)
            self.LastFixedUpdate = time.time()
            UpdatePhysics = True
        
        i = 0
        for WheelObject in self.Wheels:
            WheelObject.Suspension.StartPoint = WheelObject.worldPosition+(self.getAxisVect(Vector([0, 0, 1]))*WheelObject.Wheel.Radius)
            EndRayPoint = WheelObject.Suspension.StartPoint+(self.getAxisVect(Vector([0, 0, -1]))*WheelObject.Suspension.MaxDrop)
            Ray = WheelObject.rayCast(EndRayPoint, WheelObject.Suspension.StartPoint)
            
            if WheelObject.Steering.bIsSteering:
                WheelObject.Steering.UpdateSteeringAngle(WheelObject, self, self.MoveRight, self.SteeringSensibility, self.DeltaTime)
            
            if Ray[0] != None:
                #Test
                WHEELPOS = Ray[1]+WheelObject.getAxisVect(Vector([0, 0, 0.1]))
                
                SuspensionFinalForce = WheelObject.Suspension.GetSuspensionForce(WheelObject, Ray, self.DeltaTime)
                
                WheelObject.Wheel.UpdateRealtimeValues(WheelObject, self, Ray, self.DeltaTime)
                
                #Tire Forces
                FLong, FLateral = WheelObject.Tire.GetTireForces(WheelObject, self, self.DeltaTime)
                
                if UpdatePhysics:
                    #Apply Forces
                    #gametime = bge.logic.getTimeScale()
                    self.applyImpulse(Ray[1]+WheelObject.getAxisVect(Vector([0, 0, WheelObject.Suspension.ForceAppHeight])), (SuspensionFinalForce+FLong)*FixedTimeUpdate, False)
                    self.applyImpulse(Ray[1]+WheelObject.getAxisVect(Vector([0, 0, WheelObject.Suspension.ForceAppHeightLateral])), (FLateral)*FixedTimeUpdate, False)
                
                #Set Wheel Rotation                
                WheelRot = WheelObject.Wheel.LocalOrientation.to_euler()
                WheelRot[0] -= (WheelObject.Wheel.WheelRotation) * self.DeltaTime
                WheelRot[2] = WheelObject.localOrientation.to_euler()[2]
                WheelObject.Wheel.LocalOrientation = WheelRot.to_matrix()
                
                
                #render.drawLine(WHEELPOS, WHEELPOS+(SuspensionFinalForce)*0.0005,[1,0.0,0,1])
                #render.drawLine(WHEELPOS, WHEELPOS+(FLateral)*0.0005,[1,0.5,0,1])
                #render.drawLine(WHEELPOS, WHEELPOS+(FLong)*0.0005,[1,0,1,1])
                #render.drawLine(WheelObject.Suspension.StartPoint, Ray[1], [1,1.0,0,1])
                
                #Save last values
                WheelObject.Wheel.LastWorldPosition = Ray[1].copy()
            else:
                #render.drawLine(WheelObject.Suspension.StartPoint, EndRayPoint, [1,1.0,0,1])
                
                WheelObject.Suspension.CurrentLength = WheelObject.Suspension.MaxDrop
                WheelObject.Wheel.LocalPosition = (EndRayPoint - self.worldPosition) * self.worldOrientation + Vector([0, 0, WheelObject.Wheel.Radius])
                WheelObject.Wheel.LastPosition = WheelObject.worldPosition.copy()
            
            WheelObject.Suspension.LastLength = deepcopy(WheelObject.Suspension.CurrentLength)
            WheelObject.Wheel.LastLocalVelocity = WheelObject.Wheel.LocalWheelVelocity.copy()
            WheelObject.Wheel.LastWorldVelocity = WheelObject.Wheel.WorldWheelVelocity.copy()
            WheelObject.Wheel.LastNormalX = WheelObject.getAxisVect(Vector([1, 0, 0])).copy()
            
            
            
            #Test
            WheelObject['wheelVisual'].localPosition = WheelObject.Wheel.LocalPosition
            WheelObject['wheelVisual'].localOrientation = WheelObject.Wheel.LocalOrientation
        
            i += 1
        #if self.EnableDebug:
        #    self.DebugPhysics()
        
        self.LastVelocity = self.localLinearVelocity.copy()
        self.LastTime = time.time()
        #self.PhysicsTime = time.time() - PhyTime
        
class CarroComRodaEesteiraComAsaQueVoa(VehiclePhysics):
    def __init__(self, Object):
        super().__init__(Object)
    def VoaCarro(self):
        pass
    def EsteiraVai(self):
        pass
    def SuspensaoVai(self): #AH QUERO MUDAR COMO A SUSPENSAO FUNCIONA SÓ ISSO
        pass
        
class CavaloComRoda(CarroComRodaEesteiraComAsaQueVoa):
    def __init__(self, Object):
        super().__init__(Object)


"""SuspVelMulti = Clamp(abs(Wheels[i].SuspensionVelocity), 0.0, 1.0)
    #SuspVelMulti = abs(Wheels[i].SuspensionVelocity)
    if Wheels[i].Limit and Wheels[i].SuspensionVelocity > 0.0 and LengthC > 0.0:
        SpringStiffness1 = Clamp((SpringStiffness)*SuspVelMulti, 50000, 90000.0)
        Damper1 = Clamp((Damper)*SuspVelMulti, 8000, 10000.0)
    elif Wheels[i].Object.localPosition.z > Wheels[i].MaxUpLength and Wheels[i].SuspensionVelocity < 0.0:
        SpringStiffness1 = Clamp((SpringStiffness)*SuspVelMulti, 50000, 90000.0)
        Damper1 = Clamp((Damper)*SuspVelMulti, 8000, 10000.0)
        Wheels[i].Limit = True
    elif Wheels[i].Limit:
        Wheels[i].Limit = False"""

"""
		"LatStiffnessFactor" : 2.8,
        "LatShapeFactor" : 1.7,
        "LatPeakValue" : 1.97,
        "LatCurvatureFactor" : 0.0,
		"LatTireLoad" : 1.6
"""