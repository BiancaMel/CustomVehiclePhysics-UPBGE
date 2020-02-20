import bge
from collections import OrderedDict
import time
import mathutils
from mathutils import Vector
from numpy import clip as Clamp, interp as Lerp
from copy import deepcopy
import math
import MathLib
import sys


pc = bge.constraints
events = bge.events
render = bge.render
k = bge.logic.keyboard.inputs
scene = bge.logic.getCurrentScene()
logic = bge.logic


class Wheel():
    def __init__(self):
        #Constant Values
        self.Radius = 0.351867
        self.Mass = 50.0
        self.bIsTraction = True
        
        #Realtime Values
        self.LocalPosition = mathutils.Vector([0, 0, 0])
        self.LocalOrientation = mathutils.Matrix([[1.0000, 0.0000,  0.0000], [0.0000, 1.0000, 0.0000], [0.0000, 0.0000,  1.0000]])
        self.LocalWheelVelocity = mathutils.Vector([0, 0, 0])
        self.WorldWheelVelocity = mathutils.Vector([0, 0, 0])
        self.WheelAccel = mathutils.Vector([0, 0, 0])
        self.AngularVelocity = 0.0
        self.WheelRotation = 0.0
        self.CurrentTorque = 0.0
        self.Inertia = 1.0
        self.AngularAccel = 0.0
        
        #Temporary Values
        self.LastWorldPosition = mathutils.Vector([0, 0, 0])
        self.LastLocalVelocity = mathutils.Vector([0, 0, 0])
        self.LastWorldVelocity = mathutils.Vector([0, 0, 0])
        self.LastNormalX = mathutils.Vector([0, 0, 0])

    def UpdateRealtimeValues(self, WheelObject, VehicleObject, Ray, Time):
        self.LocalPosition = (Ray[1] - VehicleObject.worldPosition) * VehicleObject.worldOrientation + Vector([0, 0, self.Radius])
        self.LocalPosition.z = Clamp(self.LocalPosition.z, -50, WheelObject.Suspension.MaxRaise)
        self.WorldWheelVelocity = (Ray[1]-self.LastWorldPosition) / Time
        self.LocalWheelVelocity = self.WorldWheelVelocity * WheelObject.worldOrientation
        
        EngineTurnOverWheel = 0.0
        if self.bIsTraction and VehicleObject.Transmission.CurrentGear != 0:
            EngineTurnOverWheel = VehicleObject.EngineAngularVelocity
        
        if VehicleObject.Transmission.CurrentGear != 0:
            self.AngularVelocity = (6.28318530718 * EngineTurnOverWheel) / (60 * VehicleObject.Transmission.CurrentDriveRatio)#/ Time#self.LocalWheelVelocity.y / self.Radius
        else:
            self.AngularVelocity = 0.0#(6.28318530718 * EngineTurnOverWheel) / (60)
        
        self.AngularVelocity += self.AngularAccel
        self.WheelRotation = (self.LocalWheelVelocity.y / self.Radius)+self.AngularVelocity
        self.WheelAccel = VehicleObject.localLinearVelocity-VehicleObject.LastVelocity/Time#(self.LocalWheelVelocity-self.LastVelocity)#/Time

class WheelSteering():
    def __init__(self):
        #Constant Values
        self.bIsSteering = False
        self.MaxSteeringAngle = math.radians(45.0)
        
        #Realtime Values
        self.SteeringAngle = 0.0

    def UpdateSteeringAngle(self, WheelObject, VehicleObject, Steering, Sensibility, Time):
        SteeringCurve = 1.0-MathLib.MapRangeClamped(VehicleObject.SpeedKMH, 10.0, 150.0, 0.0, 0.85)
        #self.SteeringAngle = MathLib.MoveTowardsF(self.SteeringAngle, Steering * (self.MaxSteeringAngle * SteeringCurve), Sensibility*Time)
        self.SteeringAngle = MathLib.LerpF(self.SteeringAngle, Steering * (self.MaxSteeringAngle * SteeringCurve), Sensibility*Time)
        WheelRot = WheelObject.localOrientation.to_euler()
        WheelRot[2] = -self.SteeringAngle
        WheelObject.localOrientation = WheelRot.to_matrix()

class WheelTire():
    def __init__(self):
        #Constant Values
        self.LongStiffnessFactor = 0.06
        self.LongShapeFactor = 1.25
        self.LongPeakValue = 3.0
        self.LongCurvatureFactor = 0.0
        self.LongTireLoad = 10.0
        self.LongLowSpeedLoad = 0.1
        
        self.LatStiffnessFactor = 0.06
        self.LatShapeFactor = 1.25
        self.LatPeakValue = 3.0
        self.LatCurvatureFactor = 0.0
        self.LatTireLoad = 10.0
        self.LatLowSpeedLoad = 1.0
        
        self.LowSpeedValue = 0.5
        self.RollResistance = 0.6
        
        #Realtime Values
        self.CurrentLongSlip = 0.0
        self.CurrentLatSlip = 0.0
        self.TractionForce = 0.0
        self.LongForce = 0.0
        self.LateralForce = 0.0
        self.LongSlip = 0.0
        self.LateralSlipAngle = 0.0
        self.RollResistanceForce = 0.0
        self.Fy = 0.0
        self.Fx = 0.0
        self.mostra = 0.0
    
    def GetTireForces(self, WheelObject, VehicleObject, Time):
        WheelForwardVector = WheelObject.getAxisVect(MathLib.ForwardVector)
        WheelRightVector =  WheelObject.getAxisVect(MathLib.RightVector)
        WHEELPOS = WheelObject.worldPosition - WheelObject.getAxisVect(MathLib.UpVector) * WheelObject.Wheel.Radius
        
        
        SuspF = WheelObject.Suspension.CurrentForceZ
        
        Velocity2D = abs(WheelObject.Wheel.LocalWheelVelocity.x) + abs(WheelObject.Wheel.LocalWheelVelocity.y)
        
        if not MathLib.NearFloat(Velocity2D, 0.0, self.LowSpeedValue):
            self.LongSlip = ((WheelObject.Wheel.AngularVelocity * WheelObject.Wheel.Radius - WheelObject.Wheel.LocalWheelVelocity.y) / WheelObject.Wheel.LocalWheelVelocity.y)+1
            self.LateralSlipAngle = MathLib.GetAngleBetweenVectors(WheelObject.Wheel.WorldWheelVelocity.normalized(), WheelRightVector)*(1.0-Clamp(abs(self.LongSlip*1.5), 0.0, 0.9))

            self.Fy = -(self.LongPeakValue*math.sin(self.LongShapeFactor * math.atan(self.LongStiffnessFactor * self.LongSlip - self.LongCurvatureFactor*(self.LongStiffnessFactor*self.LongSlip-math.atan(self.LongStiffnessFactor*self.LongSlip)))))*SuspF/self.LongTireLoad# + SVy
            self.Fx = -(self.LatPeakValue*math.sin(self.LatShapeFactor * math.atan(self.LatStiffnessFactor * self.LateralSlipAngle - self.LatCurvatureFactor*(self.LatStiffnessFactor*self.LateralSlipAngle-math.atan(self.LatStiffnessFactor*self.LateralSlipAngle)))))*SuspF*self.LatTireLoad# + SVx
        else:
            self.LongSlip = 0.0
            self.LateralSlipAngle = 0.0
            self.Fy = Clamp(-WheelObject.Wheel.LocalWheelVelocity.y * SuspF, -SuspF, SuspF)*self.LongLowSpeedLoad
            self.Fx = Clamp(-WheelObject.Wheel.LocalWheelVelocity.x * SuspF, -SuspF, SuspF)*self.LatLowSpeedLoad
        
        
        self.RollResistanceForce = -self.RollResistance * WheelObject.Wheel.LocalWheelVelocity.y
        if WheelObject.Wheel.bIsTraction:
            if VehicleObject.Transmission.CurrentGear != 0:
                WheelObject.Wheel.CurrentTorque = VehicleObject.CurrentEngineTorque * VehicleObject.Transmission.CurrentDriveRatio
            else:
                WheelObject.Wheel.CurrentTorque = Clamp(MathLib.LerpF(WheelObject.Wheel.CurrentTorque, -0.1, Time*5), 0.0, 1000000.0)
            self.TractionForce = WheelObject.Wheel.CurrentTorque / WheelObject.Wheel.Radius
        else:
            self.TractionForce = 0.0
        
        TractionTorque = self.Fy * WheelObject.Wheel.Radius
        TotalTorque =  self.TractionForce + TractionTorque + (VehicleObject.CurrentBrakeTorque * -MathLib.Sign(WheelObject.Wheel.LocalWheelVelocity.y))
        WheelObject.Wheel.AngularAccel = (TotalTorque / WheelObject.Wheel.Inertia) * Time
        #WheelObject.Wheel.AngularVelocity += AngularAccel * Time
        #print(AngularAccel)
        self.mostra = (1.0-Clamp(abs(self.LongSlip*1.0), 0.0, 1.0))
        self.LongForce = WheelForwardVector * (TotalTorque + self.RollResistanceForce + self.Fy)
        self.LateralForce = WheelRightVector * self.Fx#*(1.0-Clamp(abs(self.LongSlip*1.05), 0.0, 1.0))#(1.0-Clamp(abs(self.LongSlip*1.0), 0.0, 1.0))
        
        #render.drawLine(WHEELPOS, WHEELPOS+WheelForwardVector*1.0,[1,0.0,1,1])
        #render.drawLine(WHEELPOS, WHEELPOS+WheelObject.Wheel.WorldWheelVelocity.normalized()*1.0,[1,0.99,0,1])
        
        return self.LongForce, self.LateralForce

class WheelSuspension():
    def __init__(self):
        #Constant Values
        self.Height = 0.75
        self.SpringStiffness = 45000.0
        self.DamperStiffness = 1500.0
        self.MaxRaise = 0.12
        self.MaxDrop = 1.0
        self.ForceAppHeight = 0.15
        self.ForceAppHeightLateral = 0.15
        
        #Realtime Values
        self.CurrentForceZ = 0.0
        self.CurrentLength = 0.0
        self.CurrentVelocity = 0.0
        self.StartPoint = Vector([0.0, 0.0, 0.0])
        
        #Temporary Values
        self.LastLength = 0.0

    def GetSuspensionForce(self, WheelObject, Ray, Time):
        self.CurrentLength = (self.StartPoint-Ray[1]).length
        self.CurrentVelocity = (self.CurrentLength-self.LastLength)/Time
        
        LengthClamped = Clamp(self.Height-self.CurrentLength, 0.0, 10000.0)
        CurrentSpringStiffness = self.SpringStiffness
        CurrentDamper = self.DamperStiffness
        self.CurrentForceZ = Clamp(CurrentSpringStiffness * LengthClamped - CurrentDamper * self.CurrentVelocity, -1000.0, 100000.0)
        
        #Beta
        DotUpVect = Ray[2].dot(MathLib.UpVector)
        SideWheelRate = 10.0
        SideMinWheelRate = 0.15
        
        return (Ray[2].lerp(MathLib.UpVector, Clamp((DotUpVect+SideMinWheelRate)**SideWheelRate, 0.0, 1.0)))*self.CurrentForceZ
        #return (Ray[2].lerp(UpVector, Clamp((DotUpVect+SideMinWheelRate)**SideWheelRate, 0.0, 1.0)))*self.CurrentForceZ
        #return (UpVector)*self.CurrentForceZ

class VehicleWheel(bge.types.KX_GameObject):    
    def __init__(self, oldOwner, VehicleObject, Setup=None):
        self.Enable = True
        self.WheelId = 0
        #print(self.WheelId)
        
        # Vehicle Setup
        self.VehicleObject = VehicleObject
        self.Wheel = Wheel()
        self.Suspension = WheelSuspension()
        self.Tire = WheelTire()
        self.Steering = WheelSteering()
        #print("Setup: ", Setup)
        
        if Setup != None:
           self.SetupWheelValues(Setup)
        
    
    def SetupWheelValues(self, SetupValues):
        #Set Wheel Values
        #self.Wheel.WheelId = SetupValues['Wheel Id']
        self.Wheel.Radius = SetupValues['Wheel']['Radius']
        self.Wheel.Mass = SetupValues['Wheel']['Mass']
        self.Wheel.Inertia = self.Wheel.Mass * (self.Wheel.Radius)
        #self.Wheel.Inertia = self.Wheel.Mass * (self.Wheel.Radius * self.Wheel.Radius) /2
        self.Wheel.bIsTraction = SetupValues['Wheel']['bIsTraction']

        #Set Suspension Values
        self.Suspension.Height = SetupValues['Suspension']['Height']
        self.Suspension.MaxRaise = SetupValues['Suspension']['MaxRaise']
        self.Suspension.MaxDrop = SetupValues['Suspension']['MaxDrop']
        self.Suspension.SpringStiffness = SetupValues['Suspension']['SpringStiffness']*(self.VehicleObject.mass*0.001)
        self.Suspension.DamperStiffness = SetupValues['Suspension']['DamperStiffness']*(self.VehicleObject.mass*0.001)
        self.Suspension.ForceAppHeight = SetupValues['Suspension']['ForceAppHeight']
        self.Suspension.ForceAppHeightLateral = SetupValues['Suspension']['ForceAppHeightLateral']
        
        #Set Tire Values
        self.Tire.LongStiffnessFactor = SetupValues['Tire']['LongStiffnessFactor']
        self.Tire.LongShapeFactor = SetupValues['Tire']['LongShapeFactor']
        self.Tire.LongPeakValue = SetupValues['Tire']['LongPeakValue']
        self.Tire.LongCurvatureFactor = SetupValues['Tire']['LongCurvatureFactor']
        self.Tire.LongTireLoad = SetupValues['Tire']['LongTireLoad']
        
        self.Tire.LatStiffnessFactor = SetupValues['Tire']['LatStiffnessFactor']
        self.Tire.LatShapeFactor = SetupValues['Tire']['LatShapeFactor']
        self.Tire.LatPeakValue = SetupValues['Tire']['LatPeakValue']
        self.Tire.LatCurvatureFactor = SetupValues['Tire']['LatCurvatureFactor']
        self.Tire.LatTireLoad = SetupValues['Tire']['LatTireLoad']
        
        #Set Steering Values
        self.Steering.bIsSteering = SetupValues['Steering']['bIsSteering']
        self.Steering.MaxSteeringAngle = math.radians(SetupValues['Steering']['MaxSteeringAngle'])













