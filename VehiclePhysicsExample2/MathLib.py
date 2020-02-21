from mathutils import Vector
from numpy import clip as Clamp
import math

RightVector = Vector([1, 0, 0])
ForwardVector = Vector([0, 1, 0])
UpVector = Vector([0, 0, 1])

def Sign(x):
    if x != 0:
        return abs(x)/x
    else:
        return 0

def MoveTowardsF(x, Target, Value):
    Signal = Sign(Target-x)
    if Signal > 0:
        return Clamp(x+(Value*Signal), -math.inf, Target)
    else:
        return Clamp(x+(Value*Signal), Target, math.inf)

def MapRange(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = 0.0
    try:
        valueScaled = float(value - leftMin) / float(leftSpan)
    except:
        valueScaled = 0.0
        

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def MapRangeClamped(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = 0.0
    try:
        valueScaled = float(Clamp(value, leftMin, leftMax) - leftMin) / float(leftSpan)
    except:
        valueScaled = 0.0
        

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def ClampVector(value, min, max):
    result = Vector([Clamp(value.x, min, min), Clamp(value.y, min, min), Clamp(value.z, min, min)])
    
    return result

def LerpF(v0, v1, t):
    return (1 - t) * v0 + t * v1

def GetValueRotated(value, other):
    result = value.copy()
    result.rotate(other)
    return result

def TransformDirection(Rotation, Direction):
    return Rotation * Direction

def GetAngleBetweenVectors(Vector1, Vector2):    
    Result = Vector2.dot(Vector1)
    
    return Result

def NearFloat(Value, Near, Range):
    return Value > Near-Range and Value < Near+Range


