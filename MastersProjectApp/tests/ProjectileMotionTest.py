import numpy as np
import matplotlib.pyplot as plt

# Constants
grav = 9.81  # gravity
airDensity = 1.225
dragCoefficient = 0.5
projectileRad = 0.0215
projectileA = projectileRad * projectileRad * 3.141
projectileMass = 0.004 #standard are 0.004, golf style are 0.008
dragConst = dragCoefficient * airDensity * projectileA * 0.5

timeStep = 0.01 #step increments in seconds

v0 = 20.0  # initial speed in m/s
target_distance = 3.0  # meters


def calcDragAcc(Vx, Vy):
    Vmag = np.sqrt(Vx * Vx + Vy * Vy)
    Vx2 = Vx * Vmag
    Vy2 = Vy * Vmag
    ax = Vx2*dragConst/projectileMass
    ay = Vy2*dragConst/projectileMass

def getStateOfChange(Sx, Sy, Vx, Vy):
    #Calculate drag
    global grav
    ax, ay = calcDragAcc(Vx, Vy)
    ay = grav - ay
    ax = -ax
    #velocity = displacement rate of change
    #return stateOfChange
    return Vx, Vy, ax, ay

def getChange(Vx, Vy, ax, ay, deltaT):
    #Get change in displacement over deltaT
    deltaSx = Vx * deltaT
    deltaSy = Vy * deltaT
    #get change in velocity over deltaT
    deltaAx = ax * deltaT
    deltaAy = ay * deltaT
    #ret
    return deltaSx, deltaSy, deltaAx, deltaAy

def rk4Step(Sx, Sy, Vx, Vy, dt):
    k1State = [Sx, Sy, Vx, Vy]
    k1 = getStateOfChange(k1State)
    k1Change = getChange(k1, dt)
    k2State = k1State + k1Change
    K2 = getStateOfChange(k2State)

    #Merge getStateOfChange & getChange functions into one
