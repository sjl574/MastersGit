import numpy as np
import matplotlib.pyplot as plt
import math

# Constants
GRAV = 9.81  # gravity
airDensity = 1.225
dragCoefficient = 0.5
projectileRad = 0.0215
projectileA = projectileRad * projectileRad * 3.141
projectileMass = 0.004 #standard are 0.004, golf style are 0.008
dragConst = dragCoefficient * airDensity * projectileA * 0.5
initX = 0
initY = 0
timeStep = 0.01 #step increments in seconds
v0 = 20.0  # initial speed in m/s

#Variables
target_distance = 3.0  # meters
launchAngle = 45

class State:
    def __init__(self, sx = None, sy = None, vx = None, vy = None):
        self.sx = sx
        self.sy = sy
        self.vx = vx
        self.vy = vy

    @property
    def displacement(self):
        return np.array([self.sx, self.sy])
    @displacement.setter
    def displacement(self, *args):
        args = args[0]
        if len(args) == 1:
            self.sx = args[0][0]
            self.sy = args[0][1]    
        if len(args) == 2:
            self.sx = args[0]
            self.sy = args[1] 

    @property
    def velocity(self):
        return np.array([self.vx, self.vy])
    @velocity.setter
    def velocity(self, *args):
        args = args[0]
        if len(args) == 1:
            self.vx = args[0][0]
            self.vy = args[0][1]    
        if len(args) == 2:
            self.vx = args[0]
            self.vy = args[1] 

    def as_np(self):
        return np.array([self.sx, self.sy, self.vx, self.vy])

    @classmethod
    def from_np(self, array):
        inst = State()
        inst.sx, inst.sy, inst.vx, inst.vy = array
        return inst

    def __add__(thisInst, otherInst):
        if type(thisInst) == type(otherInst):
            return State(thisInst.sx + otherInst.sx,
                         thisInst.sy + otherInst.sy,
                         thisInst.vx + otherInst.vx,
                         thisInst.vy + otherInst.vy
                        )

    def __str__(self):
        return str(self.as_np())
    

class ChangingState:
    def __init__(self, vx = None, vy = None, ax = None, ay = None):
        self.vx = vx
        self.vy = vy
        self.ax = ax
        self.ay = ay

    @property
    def velocity(self):
        return np.array([self.vx, self.vy])
    @velocity.setter
    def velocity(self, *args):
        args = args[0]
        if len(args) == 1:
            self.vx = args[0][0]
            self.vy = args[0][1]    
        if len(args) == 2:
            self.vx = args[0]
            self.vy = args[1] 

    @property
    def acceleration(self):
        return np.array([self.ax, self.ay])
    @acceleration.setter
    def acceleration(self, *args):
        args = args[0]
        if len(args) == 1:
            self.ax = args[0][0]
            self.ay = args[0][1]    
        if len(args) == 2:
            self.ax = args[0]
            self.ay = args[1] 

    def as_np(self):
        return np.array([self.vx, self.vy, self.ax, self.ay])

    @classmethod
    def from_np(self, array):
        inst = ChangingState()
        inst.vx, inst.vy, inst.ax, inst.ay = array
        return inst
    
    def toState(self, timeInteval) -> State:
        return State.from_np(self.as_np() * timeInteval)
        
    def __mul__(self, val):
        return ChangingState.from_np(self.as_np() * val)

    def __rmul__(self, val):
        return ChangingState.from_np(self.as_np() * val)

    def __add__(thisInst, otherInst):
        if type(thisInst) == type(otherInst):
            return ChangingState(   thisInst.vx + otherInst.vx,
                                    thisInst.vy + otherInst.vy,
                                    thisInst.ax + otherInst.ax,
                                    thisInst.ay + otherInst.ay
                                )

    def __str__(self):
        return str(self.as_np())
    

def calcV(Vmag, angleDeg):
    angleRad = math.radians(angleDeg)
    Vx = Vmag * math.cos(angleRad)
    Vy = Vmag * math.sin(angleRad)
    return np.array([Vx, Vy])

def calcDragAcc(vel):
    Vx, Vy = vel
    Vmag = np.sqrt(Vx * Vx + Vy * Vy)
    Vx2 = Vx * Vmag
    Vy2 = Vy * Vmag
    ax = (-1) * Vx2*dragConst/projectileMass
    ay = (-1) * Vy2*dragConst/projectileMass
    return np.array([ax, ay])


def rk4Slope(initState : State):
    changeState = ChangingState()
    changeState.velocity = initState.velocity
    changeState.acceleration = np.add(np.array([0, -GRAV]), calcDragAcc(initState.velocity))
    return changeState

def rk4Step(initState : State, dt):
    k1 = rk4Slope(initState)
    k2 = rk4Slope(initState + k1.toState(dt/2))
    k3 = rk4Slope(initState + k2.toState(dt/2))
    k4 = rk4Slope(initState + k3.toState(dt))

    k = (1/6) * (k1 + 2*k2 + 2*k3 + k4)
    return initState + k.toState(dt)

#example
#create intial state
state = State()
state.displacement = initX, initY
state.velocity = calcV(v0, launchAngle)

timestep = 0.01
t = 0
sArray = [state.displacement]
while (sArray[-1][1] >= 0) and t < 10:
    t += timeStep
    state = rk4Step(state, timestep)
    sArray.append(state.displacement)

sArray = np.array(sArray)
x, y = sArray[:,0], sArray[:,1]

plt.plot(x, y, marker='o')     # line + optional markers
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('Object trajectory')
plt.axis('equal')              # equal aspect ratio so 1 m in x = 1 m in y
plt.grid(True)
plt.show()