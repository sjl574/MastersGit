import numpy as np

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
    

