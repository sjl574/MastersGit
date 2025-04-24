from states import State, ChangingState
import numpy as np
import math


class ProjectileMotion:
    # Constants
    GRAV = 9.81 
    AIR_DENSITY = 1.225

    def __init__(self, dragCoefficient = 0.5, projectileRad = 0.0215, projectileMass = 0.004, 
                        targetDistance = 3.0, launchDegrees = 45, timeStep = 0.01, initialVelocity = 20.0):
            #Custom Set variables
            #Private controlled variables
            self._dragCoef = dragCoefficient
            self._projR = projectileRad
            self._projM = projectileMass
            self._launchDeg = launchDegrees
            self._v0 = initialVelocity
            #public uncontrolled variables
            self.targetDistance = targetDistance
            self.timeStep = timeStep
            #Calculated variables
            self.__calcDragConst()
            self.__calcV0()

    #------------Privately controlled equation consts
    @property
    def dragCoefficient(self):
        return self._dragCoef
    @dragCoefficient.setter
    def dragCoefficient(self, dragCo):
        self._dragCoef = dragCo
        #Recalculate drag const on changes to drag coefficient
        self.__calcDragConst()

    @property
    def projectileRad(self):
        return self._projR
    @projectileRad.setter
    def projectileRad(self, projR):
        self._projR = projR
        #recalc drag const on change to projectile radius
        self.__calcDragConst()

    @property
    def launchDegrees(self):
        return self._launchDeg
    @launchDegrees.setter
    def launchDegrees(self, degrees):
        self._launchDeg = degrees
        #recalc velocity components on change to launch angle
        self.__calcV0()

    @property
    def initialVelocity(self):
        return self._v0
    @initialVelocity.setter
    def initialVelocity(self, vel):
        self._v0 = vel
        #recalc velocity components on change to launch velocity
        self.__calcV0()

    @property
    def projectileMass(self):
        return self._projM
    @projectileMass.setter
    def projectileMass(self, mass):
        self._projM = mass
    
    #----------------------------calculated equation consts
    #calculate drag constant
    def __calcDragConst(self):
        self._projA = self._projR * self._projR * 3.141
        self._dragConst = self._dragCoef * self.AIR_DENSITY * self._projA * 0.5

    #Calculate velocity x and y constituents from magnitude and launch angle
    def __calcV0(self):
        self._launchRad = math.radians(self._launchDeg)
        self._vx0 = self._v0 * math.cos(self._launchRad)
        self._vy0 = self._v0 * math.sin(self._launchRad)

    #--------------------------calculate equation variables
    #Calculate acceleration due to drag in x and y directions
    def __calcDragAcc(self, vel):
        Vx, Vy = vel
        Vmag = np.sqrt(Vx * Vx + Vy * Vy)
        Vx2 = Vx * Vmag
        Vy2 = Vy * Vmag
        ax = (-1) * Vx2*self._dragConst/self._projM
        ay = (-1) * Vy2*self._dragConst/self._projM
        return np.array([ax, ay])

    #Rouge Kutta 4 slope calculation
    def __rk4Slope(self, initState : State):
        changeState = ChangingState()
        changeState.velocity = initState.velocity
        changeState.acceleration = np.add(np.array([0, -self.GRAV]), self.__calcDragAcc(initState.velocity))
        return changeState

    #Rouge Kutta 4 Method
    def __rk4Step(self, initState : State) -> State:
        #Calculate Rouge Kutta slopes
        k1 = self.__rk4Slope(initState)
        k2 = self.__rk4Slope(initState + k1.toState(self.timeStep/2))
        k3 = self.__rk4Slope(initState + k2.toState(self.timeStep/2))
        k4 = self.__rk4Slope(initState + k3.toState(self.timeStep))
        #Calculate wieghted average of change / slopes
        k = (1/6) * (k1 + 2*k2 + 2*k3 + k4)
        #Add change over dt to initial state to provide state at end of dt
        return initState + k.toState(self.timeStep)

    #Basic euler is fine as only first order system
    def __nonDragStep(self, initState : State) -> State:
        changeState = ChangingState()
        changeState.velocity = initState.velocity
        changeState.acceleration = [0,-self.GRAV]    #constant accel
        #Add change over dt to initial state
        return initState + changeState.toState(self.timeStep) 

    #Get array of motion displacement over projectile path (xyArray, timeArray)
    def getDragMotion(self, tLim = 10):
        #Create inital state
        dragState = State()
        dragState.displacement = 0,0
        dragState.velocity = self._vx0, self._vy0
        t = 0
        #Create array to store results
        dragMotion = [dragState.displacement]
        tArray = [t]
        #iterate across time intervals
        while (dragMotion[-1][1] >= 0) and t < tLim:
            t += self.timeStep
            dragState = self.__rk4Step(dragState)
            dragMotion.append(dragState.displacement)
            tArray.append(t)
        #return motion
        dragMotion = np.array(dragMotion)
        tArray = np.array(tArray)
        return dragMotion, tArray
    
    def getNoDragMotion(self, tLim = 10):
        #Create inital state
        noDragState = State()
        noDragState.displacement = 0,0
        noDragState.velocity = self._vx0, self._vy0
        t = 0
        #Create array to store results
        noDragMotion = [noDragState.displacement]
        tArray = [t]
        #iterate across time intervals
        while (noDragMotion[-1][1] >= 0) and t < tLim:
            t += self.timeStep
            noDragState = self.__nonDragStep(noDragState)
            noDragMotion.append(noDragState.displacement)
            tArray.append(t)
        #return motion
        noDragMotion = np.array(noDragMotion)
        tArray = np.array(tArray)
        return noDragMotion, tArray

#example
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    #create projectile motion calculator using defaults
    motioncal = ProjectileMotion()
    #can change any properties if needed:
    motioncal.launchDegrees = 50
    #get set of results
    dragPositions = motioncal.getDragMotion()[0]    #[0] = displacement, [1] = time steps
    noDragPositions = motioncal.getNoDragMotion()[0]

    xd, xy = dragPositions[:,0], dragPositions[:,1]
    xnd, ynd = noDragPositions[:,0], noDragPositions[:,1]

    plt.plot(xd, xy, marker='o')     # line + optional markers
    plt.plot(xnd, ynd, marker='x')     # line + optional markers
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Object trajectory')
    plt.axis('equal')              # equal aspect ratio so 1 m in x = 1 m in y
    plt.grid(True)
    plt.show()