if __name__ != "__main__":
    from appFiles.states import State, ChangingState
else:
    from states import State, ChangingState
import numpy as np
import math


class ProjectileMotion:
    # Constants
    GRAV = 9.81 
    AIR_DENSITY = 1.225

    def __init__(self, dragCoefficient = 0.5, projectileRad = 0.0215, projectileMass = 0.004, targetRadius = 0.1,
                        launchDegrees = 45, timeStep = 0.01, initialVelocity = 20.0, maxAngle = 70):
            #Custom Set variables
            #Private controlled variables
            self._dragCoef = dragCoefficient
            self._projR = projectileRad
            self._projM = projectileMass
            self._launchDeg = launchDegrees
            self._v0 = initialVelocity
            self._maxAngle = maxAngle
            self._targetRadius = targetRadius
            #public uncontrolled variables
            self._targetDistance = None
            self._targetAngle = None
            self._targetCoords = None
            #Public controlled variables
            self.timeStep = timeStep
            #Calculated variables
            self.__calcDragConst()
            self.__calcVxy0()

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
        self.__calcVxy0()

    @property
    def initialVelocity(self):
        return self._v0
    @initialVelocity.setter
    def initialVelocity(self, vel):
        self._v0 = vel
        #recalc velocity components on change to launch velocity
        self.__calcVxy0()

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
    def __calcVxy0(self):
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
        while (dragMotion[-1][1] >= -1) and t < tLim:
            t += self.timeStep
            dragState = self.__rk4Step(dragState)
            dragMotion.append(dragState.displacement)
            tArray.append(t)
        #return motion
        dragMotion = np.array(dragMotion)
        tArray = np.array(tArray)
        return dragMotion, tArray
    
    #Get x,y coordinates of a trajectory not compensating for drag, (set parameters using properties)
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
        while (noDragMotion[-1][1] >= -1) and t < tLim:
            t += self.timeStep
            noDragState = self.__nonDragStep(noDragState)
            noDragMotion.append(noDragState.displacement)
            tArray.append(t)
        #return motion
        noDragMotion = np.array(noDragMotion)
        tArray = np.array(tArray)
        return noDragMotion, tArray
    
    #calculate angle required for projectile target collision using simple projectile motion
    def __calcNoDragAngle(self):
        #Use quadratic equation to solve for angle
        a = self.GRAV * self._targetCoords[0]**2
        b = -2 * self._v0**2 * self._targetCoords[0]
        c = self.GRAV * self._targetCoords[0]**2 + 2*self._v0**2 * self._targetCoords[1]
        #check descriminate for solution
        descriminate = b**2 - 4*a*c
        if descriminate < 0:
            #ret None if no solutions
            return None
        theta1 = math.degrees(math.atan((-b + math.sqrt(descriminate)) / (2*a)))
        theta2 = math.degrees(math.atan((-b - math.sqrt(descriminate)) / (2*a)))
        #return lower angle solution
        return theta1, theta2

    #Get min error (+error = overshoot, negative = undershoot)
    def __getTrajectoryMinError(self, xyArray, xyTarget):
            xArray, yArray = xyArray[:,0], xyArray[:,1]
            #get closest position relative to target
            errorDists = np.sqrt((xArray - xyTarget[0])**2 + (yArray - xyTarget[1])**2)
            minErrIndx = np.argmin(errorDists)
            if (xyArray[minErrIndx][1] > xyTarget[1]):
                error = errorDists[minErrIndx]
            else:
                error = errorDists[minErrIndx] * (-1)
            return error

    #Calculate angle required for projectile target collision compensating for drag effects
    def __calcDragAngle(self, maxIterations = 20):
        #get drag motion over a range of launch angles until an over an underestimate is provided, then apply bisection search
        lastAngle = 0
        success = False
        for angle in range(0, self._maxAngle + 1, 10):
            self.launchDegrees = angle
            motion = self.getDragMotion()[0]
            error = self.__getTrajectoryMinError(motion, self._targetCoords)
            if error > 0:
                success = True
                break
            else:
                lastAngle = angle
        #report None if not possible
        if not success:
            return None
        #apply bisector search between found over / under estimate
        lowAngle = lastAngle
        highAngle = angle
        for _ in range(maxIterations):
            angle = (lowAngle + highAngle)/2
            self.launchDegrees = angle
            motion = self.getDragMotion()[0]
            error = self.__getTrajectoryMinError(motion, self._targetCoords)
            #if error mag is within target radius, success
            if abs(error) < self._targetRadius:
                return angle
            else:
                if error > 0:
                    highAngle = angle
                else:
                    lowAngle = angle
        #No solution, ret None
        print("NO BISECTION SOLUTION")
        return None

    #Calculate angle required to hit target with current parameters at given distance (toggle drag)
    def calculateAngle(self, targetDistance, targetAngle, drag = True):
        self._targetDistance = targetDistance
        self._targetAngle = targetAngle
        self._targetCoords = self.getTargetCoords(targetDistance, targetAngle)
        if drag:
            return self.__calcDragAngle()
        else:
            return self.__calcNoDragAngle()

    #Get target coordinates from passed distance and angle
    def getTargetCoords(self, targetDistance, targetAngle):
        return targetDistance * np.array([math.cos(math.radians(targetAngle)), math.sin(math.radians(targetAngle))])  

    #Plot figure using matplotlib
    def plotMPL(self, xyResults, show = True, newFig = False, markType = 'o', targetAngle = None, targetDistance = None):
        #Clear figure if new
        if newFig:
            plt.clf()
        #Split results into x and y arrays
        xd, xy = xyResults[:,0], xyResults[:,1]
        #plot figures
        plt.plot(xd, xy, marker= markType)     # line + optional markers
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('Object trajectory')
        plt.axis('equal')              # equal aspect ratio so 1 m in x = 1 m in y
        plt.grid(True)
        if targetAngle is not None and targetDistance is not None:
            targetX, targetY = self.getTargetCoords(targetDistance, targetAngle)
            plt.scatter(targetX, targetY, color='green', label='Target', s=300, marker = 'x',  zorder=5)
        if show == True:
            plt.show()
        

#example
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    #create projectile motion calculator using defaults
    motioncal = ProjectileMotion()

    #PLOT CURRENT TRAJECTORY
    def currentTrajectoryExample():
        # can change any properties if needed:
        motioncal.launchDegrees = 50
        #get and plot results
        motioncal.plotMPL(motioncal.getDragMotion()[0]) #[0] = displacement, [1] = time steps
        motioncal.plotMPL(motioncal.getNoDragMotion()[0])

    #FIND and PLOT A TRAJECTORY TO HIT A TARGET
    def noDragExample():
        targetX, targetY = motioncal.getTargetCoords(3, 40)
        theta1, theta2 = motioncal.calculateAngle(3,40, False)
        motioncal.launchDegrees = theta1
        motioncal.plotMPL(motioncal.getNoDragMotion()[0], False)
        plt.scatter(targetX, targetY, color='green', label='Target', s=300, edgecolors='black', marker = 'x',  zorder=5)
        print(f"Angles: {theta1}, {theta2}")
        motioncal.launchDegrees = theta2
        motioncal.plotMPL(motioncal.getNoDragMotion()[0])

    #Find and plot trajectory to hit target compensating for drag
    def dragExample():
        theta = motioncal.calculateAngle(2,20,True)
        print(f"DRAG ANGLE: {theta}")
        motioncal.launchDegrees = theta
        motioncal.plotMPL(motioncal.getNoDragMotion()[0], True, targetDistance = 2, targetAngle = 20)

    dragExample()


