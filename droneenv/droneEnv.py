#./vrep.sh -gREMOTEAPISERVERSERVICE_19999_TRUE_TRUE ../ejemplos/GYM/oneDrone.ttt
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
import vrep
import math,time

def distance(a,b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

class DroneEnv(gym.Env):
    quadPosition = [0,0,0]
    quadricopterTargetPosition  = [0,0,0]
    quadOrientation = [0.0,0,0]
    stepTime = 0.5
    def __init__(self):
        high = np.array([
            51,51,51,
            10,
            10,
            10,
            51,
            51,
            51,
            51,
            51,
            51])

        self.action_space = spaces.Discrete(9) #0 do nothing, 1 acelerate rotor 1, 2 acelerate rotor 2... 5 decelerate rotor 1, 6 decelerate rotor 2...
        self.observation_space = spaces.Box(-1 * high, high) # Position and speed of drone & objetive position
        self.state = None
        self.particleVelocities = [4,]
        self.throttle = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.incr = 0.001
        #Connect with simulator
        self.clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
        if self.clientID!=-1:
            logger.warn ('Connected to remote API server')
        else:
            logger.error('Not connected')
        status,self.quadricopter = vrep.simxGetObjectHandle(self.clientID,"Quadricopter",vrep.simx_opmode_blocking)
        status,self.quadricopterTarget = vrep.simxGetObjectHandle(self.clientID,"Quadricopter_target",vrep.simx_opmode_blocking)

    def getDistance(self):
        return distance(self.getQuadricopterPosition(),self.getQuadricopterTargetPosition())

    def getDistanceXYZ(self):
        q = self.getQuadricopterPosition()
        t = self.getQuadricopterTargetPosition()
        return [q[0] - t[0], q[1] - t[1], q[2] - t[2]]

    def getQuadricopterPosition(self):
        status,self.quadPosition = vrep.simxGetObjectPosition(self.clientID,self.quadricopter,-1,vrep.simx_opmode_blocking)
        return self.quadPosition

    def getQuadricopterTargetPosition(self):
        status,self.quadricopterTargetPosition = vrep.simxGetObjectPosition(self.clientID,self.quadricopterTarget,-1,vrep.simx_opmode_blocking)
        return self.quadricopterTargetPosition

    def getQuadricopterOrientation(self):
        status,self.quadOrientation = vrep.simxGetObjectOrientation(self.clientID,self.quadricopter,-1,vrep.simx_opmode_blocking)
        #print("Orientation ", quadOrientation)
        return self.quadOrientation

    def getStatus(self):
#        return self.orientation + [self.throttle, self.yaw, self.pitch, self.roll] + [0,0,0] + [0,0,0] + [0,0,0]
        return self.getDistanceXYZ() + self.quadOrientation + self.quadPosition + self.quadricopterTargetPosition

    def getParticleVelocity(self, propeler):
        #print ("Getting velocity for propeler ", propeler)
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID,
                                                                               'Quadricopter_propeller_respondable'
                                                                               + `propeler`,
                                                                               vrep.sim_scripttype_childscript,
                                                                               'getParticleVelocity',
                                                                               [], [], [], emptyBuff,
                                                                               vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            return retFloats[0]
        else:
            print (res)
            return "None"

    def setParticleVelocity(self, propeler, newVel):
        if newVel < 0:
            return 1
        #print("Set propeler #", propeler, " to ", newVel)
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(self.clientID,
                                                                                 'Quadricopter_propeller_respondable' +
                                                                                 `propeler`,
                                                                                 vrep.sim_scripttype_childscript,
                                                                                 'setParticleVelocity',
                                                                                 [], [newVel], [], emptyBuff,
                                                                                 vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            return 1
        else:
            return None

    def stabilize(self):
        emptyBuff = bytearray()
        res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(self.clientID,
                                                                               'Quadricopter',
                                                                               vrep.sim_scripttype_childscript,
                                                                               'initStabilize',
                                                                               [], [], [], emptyBuff,
                                                                               vrep.simx_opmode_oneshot)
        return None

    def setVelocities(self):
        #Throttle
        velocities = np.array([self.throttle] * 4)
        #roll
        velocities[0] += self.roll
        velocities[1] += self.roll
        velocities[2] -= self.roll
        velocities[3] -= self.roll
        #pitch
        velocities[0] += self.pitch
        velocities[3] += self.pitch
        velocities[1] -= self.pitch
        velocities[2] -= self.pitch
        #yaw
        velocities[0] -= self.yaw
        velocities[2] -= self.yaw
        velocities[1] += self.yaw
        velocities[3] += self.yaw
        #set velocities
        emptyBuff = bytearray()
        #print(velocities)
        res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(self.clientID,
                                                                               'Quadricopter',
                                                                               vrep.sim_scripttype_childscript,
                                                                               'setVelocities',
                                                                               [], velocities, [], emptyBuff,
                                                                               vrep.simx_opmode_blocking)
        #print("res", res)
        #print("retInts", retInts)
        #print("retFloats", retFloats)
        #print("retStrings", retStrings)                                                                               
        #print("valores devueltos", retFloats)
        try:
            self.quadPosition = [retFloats[0],retFloats[1],retFloats[2]]
            self.quadOrientation = [retFloats[3],retFloats[4],retFloats[5]]
        
        except:
            print "Chungui"

        
        return None


    def reset(self):
        #print("Reset")
        vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_blocking)
        time.sleep(0.1)
        vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_blocking)
        vrep.simxPauseSimulation(self.clientID, vrep.simx_opmode_blocking)
        self.vIni = 5.33
        self.throttle = self.vIni
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.setVelocities()
        self.stabilize()
        #print self.quadPosition
        self.distance = self.getDistance()

        #print("Distance ", self.distance)
        #print ("Status ", np.array(self.getStatus()))
        return np.array(self.getStatus())

    def render(self, mode):
        return None

    def step(self, action):
        reward = 0
        done = False
        #print ("Action: ", action)
        #print time.time()
        if action == 0:
            self.stabilize()
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
            time.sleep(self.stepTime * 2)
            vrep.simxPauseSimulation(self.clientID, vrep.simx_opmode_blocking)    

            #time.sleep(0.05)
            #self.setVelocities()
            #return np.array(self.getStatus()), reward, done, {}
        elif action == 1:
            self.throttle += self.incr
            if (self.throttle > 6):
                self.throttle = 6
        elif action == 2:
            self.throttle -= self.incr
            if self.throttle < 0:
                self.throttle = 0
        elif action == 3:
            self.yaw += self.incr
 #           if self.yaw > 1:
 #               self.yaw = 1
        elif action == 4:
            self.yaw += -self.incr
 #           if self.yaw < -1:
 #               self.yaw = -1
        elif action == 5:
            self.roll += self.incr
#            if self.roll > 1:
#                self.roll = 1
        elif action == 6:
            self.roll += -self.incr
#            if self.roll < -1:
#                self.roll = -1
        elif action == 7:
            self.pitch += self.incr
 #           if self.pitch > 1:
 #               self.pitch = 1
        elif action == 8:
            self.pitch += -self.incr
#            if self.pitch < -1:
#                self.pitch = -1
        self.setVelocities()
        self.yaw = 0
        self.pitch = 0
        #self.roll = 0

        newDistance = self.getDistance()
        #print("Distance->", newDistance)
        if newDistance >= self.distance:
            reward = -1
        elif newDistance < self.distance:
            reward = 1
        if newDistance > 50:
            reward = -1000
            done = True
            self.stop()
        elif newDistance < 1:
            reward = 10
        self.distance = newDistance
        self.rotation = eulerAnglesToRotationMatrix(self.getQuadricopterOrientation())
        #print("Rotation->", rotation)
        #print("Orientation->", orientation)
        if (self.rotation[2][2] < -0.1):
            reward = -1000
            done = True
            self.stop()
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        time.sleep(self.stepTime)
        vrep.simxPauseSimulation(self.clientID, vrep.simx_opmode_blocking)    
        return np.array(self.getStatus()), reward, done, {}

    def stop(self):
        vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_blocking)

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta):

    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])



    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])


    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R
