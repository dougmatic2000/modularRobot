""" This class contains various methods for high level control of omni droid robot
"""

# from cgi import print_environ
from operator import truediv
from threading import Thread
import logging
import signal
import sys
import time
import numpy as np
import os
import copy
import serial
import struct
import matplotlib.pyplot as plt
# import matplotlib.animation as animation
from DroidInertialTF import droidInertial



# Log file location
logfile = 'debugDC.log'

# Define your own logger name
logger = logging.getLogger("droidControlLog")
# Set default logging level to DEBUG
logger.setLevel(logging.DEBUG)

# create console handler
print_format = logging.Formatter('[%(levelname)s] (%(threadName)-9s) %(message)s')
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.DEBUG)
console_handler.setFormatter(print_format)

# create log file handler
# and define a custom log format, set its log level to DEBUG
log_format = logging.Formatter('[%(asctime)s] %(levelname)-8s %(name)-12s %(message)s')
file_handler = logging.FileHandler(logfile)
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(log_format)

#Add handlers to the logger
logger.addHandler(file_handler)
logger.addHandler(console_handler)


shutdown_flag = False


class droidControl:

    def __init__(self, serial_port='/dev/ttyUSB0',baud=38400,inVarNum=10,outVarNum=7,VarType = 'f'):

        # Initialise output filename and directory
        self.path = []
        self.saveStateFileName = 'stateData.csv'
        # TODO: setup path and incremental filename suffix generator dd mm yy run N
        
        # Flags for thread operation
        self.isReceiving = False
        self.isRun = True
        self.thread = None
        
        # Loop control states
        self.droidMoving = False
        self.remoteWaiting = True

        # Setup variable conversion parameters
        self.VarType = VarType

        if (self.VarType == 'f'):
            self.VarBytes = 4
            logger.debug("Python expecting 4-byte FLOAT type input")
        elif (self.VarType == 'i'):
            self.VarBytes = 2 
            logger.debug("Python expecting 2-byte INT type input")         
        else:
            logger.error("Invalid Input data type, must be 'f' or 'i'")
            sys.exit()
        
        # Initialise OUTPUT test parameters 
        self.outVarNum = outVarNum
        self.dataOut = [0]*self.outVarNum
        
        # Motor velocities and control parameters
        self.droidRadius = 0.15

        self.M = np.array([[-0.33333333, -0.33333333,  0.66666667],
                           [-0.57736721,  0.57736721,  0.        ],
                           [ 0.33333333,  0.33333333,  0.33333333]])
        self.Minv = np.array([[-0.5,-0.86602540  , 1],
                              [-0.5, 0.86602540  , 1],
                              [1   , 0      , 1]])
        
        # Control inputs
        self.maxRunTime = 20 # maximum navigation runtime
        self.MaxLinearVelocity = 1.0 # Maximum linear speed m/s
        self.MaxAngularVelocity = np.pi # Maximum angular velocity rad/s
        self.compFilterEncoderValue = 0.95
        self.compFilterCompassValue = 0.9
        self.LinearSpeed = 0.0 # m/s       
        self.AngularSpeed = 0.0 # radian/s
        
        
        # Robot states
        self.xEstN = np.array([0,0,0])
        self.xEstN_T_1 = np.array([0,0,0])
        self.vEst = np.array([0,0,0])
        self.vEstT_1 = np.array([0,0,0])
        self.vEstN = np.array([0,0,0])
        self.vEstN_T_1 = np.array([0,0,0])
        self.vCommand = np.array([0,0,0])
        self.vCommandT_1 = np.array([0,0,0])
        self.navDt = 0
        self.navTime = time.time()
        self.navTimeT_1 = self.navTime - 0.030
        # Compass heading in radians from north. NOTE Clockwise direction is reverse of nav frame
        self.compassHeading = 0
        # Motor states
        self.velM1 = 0.0
        self.velM2 = 0.0
        self.velM3 = 0.0
        self.Kprop = 0.0
        self.Kint = 0.0
        self.Kder = 0.0
        
        self.runCommand = 0.0
 
        # Initialise INPUT data collection variables
        self.inVarNum = inVarNum  
        self.inRawData = bytearray(self.inVarNum * self.VarBytes)
        self.inData = [0] * self.inVarNum
        
        # Data container for save
        #ToDO: make this automatic, currently adding timestamp and setpoint
        self.logMotorData = False
        self.saveData = [[0] * (self.inVarNum + 1)]
        # Store : Command(Ux,Uy,Uo), Odometry(Vx,Vy,Vo), IMU(Ix,Iy,Io),Time
        self.saveState = []
        self.saveStateData = False
        self.initialTimer = 0.0

        self.plotState = False

        # Open serial comms to Arduino
        try:
            self.ser = serial.Serial(serial_port, baud)
            logger.info('Serial port connected')
        except:
            logger.error('Serial port not found')
            sys.exit()
        # Start 
        self.startReadThread() 

        # Initialise MPU
        self.di = droidInertial(imu_callback_ms = 50)
        logger.debug('Initialisation successful')
        
#------Bi-directional communication methods-----------

    def writeSerial(self):
        # Method compiles and converts message to byte string using defined data format.
        # Sends message over serial
        self.ser.reset_output_buffer()
        dataOut = [
            self.runCommand,
            np.round(self.velM1,3),
            np.round(self.velM2,3),
            np.round(self.velM3,3),
            self.Kprop,
            self.Kint, 
            self.Kder]   
        dataByte = struct.pack(self.VarType *len(dataOut),*dataOut)
        self.ser.write(dataByte)

    def getSerialData(self):
        # Method reads serial stream given data format expected (floats or ints)
        # Save data for next iteration
        self.inDataOld = self.inData
        # Set data time logging
        currentTime = time.time()
        dataTime = (currentTime - self.initialTimer)
        privateData = copy.deepcopy(self.inRawData[:]) # Synchronize all data to the same sample time
        
        byteSignal = privateData[0:4]       
        intSignal = struct.unpack('i', byteSignal)[0]
        self.inData[0] = intSignal
        for i in range(1,self.inVarNum):
            # Unpack message, inVarNum = number of variables, VarBytes = datatype
            data = privateData[(i*self.VarBytes):(self.VarBytes + i*self.VarBytes)]
            self.inData[i] = struct.unpack(self.VarType, data)[0] # Unpack always returns tuple
        self.inData.append(dataTime)
        
        # Decode signal from remote, update states
        # Unpack 4 Byte packet and read bits
        self.remoteSignal = [1 if c=='1' else 0 for c in bin(intSignal)[2:].zfill(4)]
        
        logger.debug(self.remoteSignal)
        # COMMS_STATE
        if self.remoteSignal[-1] == 0:
            self.remoteWaiting = True
            # logger.debug("NO_COMMS")
        if self.remoteSignal[-1] == 1:
            self.remoteWaiting = False
        # MOTOR_STATE
        if self.remoteSignal[-2] == 0:
            self.droidMoving = False
            self.droidRunning = False
            # logger.debug("PARKED")
        if self.remoteSignal[-2] == 1:
            self.droidRunning = True
            # logger.debug("RUNNING")
        if self.remoteSignal[-3] == 0:
            self.droidMoving = False
        if self.remoteSignal[-3] == 1:
            self.droidRunning = True
            logger.debug("TIMEOUT")
            
        # todo: Change to allow variable data size inVarNum. Try append([*self.inData])
        if self.logMotorData:
            self.saveData.append([self.inData])
        # writeSerial(&message,&setspeed_M1,&speed_M1,&out_M1,&setspeed_M2,&speed_M2,
        # &out_M2,&setspeed_M3,&speed_M3,&out_M3);  
        # logger.debug('Data updated:%f', dataTime)
    
    def startReadThread(self):
        if self.thread == None:
            self.thread = Thread(target=self.readSerialThread,daemon = True)
            #self.thread.daemon = True
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)
            
    
    def readSerialThread(self):    # retrieve data
        time.sleep(1)  # give some buffer time for retrieving data
        self.ser.reset_input_buffer()
        while (self.isRun):
            self.ser.readinto(self.inRawData)
            self.isReceiving = True
            time.sleep(0.000001)
        logger.debug("Droidcontrol closing read thread")
            
#------High level control methods-----------
            
            
    def calcMotorVels(self, linearVel, heading, angularVel):
        # 3 wheel omniwheel kinematics - OBSOLETE
        # Transforms from velocity/heading/angular velocity to motor speeds
        self.velM1 = ((linearVel * (-0.5 * np.cos(heading) - np.sqrt(3) / 2 * np.sin(heading))) + (2 * angularVel * self.droidRadius))
        self.velM2 = ((linearVel * (-0.5 * np.cos(heading) + np.sqrt(3) / 2 * np.sin(heading))) + (2 * angularVel * self.droidRadius))
        self.velM3 = (linearVel * np.cos(heading) + (2 * angularVel * self.droidRadius))
    
    def inverseKinematics(self, vEst):
        # See calcDroidKinematics.p for details
        vEst = vEst/2
        vEst[2] = vEst[2] * self.droidRadius 
        [self.velM1,self.velM2,self.velM3] = self.Minv @ vEst
        
    
    def forwardKinematics(self, M1, M2, M3):
        # Forward kinematic model: INPUT - wheel velocities, OUTPUT velocity, ang vel
        vEst = self.M @ np.array([M1,M2,M3])
        vEst[2] = vEst[2]/(self.droidRadius)
        vEst = vEst*2
        return vEst
    
    def getInertialData(self):
        aLinIMU = self.di.accel
        vAngIMU = self.di.omega
        xAngIMU = self.di.theta
        omegaEst = np.array([vAngIMU[0],vAngIMU[1], vAngIMU[2]])
        aEst = np.array([aLinIMU[0],aLinIMU[1], aLinIMU[2]])
        thetaEst = xAngIMU[0] % (np.pi*2)
        self.timeIMU = self.di.TimeK
        
        return omegaEst, aEst, thetaEst
    
    def calcDroidSpeed(self,error):
        normVec = np.sqrt(error[0]**2 + error[1]**2)
        normAng = np.abs(error[2])
        
        # Normalise direction vector and protect against divide by zero
        if normVec > 0:
            unitVecX = error[0]/normVec
            unitVecY = error[1]/normVec
        else:
            unitVecX = 0
            unitVecY = 0
            
        if normAng > 0:
            unitVecA = error[2]/normAng
        else:
            unitVecA = 0 
        
        # Proportional speed controller
        Speed_L = np.min([self.LinearSpeed, 5*normVec])
        Speed_A = np.min([self.AngularSpeed, 5*normAng])
        
        self.vCommand = np.round([unitVecX * Speed_L, unitVecY * Speed_L, unitVecA * Speed_A],decimals=3)
#         logger.debug('Speed_L: {0:0.4f}'.format(Speed_L))
#         logger.debug('Speed_A: {0:0.4f}'.format(Speed_A))
    
    def processCommands(self,dirX,dirY,rotL,rotR, startRec, stopRec):
        # Process data from f710 joystick, reverse Y axis control      
        dirY = -dirY       
        velX = dirX * self.LinearSpeed
        velY = dirY * self.LinearSpeed

        velA = (rotL-rotR) * self.AngularSpeed
        # print('RotL {0:0.3f}, RotR: {1:0.3f}'.format(rotL,rotR))
        self.vCommand = np.round([velX, velY, velA],decimals=3)
        # REMOVE
        self.runCommand = 0
        # Pause drive if no command given
        if (self.vCommand != 0).any():
            self.runCommand = 1
        else:
            self.runCommand = 0

        # Control data logging
        # Handle double button press
        if (startRec == 1) and (stopRec == 1):
            pass
            
        elif startRec == 1:
            
            if self.saveStateData is True:
                pass
            else:
                logger.info('DC START RECORDING')
                self.saveStateData = True
                self.saveState = [] 

        elif stopRec == 1:
            
            if self.saveStateData is not True:
                pass
            else:
                self.saveOutput(self.saveState,self.saveStateFileName)
                logger.info('DC STOP RECORDING')
                logger.info('--------State data saving---------')
                self.saveStateData = False 
                     
        else:
            pass
        
        # logger.debug('Velocity command x: {0:0.3f}, y: {1:0.3f}, Omega: {2:0.3f},'.format(self.vCommand[0],self.vCommand[1],self.vCommand[2]))

    def compFilter(self, data1, data2,A):
        filt = (data1 * A) + (data2 * (1-A))
        return filt
    
    def estRobotState(self):       
        # Get latest information from sensors 
        self.getSerialData()
        # Calc time difference
        self.navTime = time.time()
        self.navDt = (self.navTime - self.navTimeT_1)

        # Remove noisy signal at standstill
        if self.droidRunning == True:
            # Get odometry estimate from encoders 
            estM1 = self.inData[2]
            estM2 = self.inData[5]
            estM3 = self.inData[8]
            vEstEnc = self.forwardKinematics(estM1, estM2, estM3) 
            omegaEstEnc = vEstEnc[2] 
            # Get odometry estimate from IMU         
            omegaEstImu, aEstImu, Theta_C = self.getInertialData()
            self.compassHeading = Theta_C
                     
            # Fuse encoder and gyro rotational rates
            omegaEst = self.compFilter(omegaEstEnc, omegaEstImu[2], self.compFilterEncoderValue)
            # Propagate rates and fuse with compass heading. NOTE compass orientation is reversed
            Theta_G = self.xEstN_T_1[2] + (omegaEst * self.navDt)
            thetaEst = self.compFilter(-Theta_C, Theta_G, self.compFilterCompassValue)

            # Create current rotation matrix to Nav frame (NED)
            C_nr = np.array([[np.cos(thetaEst), -np.sin(thetaEst)],[np.sin(thetaEst),np.cos(thetaEst)]])
            # Convert 2D IMU body frame measurements to Nav frame
            aEstImu_N = (np.dot(C_nr,aEstImu[0:2]))
            vEstImu_N = self.vEstN_T_1[0:2] + (aEstImu_N * self.navDt)
            vEstImu_N = np.append(vEstImu_N,omegaEst) # Back to 3D
            # Convert encoder velocity estimate to Nav frame
            vEstEnc_N = (np.dot(C_nr,vEstEnc[0:2]))
            vEstEnc_N = np.append(vEstEnc_N,omegaEstEnc)
            # Fuse velocity estimate and propagate displacement in nav frame
            self.vEstN = self.compFilter(vEstEnc_N, vEstImu_N, self.compFilterEncoderValue)
            self.xEstN = self.xEstN_T_1 + ((self.vEstN + self.vEstN_T_1)/2 * self.navDt)
            self.xEstN[2] = thetaEst % (2*np.pi)
            print("")
            logger.debug('vEst_N Imu x: {0:0.3f}, y: {1:0.3f}, Omega: {2:0.3f},'.format(vEstImu_N[0],vEstImu_N[1],vEstImu_N[2]))
            logger.debug('aEst_N Imu x: {0:0.3f}, y: {1:0.3f}'.format(aEstImu_N[0],aEstImu_N[1]))
            logger.debug('vEst_N Enc x: {0:0.3f}, y: {1:0.3f}, Omega: {2:0.3f},'.format(vEstEnc_N[0],vEstEnc_N[1],vEstEnc_N[2]))
            logger.debug('vEst_N Comp x: {0:0.3f}, y: {1:0.3f}, Omega: {2:0.3f},'.format(self.vEstN[0],self.vEstN[1],self.vEstN[2]))
            logger.debug('Gyro Theta: {0:0.3f}, Compass Theta (corrected): {1:0.3f}, Fusion Theta: {2:0.3f}'.format(Theta_G,(np.pi*2 -Theta_C),thetaEst))
        

        else:
            self.vEstN = np.array([0,0,0])
            self.vEstN_T_1 = np.array([0,0,0])
            self.compassHeading = self.getInertialData()[2]
            # For logging
            vEstEnc = np.array([0,0,0])
            aEstImu = np.array([0,0,0])
            thetaEst = self.xEstN[2]
            logger.debug("-----Droid not running ------")

        # Store state for next iteration
        self.xEstN_T_1 = self.xEstN    
        self.vEstN_T_1 = self.vEstN
        self.navTimeT_1 = self.navTime

        # Save state
        if (self.saveStateData is True):
            dataOut = self.vCommand.tolist() + vEstEnc.tolist() + aEstImu.tolist() +  [thetaEst] + self.xEstN.tolist() + [int(self.droidRunning)] +[self.navTime]
            self.saveState.append(dataOut)
        
    def calcTargetError(self, target):
        error = target - self.xEstN
        # Bring error to interval(0,360) 
        error[2] = error[2] % (2*np.pi)
        # Turn in closest direction
        if error[2] > np.pi/2:
            error[2] = error[2] - (2*np.pi)
#         logger.debug('Target x: {0:0.3f}, y: {1:0.3f}, theta: {2:0.3f},'.format(target[0],target[1],target[2]))
        logger.debug('xEst x: {0:0.3f}, y: {1:0.3f}, theta: {2:0.3f},'.format(self.xEstN[0],self.xEstN[1],self.xEstN[2]))
        logger.debug('Error x: {0:0.3f}, y: {1:0.3f}, theta: {2:0.3f},'.format(error[0],error[1],error[2]))           

        return error
    
    def navController(self, target):
        # for relative motion only
        self.xEstN = np.array([0,0,0])
        self.xEstN_T_1 = np.array([0,0,0])
        self.vEst = np.array([0,0,0])
        self.vEstT_1 = np.array([0,0,0])
        errorTol = np.array([0.05, 0.05, 0.01]) # +/- 50mm, 5 deg
        error = self.calcTargetError(target)
        # Estimate time to run at set speed
        runTime = error / np.array([self.LinearSpeed, self.LinearSpeed,self.AngularSpeed])
        if (runTime > self.maxRunTime).any():
            logger.warning('No speed set, infinite runtime calculated - aborting')
            return
        self.navTimeT_1 = time.time()
#       time.sleep(0.03)
        self.navTime = time.time()
        self.runCommand = 1.0   
        # Loop until error <= tolerance
        while (np.abs(error) > errorTol).any():
            # Get odometry for time period
            self.estRobotState()
            error = self.calcTargetError(target)
            # Calculate desired droid linear and angulare velocity
            self.calcDroidSpeed(error)
            # Determine motor speeds
            self.inverseKinematics(self.vCommand)

            # Write motor speeds
            self.writeSerial()
            self.vCommandT_1 = self.vCommand
            
            # Diagnostics and data display
#       logger.debug('M1: {0:0.3f}, M2: {1:0.3f}, M3: {2:0.3f},'.format(self.velM1,self.velM2,self.velM3))
            time.sleep(0.02)
                             
        self.runCommand = 0.0
        self.writeSerial()
        time.sleep(0.030)
        while self.droidMoving == True:
            self.writeSerial()
            self.getSerialData()
            time.sleep(0.025)
        logger.debug('Droid Parked')

    def driveDroid(self):
        # Estimate robot state using sensor feedback
        self.estRobotState()  
        # Determine motor speeds
        self.inverseKinematics(self.vCommand)
        # Write motor speeds
        self.writeSerial()
        self.vCommandT_1 = self.vCommand

        # logger.debug('M1: {0:0.3f}, M2: {1:0.3f}, M3: {2:0.3f},'.format(self.velM1,self.velM2,self.velM3))
            
#------Tests--------------------------------
    
    def testVectorDrive(self, Distance, Heading):
        Heading = Heading %(2*np.pi)
        
        # Calculate time to drive
        if Distance <= 0:
            runTime = 0
            logger.debug("No distance, aborting")
            return
        elif self.LinearSpeed <= 0:
            runTime = 0
            logger.debug("No linear velocity, aborting")
            return
        else:
            runTime = abs(Distance / self.LinearSpeed)
            logger.debug("Estimated runtime = %0.2f " % runTime)
            if runTime > 10:
                logger.debug("Runtime too long, aborting")
                return

        # Calculate 3 motor velocities from input
        self.calcMotorVels(self.LinearSpeed, Heading, 0)
        logger.debug("M1: %f" % self.velM1)
        logger.debug("M2: %f" % self.velM2)
        logger.debug("M3: %f" % self.velM3)
        # Initialise timer
        self.initialTimer = time.time()
        
        self.runCommand = 1.0
        driveTime = 0.0
        # Test drive to target - simple    
        while driveTime <= runTime:
            self.getSerialData()
            self.writeSerial()
            # Naive open-loop odometry 
            driveTime = time.time() - self.initialTimer
            displacement = self.LinearSpeed * driveTime
            for i in range(self.inVarNum):
                print("%.3f" % self.inData[i])
            logger.debug('Current displacement: %0.3f' % displacement)
            time.sleep(0.025)
            
        self.runCommand = 0.0
        self.writeSerial()
        time.sleep(0.030)
        while self.droidMoving == True: # CHECK THIS, NEVER MOVING
            self.writeSerial()
            self.getSerialData()
            time.sleep(0.025)
        logger.debug('Droid Parked')
        
    def testRotationDrive(self, Rotation):
        
        direction = Rotation/abs(Rotation)
        Rotation = Rotation %(2*np.pi)
        # Calculate time to drive
        if Rotation == 0:
            runTime = 0
            logger.debug("No rotation, aborting")
            return
        elif self.AngularSpeed ==  0:
            runTime = 0
            logger.debug("No angular velocity, aborting")
            return
        else:
            runTime = abs(Rotation / (self.AngularSpeed))
            logger.debug("Estimated runtime = %0.2f " % runTime)
            if runTime > 10:
                logger.debug("Runtime too long, aborting")
                return
        # Calculate velocity including sign to reach destination
        AngularVelocity = self.AngularSpeed * direction
        # Calculate 3 motor velocities from input
        self.calcMotorVels(0, 0, AngularVelocity)
        logger.debug("M1: %f" % self.velM1)
        logger.debug("M2: %f" % self.velM2)
        logger.debug("M3: %f" % self.velM3)
        # Initialise timer
        self.initialTimer = time.time()
        
        self.runCommand = 1.0
        driveTime = 0.0
        # Test drive to target - simple    
        while driveTime <= runTime:
            self.getSerialData()
            self.writeSerial()
            # Naive open-loop odometry 
            driveTime = time.time() - self.initialTimer
            # Replace with estimate from odometry
            rotation = self.AngularSpeed * driveTime
            for i in range(self.inVarNum):
                print("%.3f" % self.inData[i])
            logger.debug('Current rotation: %0.3f' % rotation)
            time.sleep(0.025)
            
        self.runCommand = 0.0
        self.writeSerial()
        time.sleep(0.030)
        while self.droidMoving == True:
            self.writeSerial()
            self.getSerialData()
            time.sleep(0.025)
        logger.debug('Droid Parked')
 
    def testMotors(self):
        self.velM1 = 0.2
        self.velM2 = 0.2
        self.velM3 = 0.2
        self.runCommand = 1.0
        self.writeSerial()
        time.sleep(0.025)
        self.getSerialData()
        for _ in range(10):
            self.writeSerial()
            self.getSerialData()
            for i in range(self.inVarNum):
                print("%.3f" % self.inData[i])
            time.sleep(0.025)
        self.runCommand = 0.0
        self.writeSerial()
        time.sleep(0.025)
        self.getSerialData()
        
#------Data visualisation methods-----------
    
    def saveOutput(self,data,filename):
        saveDataNP = np.array(data)
        np.savetxt(filename,saveDataNP,delimiter=',')
        logger.debug('State data saved to file')          

    def plotOutput(self):
        # Plot values: setPoint, motorSpeed, PWM, timeStamp, signal
        fig, ax = plt.subplots(1,3, figsize=(12,9))                    
        ax[0,0].plot(self.saveDataNP[:,9],self.saveDataNP[:,0], 'r',label='M1 Setpoint')
        ax[0,0].plot(self.saveDataNP[:,9],self.saveDataNP[:,1], 'b',label='M1 Motor speed')
        ax[0,0].set_ylabel('Velocity - m/s')
        ax[0,0].set_ylim(-2,2)
        ax2 = ax.twinx()
        ax2[0,0].plot(self.saveDataNP[:,9],self.saveDataNP[:,2], 'g',label='M1 PWM')
        ax2[0,0].set_ylabel('Control u - PWM')
        ax2[0,0].set_ylim(-255,255)
        ax[0,0].set_title('M1')
        
        ax[0,1].plot(self.saveDataNP[:,9],self.saveDataNP[:,3], 'r',label='M2 Setpoint')
        ax[0,1].plot(self.saveDataNP[:,9],self.saveDataNP[:,4], 'b',label='M2 Motor speed')
        ax[0,1].set_ylabel('Velocity - m/s')
        ax[0,1].set_ylim(-2,2)
#         ax2 = ax.twinx()
        ax2[0,1].plot(self.saveDataNP[:,9],self.saveDataNP[:,5], 'g',label='M2 PWM')
        ax2[0,1].set_ylabel('Control u - PWM')
        ax2[0,1].set_ylim(-255,255)
        ax[0,1].set_title('M2')
    
        ax[0,2].plot(self.saveDataNP[:,9],self.saveDataNP[:,7], 'r',label='M1 Setpoint')
        ax[0,2].plot(self.saveDataNP[:,9],self.saveDataNP[:,8], 'b',label='M2 Motor speed')
        ax[0,2].set_ylabel('Velocity - m/s')
        ax[0,2].set_ylim(-2,2)
#         ax2 = ax.twinx()
        ax2[0,2].plot(self.saveDataNP[:,9],self.saveDataNP[:,9], 'g',label='M3 PWM')
        ax2[0,2].set_ylabel('Control u - PWM')
        ax2[0,2].set_ylim(-255,255)
        ax[0,2].set_title('M3')

        fig.legend(loc="upper right", bbox_to_anchor=(1,1), bbox_transform=ax.transAxes)
        plt.show()


          
#------Shutdown methods-----------

    def close(self):
        global shutdown_flag
        shutdown_flag = True
        # Shutdown thread
        self.isRun = False
        time.sleep(1)
        # Close serial port
        self.ser.close()
        self.di.close()
        logger.debug('DroidControl releasing resources')

'''
Captures the ctrl+c keyboard command to close the services and free 
resources gracefully.
'''
def signal_handler(signal, frame):
    global shutdown_flag
    shutdown_flag = True
    dc.close()
    logger.info('Closing gracefully. Bye Bye!')
    sys.exit()


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    global dc
    dc = droidControl('/dev/ttyUSB0',38400,10,7,'f')
    t0 = time.time()
            
    #Use GUI loop for online changes
    # Get user input 
    dc.LinearSpeed = 0.2 # m.s^-1
    dc.AngularSpeed = 0.1 * np.pi # degrees.s^-1
    
    Heading = 3*np.pi/5 # radians in robot frame
    Distance = 1 # metres
    Rotation = np.pi/4 # radian in world frame
    
    dc.Kprop = 1.5 # Proportional gain
    dc.Kint = 40 # Integral gain
    dc.Kder = 0.001 # Derivative gain
    dc.runCommand = 0.0
    
#     dc.testVectorDrive(Distance, Heading)
#     time.sleep(2)
    # dc.testRotationDrive(Rotation)
#     
#     # Save file
#     dc.saveOutput()
#     
#     # Plot measurements
#     dc.plotOutput()

    print('out of loop')
    dc.close()

    print('All done')
        

