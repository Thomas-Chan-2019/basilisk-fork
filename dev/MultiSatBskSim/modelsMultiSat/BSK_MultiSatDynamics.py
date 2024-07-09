#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import numpy as np
from Basilisk import __path__
# thrusterDynamicEffector -> thrusterStateEffector ? 
from Basilisk.simulation import (spacecraft, simpleNav, simpleMassProps, reactionWheelStateEffector,
                                 thrusterDynamicEffector, simpleSolarPanel, simplePowerSink, simpleBattery, fuelTank,
                                 ReactionWheelPower)
from Basilisk.simulation import (extForceTorque) # Needed for external disturbances?
from Basilisk.utilities import (macros as mc, unitTestSupport as sp, RigidBodyKinematics as rbk,
                                simIncludeRW, simIncludeThruster)

import inspect, os, sys
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../../')

import scConfig

bskPath = __path__[0]


class BSKDynamicModels:
    """
    Defines the Dynamics class.
    """
    def __init__(self, SimBase, dynRate, spacecraftIndex, initConfigs, includeExtDisturbances=False, includeSubsystems=False):
        self.I_sc = None # moment of inertia [3x3 matrix, kg*m^2] 
        self.m_sc = None # mass [kg]
        self.solarPanelAxis = None
        self.numRW = 4 # change RW number when needed
        self.numThr = None # change Thruster number when needed
        self.tankModel = None
        self.spacecraftIndex = spacecraftIndex
        self.initConfig = initConfigs[spacecraftIndex] # Init config from json file.
        
        # Define process name, task name and task time-step
        self.taskName = "DynamicsTask" + str(spacecraftIndex)
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create task
        SimBase.dynProc[spacecraftIndex].addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        # Instantiate Dyn modules as objects
        # self.scObject = spacecraft.Spacecraft()
        self.scObject = None
        self.simpleNavObject = simpleNav.SimpleNav()
        self.simpleMassPropsObject = simpleMassProps.SimpleMassProps()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.rwFactory = simIncludeRW.rwFactory()
        self.thrusterDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.thrusterFactory = simIncludeThruster.thrusterFactory()

        if includeExtDisturbances:
            # External disturbance mapping: TODO
            self.extDisturbance = extForceTorque.ExtForceTorque()

        if includeSubsystems:
            self.solarPanel = simpleSolarPanel.SimpleSolarPanel()
            self.powerSink = simplePowerSink.SimplePowerSink()
            self.powerMonitor = simpleBattery.SimpleBattery()
            self.fuelTankStateEffector = fuelTank.FuelTank()
            self.rwPowerList = []
            for item in range(self.numRW):
                self.rwPowerList.append(ReactionWheelPower.ReactionWheelPower())
        
        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects(SimBase, includeExtDisturbances, includeSubsystems)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleMassPropsObject, 99)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, 100)
        SimBase.AddModelToTask(self.taskName, self.thrusterDynamicEffector, 100)
        
        if includeExtDisturbances: # check if we add external disturbances
            SimBase.AddModelToTask(self.taskName, self.extDisturbance, 100)
        
        if includeSubsystems:
            SimBase.AddModelToTask(self.taskName, self.solarPanel, 100)
            SimBase.AddModelToTask(self.taskName, self.powerSink, 100)
            SimBase.AddModelToTask(self.taskName, self.powerMonitor, 100)
            SimBase.AddModelToTask(self.taskName, self.fuelTankStateEffector, 100)
            for item in range(self.numRW):
                SimBase.AddModelToTask(self.taskName, self.rwPowerList[item], 100)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetSpacecraftHub(self): # Modified from original to support self-developped scConfig module.
        """
        Defines the spacecraft object properties.
        """
        # See dev/scConfig.py for details on creating a S/C, based on Astrobee config for now.
        # self.scObject = scConfig.createSC(self.initConfig.scName)
        scObject, sc_config = scConfig.createSC(self.initConfig.scName)
        self.scObject = scObject
        self.scObject.ModelTag = "sat-" + str(self.spacecraftIndex) # Update model tag in accordance to SC index
        self.I_sc = self.scObject.hub.IHubPntBc_B
        self.m_sc = self.scObject.hub.mHub # Taken from spacecraft.Spacecraft() module!
        
        return sc_config # Return a SCConfig class according to scConfig module for Thruster/RW creation.

    def SetGravityBodies(self, SimBase):
        """
        Specify what gravitational bodies to include in the simulation
        """
        # Attach the gravity body
        SimBase.EnvModel.gravFactory.addBodiesTo(self.scObject)

    def SetGroundLocations(self, SimBase):
        """
        Adds the spacecraft to the ground location module.
        """
        SimBase.EnvModel.groundStation.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetEclipseObject(self, SimBase):
        """
        Adds the spacecraft to the eclipse module.
        """
        SimBase.EnvModel.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetSimpleNavObject(self):
        """
        Defines the navigation module.
        """
        self.simpleNavObject.ModelTag = "SimpleNavigation" + str(self.spacecraftIndex)
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

    def SetSimpleMassPropsObject(self):
        """
        Defines the navigation module.
        """
        self.simpleMassPropsObject.ModelTag = "SimpleMassProperties" + str(self.spacecraftIndex)
        self.simpleMassPropsObject.scMassPropsInMsg.subscribeTo(self.scObject.scMassOutMsg)

    # We should redefine the axis of the RWs!
    def SetReactionWheelDynEffector(self):
        """
        Defines the RW state effector.
        """
        # specify RW momentum capacity
        maxRWMomentum = 50.  # Nms, TODO

        # Hard code RW Config for now:
        # Define RW directly at body x,y,z frame at [0,0,0] position, i.e. cubie centre of S/C assuming a perfect cubic structure:
        RW1 = self.rwFactory.create('Honeywell_HR12', [1, 0, 0], Omega=0.  # Initialized RPM
                           , u_max=0.001
                           , maxMomentum=maxRWMomentum
                           , rWB_B=[0., 0., 0.]) # Default position vector, to move to scConfig
        RW2 = self.rwFactory.create('Honeywell_HR12', [0, 1, 0], Omega=0.  # Initialized RPM
                           , u_max=0.001
                           , maxMomentum=maxRWMomentum
                           , rWB_B=[0., 0., 0.]) # Default position vector, to move to scConfig
        RW3 = self.rwFactory.create('Honeywell_HR12', [0, 0, 1], Omega=0.  # Initialized RPM
                           , u_max=0.001
                           , maxMomentum=maxRWMomentum
                           , rWB_B=[0., 0., 0.]) # Default position vector, to move to scConfig
        
        # # Define orthogonal RW pyramid
        # # -- Pointing directions
        # rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * mc.D2R
        # rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R
        # rwPosVector = [[0.8, 0.8, 1.79070],
        #                [0.8, -0.8, 1.79070],
        #                [-0.8, -0.8, 1.79070],
        #                [-0.8, 0.8, 1.79070]]

        # for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
        #     gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
        #     self.rwFactory.create('Honeywell_HR16',
        #                           gsHat,
        #                           maxMomentum=maxRWMomentum,
        #                           rWB_B=posVector)

        self.numRW = self.rwFactory.getNumOfDevices()
        self.rwFactory.addToSpacecraft("RWArray" + str(self.spacecraftIndex), self.rwStateEffector, self.scObject)
        
        # print(self.rwFactory.getConfigMessage()) # To figure out how to check config
        

    # We should redefine the axis of the Thrusters!
    def SetThrusterDynEffector(self, sc_config): 
        """
        Defines the thruster state effector.
        """
        maxThrust = sc_config.MaxThrust
        location = sc_config.thrLocation
        direction = sc_config.thrDirection
        
        # create the thruster devices by specifying the thruster type and its location and direction
        for pos_B, dir_B in zip(location, direction):
            # self.thrusterFactory.create('TEST_Thruster', pos_B, dir_B, useMinPulseTime=False, MaxThrust=maxThrust)
            self.thrusterFactory.create('MOOG_Monarc_5', pos_B, dir_B, useMinPulseTime=False, MaxThrust=maxThrust)
            # Decide if choose the thruster type too later!

        self.numThr = self.thrusterFactory.getNumOfDevices()

        # create thruster object container and tie to spacecraft object
        self.thrusterFactory.addToSpacecraft("thrusterFactory", self.thrusterDynamicEffector, self.scObject)
        
        # print(self.thrusterFactory.getConfigMessage()) # To figure out how to check config

    def SetFuelTank(self):
        """
        Defines the fuel tank for the thrusters.
        """
        # Define the tank
        self.fuelTankStateEffector.setTankModel(fuelTank.TANK_MODEL_UNIFORM_BURN)
        self.tankModel = fuelTank.cvar.FuelTankModelUniformBurn
        self.tankModel.propMassInit = 50.0
        self.tankModel.maxFuelMass = 75.0
        self.tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]
        self.fuelTankStateEffector.r_TB_B = [[0.0], [0.0], [0.0]]
        self.tankModel.radiusTankInit = 1
        self.tankModel.lengthTank = 1
        
        # Add the tank and connect the thrusters
        self.scObject.addStateEffector(self.fuelTankStateEffector)
        self.fuelTankStateEffector.addThrusterSet(self.thrusterDynamicEffector)

    def SetReactionWheelPower(self):
        """Sets the reaction wheel power parameters"""
        for item in range(self.numRW):
            self.rwPowerList[item].ModelTag = self.scObject.ModelTag + "RWPower" + str(item)
            self.rwPowerList[item].basePowerNeed = 5.  # baseline power draw, Watt
            self.rwPowerList[item].rwStateInMsg.subscribeTo(self.rwStateEffector.rwOutMsgs[item])
            self.rwPowerList[item].mechToElecEfficiency = 0.5

    def SetSolarPanel(self, SimBase):
        """Sets the solar panel"""
        self.solarPanel.ModelTag = "solarPanel"
        self.solarPanel.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.solarPanel.sunEclipseInMsg.subscribeTo(SimBase.EnvModel.eclipseObject.eclipseOutMsgs[0])  # choose the earth message
        self.solarPanel.sunInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.solarPanelAxis = [0, 0, 1]
        self.solarPanel.setPanelParameters(self.solarPanelAxis,  # panel normal vector in the body frame
                                           0.4 * 0.4 * 2 + 0.2 * 0.4 * 2,  # area, m^2
                                           0.35)  # efficiency

    def SetPowerSink(self):
        """Defines the energy sink parameters"""
        self.powerSink.ModelTag = "powerSink"
        self.powerSink.nodePowerOut = -2.  # Watt

    def SetBattery(self):
        """Sets up the battery with all the power components"""
        self.powerMonitor.ModelTag = "powerMonitor"
        self.powerMonitor.storageCapacity = 2 * 60.0 * 3600.0  # Convert from W-hr to Joule
        self.powerMonitor.storedCharge_Init = self.powerMonitor.storageCapacity * 0.6  # 40% depletion

        # attach the sources/sinks to the battery
        self.powerMonitor.addPowerNodeToModel(self.solarPanel.nodePowerOutMsg)
        self.powerMonitor.addPowerNodeToModel(self.powerSink.nodePowerOutMsg)
        for item in range(self.numRW):
            self.powerMonitor.addPowerNodeToModel(self.rwPowerList[item].nodePowerOutMsg)

    def SetExtForceTorque(self):
        self.extDisturbance.ModelTag = "externalDisturbance"
        # Hard coded external forces, to decide if use, or create atmospheric/J2 drag modules:
        # self.extDisturbance.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
        # self.extDisturbance.extForce_B = [[0.],[0.],[-0.]] for setting body force 
        # self.extDisturbance.extForce_N = [[0.],[0.],[-0.]] for setting inertial force
        self.scObject.addDynamicEffector(self.extDisturbance)
    
    # Global call to initialize every module
    def InitAllDynObjects(self, SimBase, includeExtDisturbances=False, includeSubsystems=False):
        """
        Initializes all dynamic objects.
        """
        sc_config = self.SetSpacecraftHub() # Returned SCConfig class for thruster/RW creation.
        self.SetGravityBodies(SimBase)
        self.SetReactionWheelDynEffector() # TODO - change axis / power
        self.SetThrusterDynEffector(sc_config) # TODO - change axis / thrust forces
        
        if includeExtDisturbances:
            self.SetExtForceTorque() # TODO - check if needed at first stage without disturbances
        
        self.SetSimpleNavObject()
        self.SetSimpleMassPropsObject()
        self.SetGroundLocations(SimBase)
        self.SetEclipseObject(SimBase)
        
        if includeSubsystems:
            self.SetFuelTank()
            self.SetReactionWheelPower()
            self.SetSolarPanel(SimBase)
            self.SetPowerSink()
            self.SetBattery()
