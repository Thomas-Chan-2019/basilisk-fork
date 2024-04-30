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

import itertools

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (inertial3D, locationPointing, attTrackingError, mrpFeedback,
                                    rwMotorTorque, spacecraftReconfig, thrForceMapping, forceTorqueThrForceMapping, thrFiringSchmitt) # most of the pointing/controller/feedback modules are not used
from Basilisk.utilities import (macros as mc, fswSetupThrusters, fswSetupRW) # need the fswSetupRW module?
from Basilisk.utilities import deprecated

import inspect, os, sys
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../../')
import transError, PIDController # self developed modules

class BSKFswModels:
    """Defines the FSW class"""
    def __init__(self, SimBase, fswRate, spacecraftIndex):
        # define empty class variables
        self.targetSCIndex = 0 # Set target SC index among ALL created SC at 0, can move to scenario creation later!
        self.spacecraftIndex = spacecraftIndex
        self.decayTime = None # control decay time
        self.xi = None # damping ratio
        self.modeRequest = "standby"
        self.stationKeeping = "OFF"

        self.vcMsg = None
        self.fswThrusterConfigMsg = None
        self.fswRwConfigMsg = None
        self.cmdTorqueMsg = None
        self.cmdTorqueDirectMsg = None
        self.attRefMsg = None
        self.attGuidMsg = None
        self.cmdRwMotorMsg = None
        
        # Added for thrusters translational control
        self.cmdForceMsg = None 
        self.cmdForceDirectMsg = None 
        # transError messages:
        self.targetTransInMsg = None
        self.chaserTransInMsg = None
        self.transRefInMsg = None
        self.transGuidOutMsg = None

        # Define process name and default time-step for all FSW tasks defined later on
        self.processName = SimBase.FSWProcessName[spacecraftIndex]
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        # Create tasks
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("inertialPointTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 20)
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("trackingErrorTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 10)
        # New tasks for new modules:
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("transErrorTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 10) # Following priority of trackingErrorTask (pending to be removed)
        # transControllerTask created here is later called below to add the actuator control axis (rwMotorTorque & [TODO] thruster mapping module)!
        SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("transControllerTask" + str(spacecraftIndex),
                                                                       self.processTasksTimeStep), 5) # Following priority of mrpFeedbackRWsTask (pending to be removed)
        
        # # ------------------ <START> PEND REMOVE ------------------
        
        # SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("sunPointTask" + str(spacecraftIndex),
        #                                                                self.processTasksTimeStep), 20)
        # SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("locPointTask" + str(spacecraftIndex),
        #                                                                self.processTasksTimeStep), 20)
        # SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("spacecraftReconfigTask" + str(spacecraftIndex),
        #                                                                self.processTasksTimeStep), 15)
        # SimBase.fswProc[spacecraftIndex].addTask(SimBase.CreateNewTask("mrpFeedbackRWsTask" + str(spacecraftIndex),
        #                                                                self.processTasksTimeStep), 5)
        # # ------------------ <END> PEND REMOVE ------------------
        


        # self.sunPoint = locationPointing.locationPointing()
        # self.sunPoint.ModelTag = "sunPoint"
        
        # self.sunPoint = locationPointing.locationPointing()
        # self.sunPoint.ModelTag = "sunPoint"


        # self.locPoint = locationPointing.locationPointing()
        # self.locPoint.ModelTag = "locPoint"
        
        # self.locPoint = locationPointing.locationPointing()
        # self.locPoint.ModelTag = "locPoint"

        # Create module data and module wraps
        self.inertial3DPoint = inertial3D.inertial3D() # Get att. reference MRP message
        self.inertial3DPoint.ModelTag = "inertial3D"
        
        self.trackingError = attTrackingError.attTrackingError()
        self.trackingError.ModelTag = "trackingError"

        self.rwMotorTorque = rwMotorTorque.rwMotorTorque()
        self.rwMotorTorque.ModelTag = "rwMotorTorque"

        # Add new modules:
        self.transError = transError.transError()
        self.transError.ModelTag = "transError"
        
        self.transController = PIDController.PIDController()
        self.transController.ModelTag = "transController"

        # <-- PEND ADD Thruster Mapping like Torque Mapping Module `rwMotorTorque`-->
        # self.thrForceMapping = thrForceMapping.thrForceMapping()
        self.thrForceMapping = forceTorqueThrForceMapping.forceTorqueThrForceMapping()
        self.thrForceMapping.ModelTag = "thrForceMapping"

        self.thrustOnTimeFiring = thrFiringSchmitt.thrFiringSchmitt()
        self.thrustOnTimeFiring.ModelTag = "thrustOnTimeFiring"
        
        # # ------------------ <START> PEND REMOVE ------------------
        
        # self.sunPoint = locationPointing.locationPointing()
        # self.sunPoint.ModelTag = "sunPoint"

        # self.locPoint = locationPointing.locationPointing()
        # self.locPoint.ModelTag = "locPoint"

        # self.spacecraftReconfig = spacecraftReconfig.spacecraftReconfig()
        # self.spacecraftReconfig.ModelTag = "spacecraftReconfig"


        # self.mrpFeedbackRWs = mrpFeedback.mrpFeedback()
        # self.mrpFeedbackRWs.ModelTag = "mrpFeedbackRWs"
        
        # # ------------------ <END> PEND REMOVE ------------------


        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask("inertialPointTask" + str(spacecraftIndex), self.inertial3DPoint, 10)
        SimBase.AddModelToTask("trackingErrorTask" + str(spacecraftIndex), self.trackingError, 9)
        # Add new modules to simulation:
        SimBase.AddModelToTask("transErrorTask" + str(spacecraftIndex), self.transError, 9) # transError
        SimBase.AddModelToTask("transControllerTask" + str(spacecraftIndex), self.transController, 7) # transControllerTask is defined above when created a new event!

        SimBase.AddModelToTask("transControllerTask" + str(spacecraftIndex), self.rwMotorTorque, 6) # Torque cmd to actuation mapping
        SimBase.AddModelToTask("transControllerTask" + str(spacecraftIndex), self.thrForceMapping, 6) # Force cmd to actuation mapping
        
        SimBase.AddModelToTask("transControllerTask" + str(spacecraftIndex), self.thrustOnTimeFiring, 6) # Force cmd to actuation mapping


        # ------------------ <START> PEND REMOVE ------------------
        
        # SimBase.AddModelToTask("sunPointTask" + str(spacecraftIndex), self.sunPoint, 10)
        # SimBase.AddModelToTask("locPointTask" + str(spacecraftIndex), self.locPoint, 10)
        # SimBase.AddModelToTask("spacecraftReconfigTask" + str(spacecraftIndex), self.spacecraftReconfig, 10)

        # SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.mrpFeedbackRWs, 7)
        # SimBase.AddModelToTask("mrpFeedbackRWsTask" + str(spacecraftIndex), self.rwMotorTorque, 6)

        # ------------------ <END> PEND REMOVE ------------------



        # Create events to be called for triggering GN&C maneuvers
        SimBase.fswProc[spacecraftIndex].disableAllTasks()

        # The standby event should not be active while the station keeping mode is also active. Standby mode disables
        # attitude control and therefore the attitude cannot be corrected for orbital correction burns.
        SimBase.createNewEvent("initiateStandby_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'standby'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.setAllButCurrentEventActivity('initiateStandby_" + str(spacecraftIndex) +
                                "', True, useIndex=True)"])

        SimBase.createNewEvent("initiateInertialPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
                               ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'inertialPointing'"],
                               ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
                                "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
                                "self.enableTask('inertialPointTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
                                "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
                                "self.setAllButCurrentEventActivity('initiateInertialPointing_" + str(spacecraftIndex) +
                                "', True, useIndex=True)"])

        # SimBase.createNewEvent("initiateSunPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
        #                        ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'sunPointing'"],
        #                        ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
        #                         "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
        #                         "self.enableTask('sunPointTask" + str(spacecraftIndex) + "')",
        #                         "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
        #                         "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
        #                         "self.setAllButCurrentEventActivity('initiateSunPointing_" + str(spacecraftIndex) +
        #                         "', True, useIndex=True)"])

        # SimBase.createNewEvent("initiateLocationPointing_" + str(spacecraftIndex), self.processTasksTimeStep, True,
        #                        ["self.FSWModels[" + str(spacecraftIndex) + "].modeRequest == 'locationPointing'"],
        #                        ["self.fswProc[" + str(spacecraftIndex) + "].disableAllTasks()",
        #                         "self.FSWModels[" + str(spacecraftIndex) + "].zeroGateWayMsgs()",
        #                         "self.enableTask('locPointTask" + str(spacecraftIndex) + "')",
        #                         "self.enableTask('trackingErrorTask" + str(spacecraftIndex) + "')",
        #                         "self.enableTask('mrpFeedbackRWsTask" + str(spacecraftIndex) + "')",
        #                         "self.setAllButCurrentEventActivity('initiateLocationPointing_" + str(spacecraftIndex) +
        #                         "', True, useIndex=True)"])

        # SimBase.createNewEvent("initiateStationKeeping_" + str(spacecraftIndex), self.processTasksTimeStep, True,
        #                        ["self.FSWModels[" + str(spacecraftIndex) + "].stationKeeping == 'ON'"],
        #                        ["self.enableTask('spacecraftReconfigTask" + str(spacecraftIndex) + "')",
        #                         "self.setEventActivity('stopStationKeeping_" + str(spacecraftIndex) + "', True)"])
        # SimBase.createNewEvent("stopStationKeeping_" + str(spacecraftIndex), self.processTasksTimeStep, True,
        #                        ["self.FSWModels[" + str(spacecraftIndex) + "].stationKeeping == 'OFF'"],
        #                        ["self.disableTask('spacecraftReconfigTask" + str(spacecraftIndex) + "')",
        #                         "self.setEventActivity('initiateStationKeeping_" + str(spacecraftIndex) + "', True)"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetInertial3DPointGuidance(self): # inertial3D module gives the Att_Ref msg!
        """
        Defines the inertial pointing guidance module.
        """
        # Pend change -> this is hardcoded
        self.inertial3DPoint.sigma_R0N = [0.0, 0.0, 0.0] # No attitude difference for now, can define an attitude trajectory later for monitoring!
        # self.inertial3DPoint.sigma_R0N = [0.1, 0.2, -0.3]
        messaging.AttRefMsg_C_addAuthor(self.inertial3DPoint.attRefOutMsg, self.attRefMsg)

    def SetSunPointGuidance(self, SimBase):
        """
        Defines the Sun pointing guidance module.
        """
        self.sunPoint.pHat_B = SimBase.DynModels[self.spacecraftIndex].solarPanelAxis
        self.sunPoint.scAttInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.sunPoint.scTransInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.sunPoint.celBodyInMsg.subscribeTo(SimBase.EnvModel.ephemObject.ephemOutMsgs[SimBase.EnvModel.sun])
        messaging.AttRefMsg_C_addAuthor(self.sunPoint.attRefOutMsg, self.attRefMsg)

    def SetLocationPointGuidance(self, SimBase):
        """
        Defines the Earth location pointing guidance module.
        """
        self.locPoint.pHat_B = [1, 0, 0]
        self.locPoint.scAttInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.locPoint.scTransInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.locPoint.locationInMsg.subscribeTo(SimBase.EnvModel.groundStation.currentGroundStateOutMsg)
        messaging.AttRefMsg_C_addAuthor(self.locPoint.attRefOutMsg, self.attRefMsg)

    def SetSpacecraftOrbitReconfig(self, SimBase):
        """
        Defines the station keeping module.
        """
        self.spacecraftReconfig.deputyTransInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        self.spacecraftReconfig.attRefInMsg.subscribeTo(self.attRefMsg)
        self.spacecraftReconfig.thrustConfigInMsg.subscribeTo(self.fswThrusterConfigMsg)
        self.spacecraftReconfig.vehicleConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.spacecraftReconfig.mu = SimBase.EnvModel.mu  # [m^3/s^2]
        self.spacecraftReconfig.attControlTime = 400  # [s]
        messaging.AttRefMsg_C_addAuthor(self.spacecraftReconfig.attRefOutMsg, self.attRefMsg)

        # connect a blank chief message
        chiefData = messaging.NavTransMsgPayload()
        chiefMsg = messaging.NavTransMsg().write(chiefData)
        self.spacecraftReconfig.chiefTransInMsg.subscribeTo(chiefMsg)

    # Setup `transRefInMsg` via hardcoded manner for now. Notice that `transRefInMsg` is defined here as:
    # Delta distance (dr) between the target & chaser spacecraft! Refer to "dev/transError.py" Line 87 for details.
    def SetTransRefMsg(self):
        self.transRefInMsg.r_RN_N = [1.0, 0.0, 0.0] # Random hardcoded delta distance
        self.transRefInMsg.v_RN_N = [0.0, 0.0, 0.0] # Random hardcoded delta velocity
        # messaging.TransGuidMsg_C_addAuthor(self.{someModule.transRefOutMsg}, self.transRefInMsg) 
        # # this .transRefOutMsg has to be defined in a new module if we have it later!

    # New setup transError.py module:
    def SetTransError(self, SimBase):
        # Target trans nav:
        self.transError.targetTransInMsg.subscribeTo(
            SimBase.DynModels[self.targetSCIndex].simpleNavObject.transOutMsg) # Taking the Target index
        # Chaser trans nav:
        self.transError.chaserTransInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.transOutMsg)
        # Trans reference:
        self.transError.transRefInMsg.subscribeTo(self.transRefInMsg)
        # The following line might not be necessary for now due to the fact that we only have one single module to find the transError.
        messaging.TransGuidMsg_C_addAuthor(self.transError.transGuidOutMsg, self.transGuidOutMsg)
        # `self.transGuidOutMsg` has been set in function setupGatewayMsgs() with C addAuthor() 
        # -> Check why, probably because the original structure would switch between control modules (SpacecraftReconfig, Local Pointing, Sun Pointing etc.)
    
    def SetTransController(self, SimBase):
        self.decayTime = 50 # copy from MRP Feedback module
        self.xi = 0.9 # copy from MRP Feedback module only
        # TODO - Maybe set controller gains via a JSON file separately, or define via `scConfig.py`?
        self.transController.P_trans = 2 * SimBase.DynModels[self.spacecraftIndex].m_sc / self.decayTime
        self.transController.P_rot = 2 * np.max(SimBase.DynModels[self.spacecraftIndex].I_sc) / self.decayTime
        self.transController.K_trans = (self.transController.P_trans / self.xi) * \
                                    (self.transController.P_trans / self.xi) / SimBase.DynModels[self.spacecraftIndex].m_sc
        self.transController.K_rot = (self.transController.P_rot / self.xi) * \
                                    (self.transController.P_rot / self.xi) / np.max(
            SimBase.DynModels[self.spacecraftIndex].I_sc)
        
        # Unlike the `mrpFeedback.py` module, we first ignore the RW inertial effects and do not import the RW params in the EOM.
        self.transController.transGuidInMsg.subscribeTo(self.transGuidOutMsg)
        self.transController.attGuidInMsg.subscribeTo(self.attGuidMsg)
        self.transController.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
    
    def SetAttitudeTrackingError(self, SimBase):
        """
        Defines the module that converts a reference message into a guidance message.
        """
        self.trackingError.attNavInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].simpleNavObject.attOutMsg)
        self.trackingError.attRefInMsg.subscribeTo(self.attRefMsg) # We need to figure out how specify attRefMsg!
        messaging.AttGuidMsg_C_addAuthor(self.trackingError.attGuidOutMsg, self.attGuidMsg)
                
    def SetMRPFeedbackRWA(self, SimBase):
        """
        Defines the control properties.
        """
        self.decayTime = 50 # for MRP Feedback module only
        self.xi = 0.9 # for MRP Feedback module only
        self.mrpFeedbackRWs.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackRWs.P = 2 * np.max(SimBase.DynModels[self.spacecraftIndex].I_sc) / self.decayTime
        self.mrpFeedbackRWs.K = (self.mrpFeedbackRWs.P / self.xi) * \
                                    (self.mrpFeedbackRWs.P / self.xi) / np.max(
            SimBase.DynModels[self.spacecraftIndex].I_sc)
        self.mrpFeedbackRWs.integralLimit = 2. / self.mrpFeedbackRWs.Ki * 0.1

        self.mrpFeedbackRWs.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.mrpFeedbackRWs.rwSpeedsInMsg.subscribeTo(
            SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwSpeedOutMsg)
        self.mrpFeedbackRWs.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mrpFeedbackRWs.guidInMsg.subscribeTo(self.attGuidMsg)

    def SetRWConfigMsg(self, SimBase):
        """
        Imports the RWs configuration information.
        """
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
        # the same msg is used here for both spacecraft
        fswSetupRW.clearSetup() # Check if this would affect the simultation!
        self.fswRwConfigMsg = SimBase.DynModels[self.spacecraftIndex].rwFactory.getConfigMessage()

    def SetThrustersConfigMsg(self, SimBase):
        """
        Imports the thrusters configuration information.
        """
        fswSetupThrusters.clearSetup()
        for key, th in SimBase.DynModels[self.spacecraftIndex].thrusterFactory.thrusterList.items():
            loc_B_tmp = list(itertools.chain.from_iterable(th.thrLoc_B))
            dir_B_tmp = list(itertools.chain.from_iterable(th.thrDir_B))
            fswSetupThrusters.create(loc_B_tmp, dir_B_tmp, th.MaxThrust)
        self.fswThrusterConfigMsg = fswSetupThrusters.writeConfigMessage()

    def SetRWMotorTorque(self):
        """
        Defines the motor torque from the control law.
        """
        controlAxes_B = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]

        self.rwMotorTorque.controlAxes_B = controlAxes_B
        self.rwMotorTorque.vehControlInMsg.subscribeTo(self.transController.cmdTorqueOutMsg)
        # self.rwMotorTorque.vehControlInMsg.subscribeTo(self.mrpFeedbackRWs.cmdTorqueOutMsg)
        self.rwMotorTorque.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
    
    def SetThrForceMapping(self, SimBase):
        # We should use forceTorqueThrForceMapping() for testing. 
        # Check how from "src/fswAlgorithms/effectorInterfaces/forceTorqueThrForceMapping/_UnitTest/test_forceTorqueThrForceMapping.py" 
        # AND https://hanspeterschaub.info/basilisk/Documentation/fswAlgorithms/effectorInterfaces/forceTorqueThrForceMapping/forceTorqueThrForceMapping.html?highlight=thrarraycmdforce
        
        # Need to replace the following from `thrForceMapping` ...
        # controlAxes_B = [1, 0, 0,
        #                  0, 1, 0,
        #                  0, 0, 1] # Thrust control axis
        # self.thrForceMapping.thrForceSign = +1
        # self.thrForceMapping.controlAxes_B = controlAxes_B
        self.thrForceMapping.thrConfigInMsg.subscribeTo(self.fswThrusterConfigMsg) # Subscribe to FSW Thruster Config!
        self.thrForceMapping.vehConfigInMsg.subscribeTo(SimBase.DynModels[self.spacecraftIndex].simpleMassPropsObject.vehicleConfigOutMsg)
        self.thrForceMapping.cmdForceInMsg.subscribeTo(self.transController.cmdForceOutMsg)
        
    def SetThrustOntimeFiring(self, SimBase):
        self.thrustOnTimeFiring.thrMinFireTime = 0.002
        self.thrustOnTimeFiring.level_on = .75
        self.thrustOnTimeFiring.level_off = .25
        
        # if useDVThrusters:
        #     thrFiringSchmittObj.baseThrustState = 1
        
        self.thrustOnTimeFiring.thrConfInMsg.subscribeTo(self.fswThrusterConfigMsg)
        self.thrustOnTimeFiring.thrForceInMsg.subscribeTo(self.thrForceMapping.thrForceCmdOutMsg)
        
        # Below has been moved to `setupGatewayMsgs()` to connect thrusterDynamicEffector to the thrustOnTimeFiring module (`forceTorqueThrForceMapping`) 
        # # Also connect this to thrusterDynamicEffector from Dynamic model:
        # SimBase.DynModels[self.spacecraftIndex].thrusterDynamicEffector.cmdsInMsg.subscribeTo(self.thrustOnTimeFiring.onTimeOutMsg)

    
    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        """
        Initializes all FSW objects.
        """
        self.SetInertial3DPointGuidance()
        self.SetTransRefMsg() # Translational dr dv setting (hardcoded for now)
        # self.SetSunPointGuidance(SimBase)
        # self.SetLocationPointGuidance(SimBase)
        self.SetAttitudeTrackingError(SimBase)
        self.SetRWConfigMsg(SimBase)
        self.SetThrustersConfigMsg(SimBase)
        # Added translational controller modules:
        self.SetTransError(SimBase)
        self.SetTransController(SimBase)
        # self.SetMRPFeedbackRWA(SimBase)
        # self.SetSpacecraftOrbitReconfig(SimBase)
        self.SetRWMotorTorque()
        self.SetThrForceMapping(SimBase)
        self.SetThrustOntimeFiring(SimBase)

    def setupGatewayMsgs(self, SimBase):
        """create C-wrapped gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        self.attRefMsg = messaging.AttRefMsg_C() # We need to set this!
        self.attGuidMsg = messaging.AttGuidMsg_C()
        # Add translational message first, need to figure out what is "Gateway Messages"
        self.transRefInMsg = messaging.TransRefMsg_C()
        self.transGuidOutMsg = messaging.TransGuidMsg_C()
        
        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels[self.spacecraftIndex].rwStateEffector.rwMotorCmdInMsg.subscribeTo(
            self.rwMotorTorque.rwMotorTorqueOutMsg)
        
        # Also connect this to thrusterDynamicEffector from Dynamic model:
        SimBase.DynModels[self.spacecraftIndex].thrusterDynamicEffector.cmdsInMsg.subscribeTo(
            self.thrustOnTimeFiring.onTimeOutMsg)
        # # Code stop working here -> need implementation on Thrust-Force-to-On-Time:
        # SimBase.DynModels[self.spacecraftIndex].thrusterDynamicEffector.cmdsInMsg.subscribeTo(
        #     self.spacecraftReconfig.onTimeOutMsg) # Need to change spacecraftReconfig related here to sth like thrForceMapping

    def zeroGateWayMsgs(self):
        """Zero all the FSW gateway message payloads"""
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        # Add to "Zero out" translational message first, need to figure out what is "Gateway Messages"
        self.transRefInMsg.write(messaging.TransRefMsgPayload())
        self.transGuidOutMsg.write(messaging.TransGuidMsgPayload())

    @property
    def inertial3DPointData(self):
        return self.inertial3DPoint

    inertial3DPointData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to inertial3DPointData as inertial3DPoint",
        inertial3DPointData)

    @property
    def inertial3DPointWrap(self):
        return self.inertial3DPoint

    inertial3DPointWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to inertial3DPointWrap as inertial3DPoint",
        inertial3DPointWrap)


    @property
    def sunPointData(self):
        return self.sunPoint

    sunPointData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to sunPointData as sunPoint",
        sunPointData)

    @property
    def sunPointWrap(self):
        return self.sunPoint

    sunPointWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to sunPointWrap as sunPoint",
        sunPointWrap)


    @property
    def locPointData(self):
        return self.locPoint

    locPointData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to locPointData as locPoint",
        locPointData)

    @property
    def locPointWrap(self):
        return self.locPoint

    locPointWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to locPointWrap as locPoint",
        locPointWrap)


    @property
    def spacecraftReconfigData(self):
        return self.spacecraftReconfig

    spacecraftReconfigData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to spacecraftReconfigData as spacecraftReconfig",
        spacecraftReconfigData)

    @property
    def spacecraftReconfigWrap(self):
        return self.spacecraftReconfig

    spacecraftReconfigWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to spacecraftReconfigWrap as spacecraftReconfig",
        spacecraftReconfigWrap)


    @property
    def trackingErrorData(self):
        return self.trackingError

    trackingErrorData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to trackingErrorData as trackingError",
        trackingErrorData)

    @property
    def trackingErrorWrap(self):
        return self.trackingError

    trackingErrorWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to trackingErrorWrap as trackingError",
        trackingErrorWrap)


    @property
    def mrpFeedbackRWsData(self):
        return self.mrpFeedbackRWs

    mrpFeedbackRWsData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackRWsData as mrpFeedbackRWs",
        mrpFeedbackRWsData)

    @property
    def mrpFeedbackRWsWrap(self):
        return self.mrpFeedbackRWs

    mrpFeedbackRWsWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to mrpFeedbackRWsWrap as mrpFeedbackRWs",
        mrpFeedbackRWsWrap)


    @property
    def rwMotorTorqueData(self):
        return self.rwMotorTorque

    rwMotorTorqueData = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rwMotorTorqueData as rwMotorTorque",
        rwMotorTorqueData)

    @property
    def rwMotorTorqueWrap(self):
        return self.rwMotorTorque

    rwMotorTorqueWrap = deprecated.DeprecatedProperty(
        "2024/07/30",
        "Due to the new C module syntax, refer to rwMotorTorqueWrap as rwMotorTorque",
        rwMotorTorqueWrap)
    