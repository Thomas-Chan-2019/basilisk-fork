from Basilisk.architecture import sysModel, messaging, bskLogging
import numpy as np

# This controller should have I/O messages from:
# Input: VehicleConfigMsgPayload, AttGuidMsgReader (if control att.), 
class transError(sysModel.SysModel):
    """
    This class inherits from the `SysModel` available in the ``Basilisk.architecture.sysModel`` module.
    The `SysModel` is the parent class which your Python BSK modules must inherit.
    The class uses the following
    virtual functions:

    #. ``Reset``: The method that will initialize any persistent data in your model to a common
       "ready to run" state (e.g. filter states, integral control sums, etc).
    #. ``UpdateState``: The method that will be called at the rate specified
       in the PythonTask that was created in the input file.

    Additionally, your class should ensure that in the ``__init__`` method, your call the super
    ``__init__`` method for the class so that the base class' constructor also gets called:

    .. code-block:: python

        super(PIDController, self).__init__()

    You class must implement the above four functions. Beyond these four functions you class
    can complete any other computations you need (``Numpy``, ``matplotlib``, vision processing
    AI, whatever).
    """
    def __init__(self):
        super(transError, self).__init__()

        # Input translational structure message from SCState
        self.targetTransInMsg = messaging.NavTransMsgReader()
        self.chaserTransInMsg = messaging.NavTransMsgReader()
        self.transRefInMsg = messaging.TransRefMsgReader()
        
        # Output body torque message name
        self.transGuidOutMsg = messaging.TransGuidMsg_C() # Dirty fix to allow C_addAuthor to work. Need to review!
        # self.transGuidOutMsg = messaging.TransGuidMsg()
        

    def Reset(self, CurrentSimNanos):
        """
        The Reset method is used to clear out any persistent variables that need to get changed
        when a task is restarted.  This method is typically only called once after selfInit/crossInit,
        but it should be written to allow the user to call it multiple times if necessary.
        :param CurrentSimNanos: current simulation time in nano-seconds
        :return: none
        """
        # This is in accordance to attTrackingError.c ("src/fswAlgorithms/attGuidance/attTrackingError/attTrackingError.c")
        logger = bskLogging.BSKLogger() # Pend remove if self.bskLogger works!
        if not self.targetTransInMsg.isLinked():
            # bskLogging._bskLog(logger, bskLogging.BSK_ERROR, "Error: transError.targetTransInMsg wasn't connected.")
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: transError.targetTransInMsg wasn't connected.")
        if not self.chaserTransInMsg.isLinked():
            # bskLogging._bskLog(logger, bskLogging.BSK_ERROR, "Error: transError.chaserTransInMsg wasn't connected.")
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: transError.chaserTransInMsg wasn't connected.")
        if not self.transRefInMsg.isLinked():
            # bskLogging._bskLog(logger, bskLogging.BSK_ERROR, "Error: transError.transRefInMsg wasn't connected.")
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: transError.transRefInMsg wasn't connected.")

        
        return

    def UpdateState(self, CurrentSimNanos):
        """
        The updateState method is the cyclical worker method for a given Basilisk class.  It
        will get called periodically at the rate specified in the task that the model is
        attached to.  It persists and anything can be done inside of it.  If you have realtime
        requirements though, be careful about how much processing you put into a Python UpdateState
        method.  You could easily detonate your sim's ability to run in realtime.

        :param CurrentSimNanos: current simulation time in nano-seconds
        :return: none
        """
        # read input message
        targetTransInMsgBuffer = self.targetTransInMsg()
        chaserTransInMsgBuffer = self.chaserTransInMsg()
        transRefInMsgBuffer = self.transRefInMsg()

        # create output message buffer
        # Need a new message type?
        transGuidOutMsgBuffer = messaging.TransGuidMsgPayload()


        # Simple subtraction for error:
        transErr1 = np.array(targetTransInMsgBuffer.r_BN_N) - np.array(chaserTransInMsgBuffer.r_BN_N)
        transErr2 = transErr1 - np.array(transRefInMsgBuffer.r_RN_N)
        
        # Also log velocity error in case we need it
        vErr1 = np.array(targetTransInMsgBuffer.v_BN_N) - np.array(chaserTransInMsgBuffer.v_BN_N)
        vErr2 = vErr1 - np.array(transRefInMsgBuffer.v_RN_N)
        
        transGuidOutMsgBuffer.r_BR_B = transErr2.tolist()
        transGuidOutMsgBuffer.v_BR_B = vErr2.tolist()
        
        self.transGuidOutMsg.write(transGuidOutMsgBuffer, CurrentSimNanos, self.moduleID)
        
        # # compute control solution
        # lrCmd = np.array(guidMsgBuffer.sigma_BR) * self.K + np.array(guidMsgBuffer.omega_BR_B) * self.P
        # torqueOutMsgBuffer.torqueRequestBody = (-lrCmd).tolist()

        # self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, CurrentSimNanos, self.moduleID)

        # # All Python SysModels have self.bskLogger available
        # # The logger level flags (i.e. BSK_INFORMATION) may be
        # # accessed from sysModel
        # if False:
        #     """Sample Python module method"""
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Time: {CurrentSimNanos * 1.0E-9} s")
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"targetTransInMsgBuffer.r_BN_N: {targetTransInMsgBuffer.r_BN_N}")
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"targetTransInMsgBuffer.v_BN_N: {targetTransInMsgBuffer.v_BN_N}")
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"chaserTransInMsgBuffer.r_BN_N: {chaserTransInMsgBuffer.r_BN_N}")
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"chaserTransInMsgBuffer.v_BN_N: {chaserTransInMsgBuffer.v_BN_N}")
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"transGuidOutMsgBuffer.r_BR_B: {transGuidOutMsgBuffer.r_BR_B}")
        self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"transGuidOutMsgBuffer.v_BR_B: {transGuidOutMsgBuffer.v_BR_B}")
    
        return