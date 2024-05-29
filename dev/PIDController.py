from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import orbitalMotion, unitTestSupport as UAT, RigidBodyKinematics as RBK # Import for Hill-frame conversion & control if necessary
import numpy as np

# See https://hanspeterschaub.info/basilisk/Learn/makingModules/pyModules.html for Python Module creation original example.
# This controller should have I/O messages from:
# Input: VehicleConfigMsgPayload, AttGuidMsgReader (if control att.), TODO
# Unlike the `mrpFeedback.py` module, we first ignore the RW inertial effects and do not import the RW params in the EOM.
class PIDController(sysModel.SysModel):
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
        super(PIDController, self).__init__()

        # Translational Gains Initialization:
        self.Kp_trans = np.array([[0,0,0],
                                 [0,0,0],
                                 [0,0,0]]
                                ) # Proportional gain
        self.Kd_trans = np.array([[0,0,0],
                                 [0,0,0],
                                 [0,0,0]]
                                ) # Derivative gain
        
        # Rotational Gains Initialization:
        self.K_rot = 0 # Proportional gain
        self.P_rot = 0 # Derivative gain
        
        # Input guidance structure message: Translational
        self.transGuidInMsg = messaging.TransGuidMsgReader()
        # self.transGuidInMsg = messaging.TransGuidMsg_C()
        # Input guidance structure message: Rotational
        self.attGuidInMsg = messaging.AttGuidMsgReader()
        # For mass & moment of inertia
        self.vehConfigInMsg = messaging.VehicleConfigMsgReader()
        
        # # Include thruster & RW arrays configs:
        # self.thrParamsInMsg = messaging.THRArrayConfigMsg()
        # self.rwParamsInMsg = messaging.RWArrayConfigMsg()
        
        # Output body thrust & force message name
        self.cmdForceOutMsg = messaging.CmdForceBodyMsg() # 
        # self.cmdForceInMsg = messaging.CmdForceInertialMsg # to verify
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        
        # # Input guidance structure message
        # self.guidInMsg = messaging.AttGuidMsgReader()
        # # Output body torque message name
        # self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()

    def Reset(self, CurrentSimNanos):
        # # Proportional gain term used in control
        # self.Kp_trans = 0
        # self.K_rot = 0
        # # Derivative gain term used in control
        # self.Kd_trans = 0
        # self.P_rot = 0
        
        # TODO: Add back Reset() actions in accordance to mrpFeedback.c, basically:
        # 1) Message subscription check -> throw BSK log error if not linked;
        if not self.transGuidInMsg.isLinked():
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: transController.transGuidInMsg wasn't connected.")
        if not self.attGuidInMsg.isLinked():
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: transController.attGuidInMsg wasn't connected.")
        if not self.vehConfigInMsg.isLinked():
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: transController.vehConfigInMsg wasn't connected.")
            
        # 2) Read `VehicleConfigMsgPayload` for I_sc (also m_SC);
        # 3) Reset control gains or constraints for MPC controllers when implemented.
        
        """
        The Reset method is used to clear out any persistent variables that need to get changed
        when a task is restarted.  This method is typically only called once after selfInit/crossInit,
        but it should be written to allow the user to call it multiple times if necessary.
        :param CurrentSimNanos: current simulation time in nano-seconds
        :return: none
        """
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
        transGuidMsgBuffer = self.transGuidInMsg()
        attGuidMsgBuffer = self.attGuidInMsg()
        vehConfigMsgBuffer = self.vehConfigInMsg()

        # create output message buffer
        forceOutMsgBuffer = messaging.CmdForceBodyMsgPayload()
        torqueOutMsgBuffer = messaging.CmdTorqueBodyMsgPayload()
        
        # Do we need Hill-frame based controller?
        # If so -> use `orbitalMotion.rv2hill()` and `.hill2rv()`
        # `rv2hill(rc_N, vc_N, rd_N, vd_N)`
        # `hill2rv(rc_N, vc_N, rho_H, rhoPrime_H)`
        
        # compute TRANS control solution
        FrCmd = self.Kp_trans @ np.array(transGuidMsgBuffer.r_BR_B) + self.Kd_trans @ np.array(transGuidMsgBuffer.v_BR_B)
        # FrCmd = np.array(transGuidMsgBuffer.r_BR_B) * self.Kd_trans + np.array(transGuidMsgBuffer.v_BR_B) * self.Kp_trans
        forceOutMsgBuffer.forceRequestBody = (-FrCmd).tolist()
        self.cmdForceOutMsg.write(forceOutMsgBuffer, CurrentSimNanos, self.moduleID)

        # compute ATT control solution
        omega_BR_B_Tilde = RBK.v3Tilde(attGuidMsgBuffer.omega_BR_B)
        # omega_BR_B_Tilde = np.array(omega_BR_B_Tilde) # this is unnecessary!
        Isc = np.array(vehConfigMsgBuffer.ISCPntB_B).reshape(3,3) # From vehConfig ISCPntB_B[9] to Isc[3][3] np array.
        wTilde_I_w = omega_BR_B_Tilde @ Isc @ np.array(attGuidMsgBuffer.omega_BR_B).reshape(3,1) # [omegaTilde]*[I]*omega term; reshaping of omega[3] to a np column vector for matrix multiplication (@ operator). 
        lrCmd = np.array(attGuidMsgBuffer.sigma_BR) * self.K_rot + np.array(attGuidMsgBuffer.omega_BR_B) * self.P_rot + wTilde_I_w.reshape(1,3).squeeze()
        # lrCmd = np.array(attGuidMsgBuffer.sigma_BR) * self.K_rot + np.array(attGuidMsgBuffer.omega_BR_B) * self.P_rot
        # To add reference trajectory related terms (e.g. omega_r, sigma_r) when needed:
        # PD Form:
        # u = -[K]*sigma - [P]*(omega - omega_r) + [I]*(omegaDot_r - [omegaTilde]*omega_r) + [omegaTilde]*[I]*omega - L
        torqueOutMsgBuffer.torqueRequestBody = (-lrCmd).tolist()
        self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, CurrentSimNanos, self.moduleID)

        # All Python SysModels have self.bskLogger available
        # The logger level flags (i.e. BSK_INFORMATION) may be
        # accessed from sysModel
        if 1:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"------ TransController Module ------")
            """Sample Python module method"""
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Time: {CurrentSimNanos * 1.0E-9} s")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"transGuidMsgBuffer.r_BR_B: {transGuidMsgBuffer.r_BR_B}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"transGuidMsgBuffer.v_BR_B: {transGuidMsgBuffer.v_BR_B}")
            
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"ForceRequestBody: {forceOutMsgBuffer.forceRequestBody}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"TorqueRequestBody: {torqueOutMsgBuffer.torqueRequestBody}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Written Msg - ForceRequestBody: {self.cmdForceOutMsg.read().forceRequestBody}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Written Msg - TorqueRequestBody: {self.cmdTorqueOutMsg.read().torqueRequestBody}")
            
        return