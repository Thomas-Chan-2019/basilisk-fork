from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import orbitalMotion, RigidBodyKinematics as RBK # Import for Hill-frame conversion & control if necessary
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

        # Proportional gain term used in control
        self.K_trans = 0
        self.K_rot = 0
        # Derivative gain term used in control
        self.P_trans = 0
        self.P_rot = 0
        
        # Input guidance structure message: Translational
        self.transGuidInMsg = messaging.TransGuidMsg()
        # Input guidance structure message: Rotational
        self.attGuidInMsg = messaging.AttGuidMsg()
        # For mass & moment of inertia
        self.vehConfigInMsg = messaging.VehicleConfigMsg()
        
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
        # Proportional gain term used in control
        self.K_trans = 0
        self.K_rot = 0
        # Derivative gain term used in control
        self.P_trans = 0
        self.P_rot = 0
        
        # TODO: Add back Reset() actions in accordance to mrpFeedback.c, basically:
        # 1) Message subscription check -> throw BSK log error if not linked;
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

        # create output message buffer
        forceOutMsgBuffer = messaging.CmdForceBodyMsgPayload()
        torqueOutMsgBuffer = messaging.CmdTorqueBodyMsgPayload()
        
        # Do we need Hill-frame based controller?
        # If so -> use `orbitalMotion.rv2hill()` and `.hill2rv()`
        # `rv2hill(rc_N, vc_N, rd_N, vd_N)`
        # `hill2rv(rc_N, vc_N, rho_H, rhoPrime_H)`
        
        # compute TRANS control solution
        FrCmd = np.array(transGuidMsgBuffer.r_BR_B) * self.K_trans + np.array(transGuidMsgBuffer.v_BR_B) * self.P_trans
        forceOutMsgBuffer.forceRequestBody = (-FrCmd).tolist()
        self.cmdForceOutMsg.write(forceOutMsgBuffer, CurrentSimNanos, self.moduleID)

        # compute ATT control solution
        omega_BR_B_Tilde = RBK.v3Tilde(attGuidMsgBuffer.omega_BR_B)
        lrCmd = np.array(attGuidMsgBuffer.sigma_BR) * self.K_rot + np.array(attGuidMsgBuffer.omega_BR_B) * self.P_rot + np.array(omega_BR_B_Tilde) * np.array(self.vehConfigInMsg.ISCPntB_B) * np.array(attGuidMsgBuffer.omega_BR_B)
        # To add reference trajectory related terms (e.g. omega_r, sigma_r) when needed:
        # PD Form:
        # u = -[K]*sigma - [P]*(omega - omega_r) + [I]*(omegaDot_r - [omegaTilde]*omega_r) + [omegaTilde]*[I]*omega - L
        torqueOutMsgBuffer.torqueRequestBody = (-lrCmd).tolist()
        self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, CurrentSimNanos, self.moduleID)

        # All Python SysModels have self.bskLogger available
        # The logger level flags (i.e. BSK_INFORMATION) may be
        # accessed from sysModel
        if False:
            """Sample Python module method"""
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Time: {CurrentSimNanos * 1.0E-9} s")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"TorqueRequestBody: {torqueOutMsgBuffer.torqueRequestBody}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"sigma_BR: {guidMsgBuffer.sigma_BR}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"omega_BR_B: {guidMsgBuffer.omega_BR_B}")

        return