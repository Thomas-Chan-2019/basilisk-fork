from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import orbitalMotion, unitTestSupport as UAT, RigidBodyKinematics as RBK # Import for Hill-frame conversion & control if necessary
import numpy as np
import math, copy
from scipy.signal import place_poles

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
    def __init__(self, a_orbit, controllerType='pole-place'):
        super(PIDController, self).__init__()

        # Translational Gains Initialization 
        self.a_orbit = a_orbit
        self.controllerType = controllerType
        
        # Set gain
        Kp_trans, Kd_trans = self.__SetTransControllerGains()
        self.Kp_trans = Kp_trans # Proportional gain
        self.Kd_trans = Kd_trans # Derivative gain
        
        # self.Kp_trans = np.array([[0,0,0],
        #                          [0,0,0],
        #                          [0,0,0]]
        #                         ) # Proportional gain
        # self.Kd_trans = np.array([[0,0,0],
        #                          [0,0,0],
        #                          [0,0,0]]
        #                         ) # Derivative gain
        
        # Rotational Gains Initialization:
        self.K_rot = 0 # Proportional gain
        self.P_rot = 0 # Derivative gain
        
        ## INPUT:
        # Input guidance structure message: Translational
        self.transGuidInMsg = messaging.TransGuidMsgReader()
        # self.transGuidInMsg = messaging.TransGuidMsg_C()
        # Input guidance structure message: Rotational
        self.attGuidInMsg = messaging.AttGuidMsgReader()
        # For mass & moment of inertia
        self.vehConfigInMsg = messaging.VehicleConfigMsgReader()
        self.transNavInMsg = messaging.NavTransMsgReader()
        self.attNavInMsg = messaging.NavAttMsgReader()
        
        # Include thruster & RW arrays configs:
        # self.thrParamsInMsg = messaging.THRArrayConfigMsgReader()
        # self.rwParamsInMsg = messaging.RWArrayConfigMsg()
        
        ## OUTPUT:
        # Output body thrust & force message name
        self.cmdForceOutMsg = messaging.CmdForceBodyMsg() # 
        # self.cmdForceInMsg = messaging.CmdForceInertialMsg # to verify
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        
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
        
        if not self.transNavInMsg.isLinked():
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: transController.transNavInMsg wasn't connected.")
        if not self.attNavInMsg.isLinked():
            self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: transController.attNavInMsg wasn't connected.")
            
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
        transGuidMsgBuffer = self.transGuidInMsg() # transGuidMsg r,v are in Hill-frame.
        attGuidMsgBuffer = self.attGuidInMsg()
        vehConfigMsgBuffer = self.vehConfigInMsg()
        transInMsgBuffer = self.transNavInMsg()
        attNavInMsgBuffer = self.attNavInMsg()
        # create output message buffer
        forceOutMsgBuffer = messaging.CmdForceBodyMsgPayload()
        torqueOutMsgBuffer = messaging.CmdTorqueBodyMsgPayload()
        
        # Do we need Hill-frame based controller?
        # If so -> use `orbitalMotion.rv2hill()` and `.hill2rv()`
        # `rv2hill(rc_N, vc_N, rd_N, vd_N)`
        # `hill2rv(rc_N, vc_N, rho_H, rhoPrime_H)`
        
        # compute TRANS control solution
        rc_H = np.array(transGuidMsgBuffer.r_BR_B)
        vc_H = np.array(transGuidMsgBuffer.v_BR_B)
        FrCmd_hill = - self.Kp_trans @ rc_H - self.Kd_trans @ vc_H
        
        # Set small forces to zero for initial actuation noise filtering:
        eps = 1e-6
        FrCmd_hill[np.abs(FrCmd_hill) < eps] = 0.0
        
        # Convert the Hill-frame control force to Body frames
        r_BN_N = np.array(transInMsgBuffer.r_BN_N)
        v_BN_N = np.array(transInMsgBuffer.v_BN_N)
        # Retrive DCM from hill H frame to inertial N frame, notice the transpose.
        DCM_NH = orbitalMotion.hillFrame(r_BN_N, v_BN_N).transpose()
        # Retrive DCM from inertial N frame to body B frame via its own attitude from attNavInMsg of simpleNav module.
        DCM_BN = RBK.MRP2C(np.array(attNavInMsgBuffer.sigma_BN)) # .transpose()
        FrCmd_N = DCM_NH @ FrCmd_hill
        
        # Print log -> Pend remove:
        # print("Hill-frame Cmd Force: ", FrCmd_hill)
        # print("Inertial Cmd Force: ", FrCmd_N)
        # print("DCM Inertial from Hill: ", DCM_NH)
        # print("DCM Body from Inertial: ", DCM_BN)
        # Print log -> Pend remove:
        
        # Convert Hill-frame Cmd force to Inertial Cmd force:
        FrCmd = DCM_BN @ DCM_NH @ FrCmd_hill
        
        # Print log -> Pend remove:
        # print("Cmd Force: ", FrCmd)
        # Print log -> Pend remove:
        
        # forceOutMsgBuffer.forceRequestBody = (-FrCmd).tolist()
        forceOutMsgBuffer.forceRequestBody = (FrCmd).tolist() # Set the negative sign when using the module!!!
        self.cmdForceOutMsg.write(forceOutMsgBuffer, CurrentSimNanos, self.moduleID)

        # compute ATT control solution
        omega_BR_B_Tilde = RBK.v3Tilde(attGuidMsgBuffer.omega_BR_B)
        # omega_BR_B_Tilde = np.array(omega_BR_B_Tilde) # this is unnecessary!
        Isc = np.array(vehConfigMsgBuffer.ISCPntB_B).reshape(3,3) # From vehConfig ISCPntB_B[9] to Isc[3][3] np array.
        wTilde_I_w = omega_BR_B_Tilde @ Isc @ np.array(attGuidMsgBuffer.omega_BR_B).reshape(3,1) # [omegaTilde]*[I]*omega term; reshaping of omega[3] to a np column vector for matrix multiplication (@ operator). 
        lrCmd = - np.array(attGuidMsgBuffer.sigma_BR) * self.K_rot - np.array(attGuidMsgBuffer.omega_BR_B) * self.P_rot - wTilde_I_w.reshape(1,3).squeeze()
        # lrCmd = np.array(attGuidMsgBuffer.sigma_BR) * self.K_rot + np.array(attGuidMsgBuffer.omega_BR_B) * self.P_rot
        # To add reference trajectory related terms (e.g. omega_r, sigma_r) when needed:
        # PD Form:
        # u = -[K]*sigma - [P]*(omega - omega_r) + [I]*(omegaDot_r - [omegaTilde]*omega_r) + [omegaTilde]*[I]*omega - L
        torqueOutMsgBuffer.torqueRequestBody = (lrCmd).tolist()
        self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, CurrentSimNanos, self.moduleID)

        # All Python SysModels have self.bskLogger available
        # The logger level flags (i.e. BSK_INFORMATION) may be
        # accessed from sysModel
        if False:
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"------ TransController Module ------")
            """Sample Python module method"""
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Time: {CurrentSimNanos * 1.0E-9} s")
            # self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"transGuidMsgBuffer.r_BR_B: {transGuidMsgBuffer.r_BR_B}")
            # self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"transGuidMsgBuffer.v_BR_B: {transGuidMsgBuffer.v_BR_B}")
            
            # self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"ForceRequestBody: {forceOutMsgBuffer.forceRequestBody}")
            # self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"TorqueRequestBody: {torqueOutMsgBuffer.torqueRequestBody}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Written Msg - ForceRequestBody: {self.cmdForceOutMsg.read().forceRequestBody}")
            self.bskLogger.bskLog(sysModel.BSK_INFORMATION, f"Written Msg - TorqueRequestBody: {self.cmdTorqueOutMsg.read().torqueRequestBody}")
            
        return
    
    def __SetTransControllerGains(self): # Internal function to set controller gain here.
            # Controller PID tuning: 
            mu_Earth = orbitalMotion.MU_EARTH * math.pow(1000,3) # Convert to S.I.: m^3/s^2
            w = math.sqrt(mu_Earth / math.pow(self.a_orbit,3)) # Mean motion or Angular velocity
            
            # Closed-loop system of CW-equation:
            A = np.array([[0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1],
                        [0, 0, 0, 0, 2*w, 0],
                        [0, 3*math.pow(w,2), 0, -2*w, 0, 0],
                        [0, 0, -math.pow(w,2), 0, 0, 0]]) # 6x6 A-matrix
            B = np.array([[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]]) # 6x3 B-matrix
            
            # Controller Design:
            # 1) Linearization - Pole placement using scipy.signal.place_poles():
            # design_poles = np.array([-10.0, -10.0, -10.0, -1.0, -1.0, -1.0]) # Works but too damped
            # design_poles = np.array([-2.0, -2.1, -2.2, -0.5, -0.6, -0.7]) # Works
            # design_poles = np.array([-4.0, -4.1, -4.2, -1.0, -1.1, -1.2]) # Works
            # design_poles = np.array([-0.3, -0.1, -0.2, -0.5, -0.6, -0.7]) # TOO LOW... Trying to lower the damping!
            design_poles = np.array([-1.0, -1.1, -1.2, -0.5, -0.6, -0.7]) # Seems good enough with 600 Ns total impulse - Trying to lower the damping!
            
            # Try only replace the +ve pole:
            # design_poles = __replace_unstable_eigenvalues(A, num_unstable=1, replacement_values=[-1000.0])
            
            original_poles = [0, 0, w*1j, -w*1j, +w*math.sqrt(-4*w + 3), -w*math.sqrt(-4*w + 3)]
            # design_poles = [0, 0, w*1j, -w*1j, -1 +w*math.sqrt(-4*w + 3), -1 -w*math.sqrt(-4*w + 3)]
            design_poles_dyn = [-0.5, -0.5, -1-w*1j, -1+w*1j, -1 +w*math.sqrt(-4*w + 3), -1 -w*math.sqrt(-4*w + 3)] # Works with 600 Ns total impulse
            
            if self.controllerType == "pole-place":
                polePlaceResult = place_poles(A, B, design_poles)
                K = polePlaceResult.gain_matrix
                print(K)
                print(A - B@K)
                
                Kp_trans = K[:, :3] # retrieve first 3 columns of K as Kp
                Kd_trans = K[:, 3:] # retrieve last 3 columns of K as Kd
            
            # 1) Pole placement with SRL config -> No z-axis actuation:
            elif self.controllerType == "SRL":
                design_poles_SRL = np.array([-0.1, -0.05, -0.08, -0.03, -0.05, -0.5])
                # polePlaceResult = place_poles(A, B, design_poles)
                polePlaceResult = place_poles(A, B, design_poles_SRL)
                K = polePlaceResult.gain_matrix
                print(K)
                print(A - B@K)
                
                Kp_trans = K[:, :3] # retrieve first 3 columns of K as Kp
                Kd_trans = K[:, 3:] # retrieve last 3 columns of K as Kd

                # ku = 1/1 * np.array([[2,0,0], [0,2,0], [0,0,0]]).transpose() # Remove z-axis actuation

                # Kp_trans = 0.60 * ku
                # Ki_trans = 0.1* 1.2 * ku / 18
                # Kd_trans = 5*0.075 * ku * 18
                Kp_trans[-1,:] = [0, 0, 0]
                Kd_trans[-1,:] = [0, 0, 0]
            
            # 2) Feedback Linearization + Pole Placement: # Reuse A, B matrices and poles from 1).
            elif self.controllerType == "feedback-lin":
                # i) Feedback linearization PHI-term to remove non-linearity 
                phi_p = A[3:,:3] # position nonlinearity - p
                phi_d = A[3:,3:] # derivative nonlinearity - d
                # Equivalent to below:
                # phi_x_p = np.array[[0,0,0],
                #                    [0,3*math.pow(w,2),0],
                #                    [0,0,-math.pow(w,2)]] 
                # phi_x_d = np.array([[0,2*w,0],
                #                     [-2*w,0,0],
                #                     [0,0,0]]) 
                
                # (Below is equivalent to set the corresponding part in A equal to zero, but this is kept for clear understanding.)
                A_tilde = copy.deepcopy(A)
                A_tilde[3:,:3] = A[3:,:3] - phi_p # Remove nonlinearity phi_p
                A_tilde[3:,3:] = A[3:,3:] - phi_d # Remove nonlinearity phi_d
                
                # ii) Set gain matrices through pole placement:
                polePlaceResult = place_poles(A_tilde, B, design_poles)
                K_temp = polePlaceResult.gain_matrix
                print(K_temp)
                print(A_tilde - B@K_temp)
                
                Kp_trans = phi_p + K_temp[:, :3] # retrieve first 3 columns of K as Kp
                Kd_trans = phi_d + K_temp[:, 3:] # retrieve last 3 columns of K as Kd
                # Kp_trans = np.array([[0,0,0],
                #                      [0,-3*math.pow(w,2),0],
                #                      [0,0,math.pow(w,2)]]) + \
                #            np.array([[-k1,0,0],
                #                      [0,-k2,0],
                #                      [0,0,-k3]]) 
                # Kd_trans = np.array([[-1,-2*w,0],
                #                      [2*w,-1,0],
                #                      [0,0,-1]]) + \
                #            np.array([[-k4,0,0],
                #                      [0,-k5,0],
                #                      [0,0,-k6]])
            else:
                self.bskLogger.bskLog(sysModel.BSK_ERROR, f"Error: Wrong Controller Gain selection type. Please check Controller Gain implementation.")
            
            return Kp_trans, Kd_trans
        
    def __replace_unstable_eigenvalues(A, num_unstable, replacement_values):
            """ 
            Replace unstable eigenvalues (those with positive real parts) 
            with new desired stable values.
            """
            eigenvalues, _ = np.linalg.eig(A)
            # Identify unstable eigenvalues (positive real part)
            unstable_indices = np.where(np.real(eigenvalues) > 0)[0]
            
            # If the number of unstable eigenvalues is not as expected, raise an error
            if len(unstable_indices) != num_unstable:
                raise ValueError(f"Expected {num_unstable} unstable eigenvalues, but found {len(unstable_indices)}")
            
            # Replace the unstable eigenvalues with the provided stable values
            for i, idx in enumerate(unstable_indices):
                eigenvalues[idx] = replacement_values[i]
            
            return eigenvalues