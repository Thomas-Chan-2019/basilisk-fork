/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */


#ifndef LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H
#define LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/ArrayMotorForceMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/TranslatingRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

/*! @brief linear spring mass damper state effector class */
class linearTranslationOneDOFStateEffector :
	public StateEffector, public SysModel
{
public:
// MESSAGING
    Message<TranslatingRigidBodyMsgPayload> translatingBodyOutMsg;        //!< state output message
    Message<SCStatesMsgPayload> translatingBodyConfigLogOutMsg;           //!< translating body state config log message
    ReadFunctor<ArrayMotorForceMsgPayload> motorForceInMsg;               //!< -- (optional) motor force input message
    ReadFunctor<TranslatingRigidBodyMsgPayload> translatingBodyRefInMsg;  //!< -- (optional) reference state input message
    ReadFunctor<ArrayEffectorLockMsgPayload> LockInMsg;                   //!< -- (optional) lock flag input message

// Constructor/Destructor and setter
    linearTranslationOneDOFStateEffector();           //!< -- Contructor
	~linearTranslationOneDOFStateEffector();          //!< -- Destructor
    void linearTranslationOneDOFStateEffector::setParameters(double mass, double k, double c, double rhoInit, double rhoDotInit, Eigen::Vector3d pHat_B, Eigen::Vector3d r_PcP_P, Eigen::Vector3d r_P0B_B, Eigen::Matrix3d IPntPc_P, Eigen::Matrix3d dcm_PB); //!< -- set user defined parameters
private:
// Scalar parameters set by user
    double mass = 1.0;              //!< [kg] mass of effector
    double k = 0;                   //!< [N/m] linear spring constant
    double c = 0;                   //!< [N-s/m] linear damping term
    double rhoInit = 0;             //!< [m] Initial value for particle offset
    double rhoDotInit = 0;          //!< [m/s] Initial value for particle offset derivative
//

// Vector/Matrix parameters set by user
	Eigen::Vector3d pHat_B;         //!< -- axis of translation in B frame components.
    Eigen::Vector3d r_PcP_P;        //!< [m] vector pointing from location P0 along pHat to P_C in p frame components
    Eigen::Vector3d r_P0B_B;        //!< [m] vector pointing from body frame B origin to point p0 origin of p frame in B frame components
    Eigen::Matrix3d IPntPc_P;       //!< [kg-m^2] Inertia of pc about point p in p frame component
    Eigen::Matrix3d dcm_PB;         //!< -- DCM from the p frame to the body frame
//

    std::string nameOfRhoState;     //!< [-] Identifier for the rho state data container
    std::string nameOfRhoDotState;  //!< [-] Identifier for the rhoDot state data container

    int lockFlag = 0;                   //!< [] flag for locking the translation axis
    double cRho;                        //!< -- Term needed for back-sub method

    double rho;                         //!< [m] displacement from equilibrium
    double rhoDot;                      //!< [m/s] time derivative of displacement from equilibrium
    double rhoRef = 0.0;                //!< [m] translating body reference position
    double rhoDotRef = 0.0;             //!< [m/s] translating body reference velocity
    double motorForce = 0.0;            //!< [N] optional motor force
    Eigen::Vector3d r_PcB_B;            //!< [m] position vector from B to center of mass location of effector
    Eigen::Vector3d r_PcP0_B;           //!< [m] vector pointing from point p0 origin of p frame to center of mass location of effector in B frame components
    Eigen::Matrix3d rTilde_PcP_B;       //!< [m] tilde matrix of r_PcP_B
	Eigen::Vector3d rPrime_PcP_B;       //!< [m/s] Body time derivative of r_PcP_B
	Eigen::Matrix3d rPrimeTilde_PcP_B;  //!< [m/s] Tilde matrix of rPrime_PcP_B
	Eigen::Matrix3d rTilde_PcB_B;       //!< [m] tilde matrix of r_PcB_B
	Eigen::Vector3d rPrime_PcB_B;       //!< [m/s] Body time derivative of r_PcB_B
	Eigen::Matrix3d rPrimeTilde_PcB_B;  //!< [m/s] Tilde matrix of rPrime_PcB_B
	Eigen::Matrix3d IPntPc_B;           //!< [kg-m^2] Inertia of Pc about point B in B frame components
	Eigen::Matrix3d dcm_BN;             //!< -- DCM from the B frame to the N frame
    Eigen::Vector3d omega_BN_B;         //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components.
    Eigen::Matrix3d omegaTilde_BN_B;    //!< [rad/s] tilde matrix of omega_BN_B
    Eigen::MatrixXd* inertialPositionProperty = nullptr;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase

    Eigen::Vector3d aRho;          //!< -- Term needed for back-sub method
    Eigen::Vector3d bRho;          //!< -- Term needed for back-sub method

    Eigen::MatrixXd *g_N;          //!< [m/s^2] Gravitational acceleration in N frame components
	StateData *rhoState;		   //!< -- state data for displacement from equilibrium
    Eigen::MatrixXd *c_B;          //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;     //!< [m/s] Body time derivative of vector c_B in B frame components
	StateData *rhoDotState;		   //!< -- state data for time derivative of rho;
	StateData *omegaState;         //!< -- state data for the hubs omega_BN_B
	StateData *sigmaState;         //!< -- state data for the hubs sigma_BN
    static uint64_t effectorID;    //!< [] ID number of this panel


    //COMMENTS
    // Translating body properties
    Eigen::Vector3d r_PcN_N;            //!< [m] position vector of translating body's center of mass Sc relative to the inertial frame origin N
    Eigen::Vector3d v_PcN_N;            //!< [m/s] inertial velocity vector of Sc relative to inertial frame
    Eigen::Vector3d sigma_PN;           //!< -- MRP attitude of frame S relative to inertial frame
    Eigen::Vector3d omega_PN_P;         //!< [rad/s] inertial translating body frame angular velocity vector

    // functions
	void registerStates(DynParamManager& states);     //!< -- Method for SMD to register its states
	void linkInStates(DynParamManager& states);       //!< -- Method for SMD to get access of other states
    void writeOutputStateMessages(uint64_t CurrentSimNanos);
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B);  //!< -- Force and torque on s/c due to translating body
    void updateEffectorMassProps(double integTime);  //!< -- Method for stateEffector to give mass contributions
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives
    void UpdateState(uint64_t CurrentSimNanos);
    void computeTranslatingBodyInertialStates();               //!< Method for computing the SB's states
    void backSubContribution(BackSubMatrices& backSubContr, Eigen::Vector3d omega_BN_B, Eigen::Vector3d F_g);

};


#endif /* LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H */