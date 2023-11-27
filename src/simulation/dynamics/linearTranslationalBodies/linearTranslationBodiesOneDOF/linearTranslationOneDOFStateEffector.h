/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/*! @brief linear spring mass damper state effector class */
class linearTranslationOneDOFStateEffector :
	public StateEffector, public SysModel
{
public:
    double mass = 1.0;                    //!< [kg] mass of fuel
    double k = 0;                      //!< [N/m] linear spring constant for spring mass damper
    double c = 0;                      //!< [N-s/m] linear damping term for spring mass damper
    double rhoInit = 0;                //!< [m] Initial value for spring mass damper particle offset
    double rhoDotInit = 0;             //!< [m/s] Initial value for spring mass damper particle offset derivative

    std::string nameOfRhoState;    //!< [-] Identifier for the rho state data container
    std::string nameOfRhoDotState; //!< [-] Identifier for the rhoDot state data container

	BSKLogger bskLogger;                      //!< -- BSK Logging

	Eigen::Vector3d pHat_B;                                          //!< -- spinning axis in p frame components.
    Eigen::Vector3d r_pcp0_p;                                          //!< [m] vector pointing from location P along pHat to P_C in p frame components
    Eigen::Vector3d r_p0B_B;                                         //!< [m] vector pointing from body frame B origin to point p0 origin of p frame in B frame components
    Eigen::Matrix3d IPntpc_p;                                        //!< [kg-m^2] Inertia of pc about point p in p frame component
    Eigen::Matrix3d dcm_pB;                                         //!< -- DCM from the p frame to the body frame

    Message<TranslatingRigidBodyMsgPayload> translatingBodyOutMsg;           //!< state output message
    Message<SCStatesMsgPayload> translatingBodyConfigLogOutMsg;         //!< translating body state config log message

// note: i am using dcm pb instead of just giving pHat_B
private:
    double cRho;                   //!< -- Term needed for back-sub method

    double rho;                    //!< [m] spring mass damper displacement from equilibrium
    double rhoDot;                 //!< [m/s] time derivative of displacement from equilibrium
    Eigen::Vector3d r_pcB_B;       //!< [m] position vector form B to center of mass location of particle
    Eigen::Vector3d r_pcp0_B;           //!< [m] vector pointing from body frame B origin to point p0 origin of p frame in B frame components
    Eigen::Matrix3d rTilde_pcp_B;  //!< [m] tilde matrix of r_Pc_B
	Eigen::Vector3d rPrime_pcp_B;  //!< [m/s] Body time derivative of r_Pc_B
	Eigen::Matrix3d rPrimeTilde_pcp_B;  //!< [m/s] Tilde matrix of rPrime_PcB_B
	Eigen::Matrix3d rTilde_pcB_B;  //!< [m] tilde matrix of r_Pc_B
	Eigen::Vector3d rPrime_pcB_B;  //!< [m/s] Body time derivative of r_Pc_B
	Eigen::Matrix3d rPrimeTilde_pcB_B;  //!< [m/s] Tilde matrix of rPrime_PcB_B
	Eigen::Matrix3d IPntpc_B;        //!< [kg-m^2] Inertia of pc about point B in B frame component
	Eigen::Matrix3d dcm_BN;                                         //!< -- DCM from the B frame to the N frame

    Eigen::Vector3d aRho;          //!< -- Term needed for back-sub method
    Eigen::Vector3d bRho;          //!< -- Term needed for back-sub method

    Eigen::MatrixXd *g_N;          //!< [m/s^2] Gravitational acceleration in N frame components
	StateData *rhoState;		   //!< -- state data for spring mass damper displacement from equilibrium
    Eigen::MatrixXd *c_B;            //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;       //!< [m/s] Body time derivative of vector c_B in B frame components
	StateData *rhoDotState;		   //!< -- state data for time derivative of rho;
	StateData *omegaState;         //!< -- state data for the hubs omega_BN_B
	StateData *sigmaState;         //!< -- state data for the hubs sigma_BN
	StateData *velocityState;      //!< -- state data for the hubs rDot_BN_N
    static uint64_t effectorID;    //!< [] ID number of this panel

public:
	linearTranslationOneDOFStateEffector();           //!< -- Contructor
	~linearTranslationOneDOFStateEffector();          //!< -- Destructor
	void registerStates(DynParamManager& states);  //!< -- Method for SMD to register its states
	void linkInStates(DynParamManager& states);  //!< -- Method for SMD to get access of other states
    void retrieveMassValue(double integTime);
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B);  //!< -- Force and torque on s/c due to linear spring mass damper
    void updateEffectorMassProps(double integTime);  //!< -- Method for stateEffector to give mass contributions
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives
};


#endif /* LINEAR_TRANSLATION_ONE_DOF_STATE_EFFECTOR_H */
























///*
// ISC License
//
// Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
//
// */
//
//#ifndef SPINNING_BODY_ONE_DOF_STATE_EFFECTOR_H
//#define SPINNING_BODY_ONE_DOF_STATE_EFFECTOR_H
//
//#include <Eigen/Dense>
//#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
//#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
//#include "architecture/_GeneralModuleFiles/sys_model.h"
//#include "architecture/utilities/avsEigenMRP.h"
//
//#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
//#include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
//#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
//#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
//#include "architecture/messaging/messaging.h"
//
//#include "architecture/utilities/bskLogging.h"
//
///*! @brief spinning body state effector class */
//class LinearTranslationOneDOFStateEffector: public StateEffector, public SysModel {
//
//public:
//    double mass = 1.0;                                               //!< [kg] mass of spinning body
//    double k = 0.0;                                                  //!< [N-m/rad] torsional spring constant
//    double c = 0.0;                                                  //!< [N-m-s/rad] rotational damping coefficient
//    double rhoInit = 0.0;                                            //!< [m] initial fuel position along pHat
//    double rhoDotInit = 0.0;                                         //!< [m/s] initial fuel velocity along pHat
//    std::string nameOfRhoState;                                      //!< -- identifier for the rho state data container
//    std::string nameOfRhoDotState;                                   //!< -- identifier for the rhoDot state data container
//    Eigen::Vector3d pHat_p;                                          //!< -- spinning axis in p frame components.
//    Eigen::Vector3d r_pcp_p;                                          //!< [m] vector pointing from location P along pHat to P_C in p frame components
//    Eigen::Vector3d r_p0B_B;                                         //!< [m] vector pointing from body frame B origin to point p0 origin of p frame in B frame components
//    Eigen::Matrix3d IPntpc_p;                                        //!< [kg-m^2] Inertia of pc about point p in p frame component
//    Eigen::Matrix3d dcm_pB;                                         //!< -- DCM from the p frame to the body frame
//    Message<HingedRigidBodyMsgPayload> linearTranslationOutMsg;           //!< state output message
//    Message<SCStatesMsgPayload> linearTranslationConfigLogOutMsg;         //!< spinning body state config log message
//    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg;        //!< -- (optional) motor torque input message
//    ReadFunctor<ArrayEffectorLockMsgPayload> motorLockInMsg;         //!< -- (optional) motor lock flag input message
//    ReadFunctor<HingedRigidBodyMsgPayload> linearTranslationInMsg;     //!< -- (optional) spinning body reference input message name
//
//    LinearTranslationOneDOFStateEffector();  //!< -- Constructor
//    ~LinearTranslationOneDOFStateEffector() override; //!< -- Destructor
//    void Reset(uint64_t CurrentClock) override;  //!< -- Method for reset
//    void writeOutputStateMessages(uint64_t CurrentClock) override;   //!< -- Method for writing the output messages
//    void UpdateState(uint64_t CurrentSimNanos) override;             //!< -- Method for updating information
//    void registerStates(DynParamManager& statesIn) override;         //!< -- Method for registering the SB states
//    void linkInStates(DynParamManager& states) override;             //!< -- Method for getting access to other states
//    void updateContributions(double integTime, BackSubMatrices& backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N) override;   //!< -- Method for back-substitution contributions
//    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN) override;                         //!< -- Method for SB to compute its derivatives
//    void updateEffectorMassProps(double integTime) override;         //!< -- Method for giving the s/c the HRB mass props and prop rates
//    void updateEnergyMomContributions(double integTime, Eigen::Vector3d& rotAngMomPntCContr_B, double& rotEnergyContr, Eigen::Vector3d omega_BN_B) override;         //!< -- Method for computing energy and momentum for SBs
//    void prependSpacecraftNameToStates() override;                   //!< Method used for multiple spacecraft
//    void computeLinearTranslationBodyInertialStates();               //!< Method for computing the SB's states
//
//private:
//    static uint64_t effectorID;         //!< [] ID number of this panel
//    double u = 0.0;                     //!< [N-m] optional motor torque
//    double rhoRef = 0.0;              //!< [rad] spinning body reference rho
//    double rhoDotRef = 0.0;           //!< [rad] spinning body reference rho rate
//
//    // Terms needed for back substitution
////    Eigen::Vector3d aTheta;             //!< -- rDDot_BN term for back substitution
////    Eigen::Vector3d bTheta;             //!< -- omegaDot_BN term for back substitution
////    double cTheta = 0.0;                //!< -- scalar term for back substitution
////    double mTheta = 0.0;                //!< -- auxiliary term for back substitution
//
//    // Vector quantities
//
//    Eigen::Vector3d pHat_B;             //!< -- translation axis in B frame components
//    Eigen::Vector3d r_PcP_B;       //!< [m] position vector form B to center of mass location of particle
//	Eigen::Vector3d rPrime_PcP_B;  //!< [m/s] Body time derivative of r_Pc_B
//	Eigen::Matrix3d rPrimeTilde_PcB_B;  //!< [m/s] Tilde matrix of rPrime_PcB_B
//    Eigen::Vector3d omega_BN_B;         //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components.
//    Eigen::MRPd sigma_BN;               //!< -- body frame attitude wrt to the N frame in MRPs
//
//    // Matrix quantities
//    Eigen::Matrix3d rTilde_PcP_B;        //!< [m] tilde matrix of r_Pc_B
//    Eigen::Matrix3d omegaTilde_BN_B;    //!< [rad/s] tilde matrix of omega_BN_B
//    Eigen::Matrix3d dcm_BN;             //!< -- DCM from inertial frame to body frame
//    Eigen::Matrix3d IPntpc_B;           //!< [kg-m^2] inertia of spinning body about point Sc in B frame components
//
//    // Spinning body properties
//    Eigen::Vector3d r_ScN_N;            //!< [m] position vector of spinning body center of mass Sc relative to the inertial frame origin N
//    Eigen::Vector3d v_ScN_N;            //!< [m/s] inertial velocity vector of Sc relative to inertial frame
//
//    // States
//    double rho = 0.0;                           //!< [m]
//    double rhoDot = 0.0;                        //!< [m/s]
//    Eigen::MatrixXd* inertialPositionProperty = nullptr;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
//    Eigen::MatrixXd* inertialVelocityProperty = nullptr;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
//    StateData *rhoState = nullptr;              //!< -- state manager of rho
//    StateData *rhoDotState = nullptr;           //!< -- state manager of rhoDot
//};
//
//
//#endif /* SPINNING_BODY_ONE_DOF_STATE_EFFECTOR_H */
