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

#include "linearTranslationOneDOFStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"

// get rid of mass state for now
/*! This is the constructor, setting variables to default values */
linearTranslationOneDOFStateEffector::linearTranslationOneDOFStateEffector()
{
	// - zero the contributions for mass props and mass rates
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.setZero();
	this->effProps.rEff_CB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();

	// - Initialize the variables to working values
	this->pHat_p.setIdentity();
	this->r_pcp_p.setZero();
	this->r_p0B_B.setIdentity();
	this->IPntpc_p.setIdentity();
	this->dcm_pB.setIdentity();
	this->nameOfRhoState = "linearTranslationRho" + std::to_string(this->effectorID);
	this->nameOfRhoDotState = "linearTranslationRhoDot" + std::to_string(this->effectorID);
    this->effectorID++;

	return;
}

uint64_t linearTranslationOneDOFStateEffector::effectorID = 1;

/*! This is the destructor, nothing to report here */
linearTranslationOneDOFStateEffector::~linearTranslationOneDOFStateEffector()
{
    this->effectorID = 1;    /* reset the panel ID*/
    return;
}

/*! Method for spring mass damper particle to access the states that it needs. It needs gravity and the hub states */
void linearTranslationOneDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    // - Grab access to the hub states
	this->omegaState = statesIn.getStateObject("hubOmega");
	this->sigmaState = statesIn.getStateObject("hubSigma");
	this->velocityState = statesIn.getStateObject("hubVelocity");

    // - Grab access to gravity
    this->g_N = statesIn.getPropertyReference("g_N");

    // - Grab access to c_B and cPrime_B
    this->c_B = statesIn.getPropertyReference("centerOfMassSC");
    this->cPrime_B = statesIn.getPropertyReference("centerOfMassPrimeSC");

    return;
}

/*! This is the method for the spring mass damper particle to register its states: rho and rhoDot */
void linearTranslationOneDOFStateEffector::registerStates(DynParamManager& states)
{
    // - Register rho and rhoDot
	this->rhoState = states.registerState(1, 1, nameOfRhoState);
    Eigen::MatrixXd rhoInitMatrix(1,1);
    rhoInitMatrix(0,0) = this->rhoInit;
    this->rhoState->setState(rhoInitMatrix);
	this->rhoDotState = states.registerState(1, 1, nameOfRhoDotState);
    Eigen::MatrixXd rhoDotInitMatrix(1,1);
    rhoDotInitMatrix(0,0) = this->rhoDotInit;
    this->rhoDotState->setState(rhoDotInitMatrix);

//	// - Register mass
//	this->massState = states.registerState(1, 1, nameOfMassState);
//    Eigen::MatrixXd massInitMatrix(1,1);
//    massInitMatrix(0,0) = this->massInit;
//    this->massState->setState(massInitMatrix);

	return;
}



/*! This is the method for the SMD to add its contributions to the mass props and mass prop rates of the vehicle */
void linearTranslationOneDOFStateEffector::updateEffectorMassProps(double integTime)
{
	// - Grab rho from state manager and define r_PcB_B
	this->rho = this->rhoState->getState()(0,0);
	this->r_pcp0_B = this->r_pcp0_p*this->dcm_pB
	this->r_pcB_B = this->r_p0B_B + this->rho * this->pHat_B + this->r_pcp0_B;

	// - Update the effectors mass
	this->effProps.mEff = this->mass;
	// - Update the position of CoM
	this->effProps.rEff_CB_B = this->r_pcB_B;
	// - Update the inertia about B UPDATED
	this->rTilde_pcB_B = eigenTilde(this->r_pcB_B);
	this->IPntpc_B = this->dcm_pB*this->IPntpc_p*this->dcm_pB.transpose();
	this->effProps.IEffPntB_B = this->IPntpc_B + this->mass * this->rTilde_pcB_B * this->rTilde_pcB_B.transpose();

	// - Grab rhoDot from the stateManager and define rPrime_PcB_B
	this->rhoDot = this->rhoDotState->getState()(0, 0);
	this->rPrime_pcB_B = this->rhoDot * this->pHat_B;
	this->effProps.rEffPrime_CB_B = this->rPrime_pcB_B;

	// - Update the body time derivative of inertia about B
	this->rPrimeTilde_pcB_B = eigenTilde(this->rPrime_pcB_B);
	// need to update this with Ipntpc_p somehow (nvm IPrimepntpc_p = 0)
	this->effProps.IEffPrimePntB_B = -this->mass*(this->rPrimeTilde_pcB_B*this->rTilde_pcB_B
                                                                          + this->rTilde_pcB_B*this->rPrimeTilde_pcB_B);

    return;
}

///*! This is method is used to pass mass properties information to the fuelTank */
//void linearTranslationOneDOFStateEffector::retrieveMassValue(double integTime)
//{
//    // Save mass value into the fuelSlosh class variable
//    this->fuelMass = this->massSMD;
//
//    return;
//}

/*! This method is for the SMD to add its contributions to the back-sub method */
void linearTranslationOneDOFStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // - Find dcm_BN UNALTERED
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_NB;
    sigmaLocal_BN = (Eigen::Vector3d ) this->sigmaState->getState();
    dcm_NB = sigmaLocal_BN.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();

    // - Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_B;
    gLocal_N = *this->g_N;
    g_B = dcm_BN*gLocal_N;
    F_g = this->mass*g_B;

	//  aRho
    this->aRho = -this->pHat_B.transpose();
//    old aRho
//    this->aRho = -this->pHat_B;

    //  bRho
    this->bRho = this->rTilde_pcB_B*this->pHat_B.transpose();
//    old bRho
//    this->bRho = -this->rTilde_PcB_B*this->pHat_B;

    //  cRho

    // grabbing omega
    Eigen::Vector3d omega_BN_B_local = this->omegaState->getState();
    Eigen::Matrix3d omegaTilde_BN_B_local;
    omegaTilde_BN_B_local = eigenTilde(omega_BN_B_local);
//  other cRho
//	cRho = 1.0/(this->massSMD)*(this->pHat_B.dot(this->massSMD * g_B) - this->k*this->rho - this->c*this->rhoDot
//		         - 2 * this->massSMD*this->pHat_B.dot(omegaTilde_BN_B_local * this->rPrime_PcB_B)
//		                   - this->massSMD*this->pHat_B.dot(omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->r_PcB_B));
	cRho = 1.0/(this->mass)*(this->pHat_B.transpose() * F_g - this->k*this->rho - this->c*this->rhoDot
		         - 2 * this->mass*this->pHat_B.transpose() * (omegaTilde_BN_B_local * this->rPrime_PcB_B)
		                   - this->mass*this->pHat_B.transpose() * (omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->r_PcB_B));

	// - Compute matrix/vector contributions, this excludes the general contributions
	backSubContr.matrixA = this->mass*this->pHat_B*this->aRho.transpose();
    backSubContr.matrixB = this->mass*this->pHat_B*this->bRho.transpose();
    backSubContr.matrixC = this->mass*this->rTilde_pcB_B*this->pHat_B*this->aRho.transpose();
	backSubContr.matrixD = this->mass*this->rTilde_pcB_B*this->pHat_B*this->bRho.transpose();
	backSubContr.vecTrans = -this->mass*this->cRho*this->pHat_B;
	backSubContr.vecRot = -this->mass*omegaTilde_BN_B_local * this->rTilde_pcB_B *this->rPrime_PcB_B -
                                                             this->massSMD*this->cRho*this->rTilde_PcB_B * this->pHat_B;
    return;
}

/*! This method is used to define the derivatives of rho. One is the trivial kinematic derivative and the other is
 derived using the back-sub method */
void linearTranslationOneDOFStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{

	// - Find DCM
	Eigen::MRPd sigmaLocal_BN;
	Eigen::Matrix3d dcm_BN;
	sigmaLocal_BN = (Eigen::Vector3d) this->sigmaState->getState();
	dcm_BN = (sigmaLocal_BN.toRotationMatrix()).transpose();

	// - Set the derivative of rho to rhoDot
	this->rhoState->setDerivative(this->rhoDotState->getState());

	// - Compute rhoDDot
	Eigen::MatrixXd rhoDDot(1,1);
    Eigen::Vector3d omegaDot_BN_B_local = this->omegaState->getStateDeriv();
    Eigen::Vector3d rDDot_BN_N_local = this->velocityState->getStateDeriv();
	Eigen::Vector3d rDDot_BN_B_local = dcm_BN*rDDot_BN_N_local;
    rhoDDot(0,0) = this->aRho.dot(rDDot_BN_B_local) + this->bRho.dot(omegaDot_BN_B_local) + this->cRho;
	this->rhoDotState->setDerivative(rhoDDot);

//    // - Set the massDot already computed from fuelTank to the stateDerivative of mass
//    conv(0,0) = this->fuelMassDot;
//    this->massState->setDerivative(conv);

    return;
}

/*! This method is for the SMD to add its contributions to energy and momentum */
void linearTranslationOneDOFStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                          double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    //  - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();
    Eigen::Vector3d rDotPcB_B;

    // - Find rotational angular momentum contribution from hub
    rDotpcB_B = this->rPrime_pcB_B + omegaLocal_BN_B.cross(this->r_pcB_B);
    rotAngMomPntCContr_B = this->mass*this->r_PcB_B.cross(rDotPcB_B);

    // - Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*this->mass*rDotPcB_B.dot(rDotPcB_B) + 1.0/2.0*this->k*this->rho*this->rho;

    return;
}

void linearTranslationOneDOFStateEffector::calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B)
{
    // - Get the current omega state
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = this->omegaState->getState();
    Eigen::Matrix3d omegaLocalTilde_BN_B;
    omegaLocalTilde_BN_B = eigenTilde(omegaLocal_BN_B);

    // - Get rhoDDot from last integrator call
    double rhoDDotLocal;
    rhoDDotLocal = rhoDotState->getStateDeriv()(0, 0);

    // - Calculate force that the FSP is applying to the spacecraft
    this->forceOnBody_B = -(this->mass*this->pHat_B*rhoDDotLocal + 2*omegaLocalTilde_BN_B*this->mass*this->rhoDot*this->pHat_B);

    // - Calculate torque that the FSP is applying about point B
    this->torqueOnBodyPntB_B = -(this->mass*this->rTilde_PcB_B*this->pHat_B*rhoDDotLocal + this->mass*omegaLocalTilde_BN_B*this->rTilde_PcB_B*this->rPrime_PcB_B - this->mass*(this->rPrimeTilde_PcB_B*this->rTilde_PcB_B + this->rTilde_PcB_B*this->rPrimeTilde_PcB_B)*omegaLocal_BN_B);

    // - Define values needed to get the torque about point C
    Eigen::Vector3d cLocal_B = *this->c_B;
    Eigen::Vector3d cPrimeLocal_B = *this->cPrime_B;
    Eigen::Vector3d r_PcC_B = this->r_PcB_B - cLocal_B;
    Eigen::Vector3d rPrime_PcC_B = this->rPrime_PcB_B - cPrimeLocal_B;
    Eigen::Matrix3d rTilde_PcC_B = eigenTilde(r_PcC_B);
    Eigen::Matrix3d rPrimeTilde_PcC_B = eigenTilde(rPrime_PcC_B);

    // - Calculate the torque about point C
    this->torqueOnBodyPntC_B = -(this->mass*rTilde_PcC_B*this->pHat_B*rhoDDotLocal + this->mass*omegaLocalTilde_BN_B*rTilde_PcC_B*rPrime_PcC_B - this->mass*(rPrimeTilde_PcC_B*rTilde_PcC_B + rTilde_PcC_B*rPrimeTilde_PcC_B)*omegaLocal_BN_B);

    return;
}






















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
//#include "linearTranslationOneDOFStateEffector.h"
//#include "architecture/utilities/avsEigenSupport.h"
//#include "architecture/utilities/rigidBodyKinematics.h"
//#include <string>
//
///*! This is the constructor, setting variables to default values */
//linearTranslationOneDOFStateEffector::linearTranslationOneDOFStateEffector()
//{
//    // Zero the mass props and mass prop rates contributions
//    this->effProps.mEff = 0.0;
//    this->effProps.rEff_CB_B.fill(0.0);
//    this->effProps.IEffPntB_B.fill(0.0);
//    this->effProps.rEffPrime_CB_B.fill(0.0);
//    this->effProps.IEffPrimePntB_B.fill(0.0);
//
//    // Initialize variables to working values
//    this->IPntp_pc.setIdentity();
//    this->dcm_pB.setIdentity();
//    this->r_p0_B.setZero();
//    this->r_pc_p.setZero();
//    this->pHat_p.setZero();
//
//    this->nameOfRhoState = "linearTranslationRho" + std::to_string(linearTranslationOneDOFStateEffector::effectorID);
//    this->nameOfRhoDotState = "linearTranslationRhoDot" + std::to_string(linearTranslationOneDOFStateEffector::effectorID);
//    linearTranslationOneDOFStateEffector::effectorID++;
//}
//
//uint64_t linearTranslationOneDOFStateEffector::effectorID = 1;
//
///*! This is the destructor, nothing to report here */
//linearTranslationOneDOFStateEffector::~linearTranslationOneDOFStateEffector()
//{
//    linearTranslationOneDOFStateEffector::effectorID = 1;
//}
//
///*! This method is used to reset the module. */
//void linearTranslationOneDOFStateEffector::Reset(uint64_t CurrentClock)
//{
//    // Normalize the sHat vector
//    if (this->pHat_p.norm() > 0.01) {
//        this->pHat_p.normalize();
//    }
//    else {
//        bskLogger.bskLog(BSK_ERROR, "Norm of pHat must be greater than 0. pHat may not have been set by the user.");
//    }
//}
//
//
///*! This method takes the computed theta states and outputs them to the messaging system. */
//void linearTranslationOneDOFStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
//{
//    // Write out the translational motion output messages
//    if (this->linearTranslationOutMsg.isLinked()) {
//        HingedRigidBodyMsgPayload spinningBodyBuffer;
//        linearTranslationBuffer = this->spinningBodyOutMsg.zeroMsgPayload;
//        linearTranslationBuffer.rho = this->rho;
//        linearTranslationBuffer.rhoDot = this->rhoDot;
//        this->linearTranslationOutMsg.write(&spinningBodyBuffer, this->moduleID, CurrentClock);
//    }
//
//    // Write out the translational motion state config log message
//    if (this->linearTranslationConfigLogOutMsg.isLinked()) {
//        SCStatesMsgPayload configLogMsg;
//        configLogMsg = this->linearTranslationConfigLogOutMsg.zeroMsgPayload;
//
//        // Logging the S frame is the body frame B of that object
//        eigenVector3d2CArray(this->r_ScN_N, configLogMsg.r_BN_N);
//        eigenVector3d2CArray(this->v_ScN_N, configLogMsg.v_BN_N);
//        eigenVector3d2CArray(this->sigma_SN, configLogMsg.sigma_BN);
//        eigenVector3d2CArray(this->omega_SN_S, configLogMsg.omega_BN_B);
//        this->linearTranslationConfigLogOutMsg.write(&configLogMsg, this->moduleID, CurrentClock);
//    }
//}
//
///*! This method prepends the name of the spacecraft for multi-spacecraft simulations.*/
//void linearTranslationOneDOFStateEffector::prependSpacecraftNameToStates()
//{
//    this->nameOfRhoState = this->nameOfSpacecraftAttachedTo + this->nameOfRhoState;
//    this->nameOfRhoDotState = this->nameOfSpacecraftAttachedTo + this->nameOfRhoDotState;
//}
//
///*! This method allows the SB state effector to have access to the hub states and gravity*/
//void linearTranslationOneDOFStateEffector::linkInStates(DynParamManager& statesIn)
//{
//    // - Get access to the hub's states needed for dynamic coupling
//    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N");
//    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "v_BN_N");
//}
//
///*! This method allows the LT state effector to register its states: rho and rhoDot with the dynamic parameter manager */
//void linearTranslationOneDOFStateEffector::registerStates(DynParamManager& states)
//{
//    // Register the theta state
//    this->rhoState = states.registerState(1, 1, this->nameOfRhoState);
//    Eigen::MatrixXd rhoInitMatrix(1,1);
//    rhoInitMatrix(0,0) = this->rhoInit;
//    this->rhoState->setState(rhoInitMatrix);
//
//    // Register the thetaDot state
//    this->rhoDotState = states.registerState(1, 1, this->nameOfRhoDotState);
//    Eigen::MatrixXd rhoDotInitMatrix(1,1);
//    rhoDotInitMatrix(0,0) = this->rhoDotInit;
//    this->rhoDotState->setState(rhoDotInitMatrix);
//}
//
///*! This method allows the SB state effector to provide its contributions to the mass props and mass prop rates of the
// spacecraft */
//void linearTranslationOneDOFStateEffector::updateEffectorMassProps(double integTime)
//{
//    // Give the mass of the spinning body to the effProps mass
//    this->effProps.mEff = this->mass;
//
//    // Grab current states
//    this->rho = this->rhoState->getState()(0, 0);
//    this->rhoDot = this->rhoDotState->getState()(0, 0);
//
//    // Compute the DCM from S frame to B frame and write sHat in B frame
////    double dcm_S0S[3][3];
////    double prv_S0S_array[3];
////    Eigen::Vector3d prv_S0S = -this->theta * this->sHat_S;
////    eigenVector3d2CArray(prv_S0S, prv_S0S_array);
////    PRV2C(prv_S0S_array, dcm_S0S);
////    this->dcm_BS = this->dcm_S0B.transpose() * c2DArray2EigenMatrix3d(dcm_S0S);
//
//    this->pHat_B = this->dcm_pB * this->pHat_p;
//
//    // Compute the effector's CoM with respect to point B
//    this->r_ScS_B = this->dcm_BS * this->r_ScS_S;
//    this->r_ScB_B = this->r_ScS_B + this->r_SB_B;
//    this->effProps.rEff_CB_B = this->r_ScB_B;
//
//    // Find the inertia of the hinged rigid body about point B
//    this->rTilde_ScB_B = eigenTilde(this->r_ScB_B);
//    this->IPntSc_B = this->dcm_BS * this->IPntSc_S * this->dcm_BS.transpose();
//    this->effProps.IEffPntB_B = this->IPntSc_B - this->mass * this->rTilde_ScB_B * this->rTilde_ScB_B;
//
//    // Define omega_SB_B and its cross product operator
//    this->omega_SB_B = this->thetaDot * this->sHat_B;
//    this->omegaTilde_SB_B = eigenTilde(this->omega_SB_B);
//
//    // Find rPrime_ScB_B
//    this->rPrime_ScS_B = this->omegaTilde_SB_B * this->r_ScS_B;
//    this->rPrime_ScB_B = this->rPrime_ScS_B;
//    this->effProps.rEffPrime_CB_B = this->rPrime_ScB_B;
//
//    // Find body time derivative of IPntSc_B
//    Eigen::Matrix3d rPrimeTilde_ScB_B = eigenTilde(this->rPrime_ScB_B);
//    this->effProps.IEffPrimePntB_B = this->omegaTilde_SB_B* this->IPntSc_B - this->IPntSc_B *this->omegaTilde_SB_B
//        - this->mass * (rPrimeTilde_ScB_B * this->rTilde_ScB_B + this->rTilde_ScB_B * rPrimeTilde_ScB_B);
//}
//
///*! This method allows the SB state effector to give its contributions to the matrices needed for the back-sub
// method */
//void linearTranslationOneDOFStateEffector::updateContributions(double integTime,
//                                                          BackSubMatrices & backSubContr,
//                                                          Eigen::Vector3d sigma_BN,
//                                                          Eigen::Vector3d omega_BN_B,
//                                                          Eigen::Vector3d g_N)
//{
//    // Define omega_SN_B
//    this->omega_BN_B = omega_BN_B;
//    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
//    this->omega_SN_B = this->omega_SB_B + this->omega_BN_B;
//    Eigen::Matrix3d omegaTilde_SN_B = eigenTilde(this->omega_SN_B);
//
//    // Define IPntS_B for compactness
//    Eigen::Matrix3d rTilde_ScS_B = eigenTilde(this->r_ScS_B);
//    Eigen::Matrix3d IPntS_B = this->IPntSc_B - this->mass * rTilde_ScS_B * rTilde_ScS_B;
//
//    // Find the DCM from N to B frames
//    this->sigma_BN = sigma_BN;
//    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
//
//    // Map gravity to body frame
//    const Eigen::Vector3d& gLocal_N = g_N;
//    Eigen::Vector3d g_B = this->dcm_BN * gLocal_N;
//
//    // Define auxiliary variable mTheta
//    this->mTheta = this->sHat_B.transpose() * IPntS_B * this->sHat_B;
//
//    // Lock the axis if the flag is set to 1
//    if (this->lockFlag == 1)
//    {
//        // Define aTheta, bTheta and cTheta
//        this->aTheta.setZero();
//        this->bTheta.setZero();
//        this->cTheta = 0.0;
//    }
//    else {
//        // Define aTheta
//        this->aTheta = this->mass * rTilde_ScS_B * this->sHat_B / this->mTheta;
//
//        // Define bTheta
//        Eigen::Matrix3d rTilde_SB_B = eigenTilde(this->r_SB_B);
//        this->bTheta = -(IPntS_B - this->mass * rTilde_SB_B * rTilde_ScS_B) * this->sHat_B / this->mTheta;
//
//        // Define cTheta
//        Eigen::Vector3d rDot_SB_B = this->omegaTilde_BN_B * this->r_SB_B;
//        Eigen::Vector3d gravityTorquePntS_B = rTilde_ScS_B * this->mass * g_B;
//        this->cTheta = (this->u - this->k * (this->theta - this->thetaRef) - this->c * (this->thetaDot - this->thetaDotRef)
//                + this->sHat_B.dot(gravityTorquePntS_B - omegaTilde_SN_B * IPntS_B * this->omega_SN_B
//                - IPntS_B * this->omegaTilde_BN_B * this->omega_SB_B
//                - this->mass * rTilde_ScS_B * this->omegaTilde_BN_B * rDot_SB_B)) / this->mTheta;
//    }
//
//    // For documentation on contributions see Vaz Carneiro, Allard, Schaub spinning body paper
//    // Translation contributions
//    backSubContr.matrixA = -this->mass * rTilde_ScS_B * this->sHat_B * this->aTheta.transpose();
//    backSubContr.matrixB = -this->mass * rTilde_ScS_B * this->sHat_B * this->bTheta.transpose();
//    backSubContr.vecTrans = -this->mass * this->omegaTilde_SB_B * this->rPrime_ScS_B
//            + this->mass * rTilde_ScS_B * this->sHat_B * this->cTheta;
//
//    // Rotation contributions
//    backSubContr.matrixC = (this->IPntSc_B - this->mass * this->rTilde_ScB_B * rTilde_ScS_B)
//            * this->sHat_B * this->aTheta.transpose();
//    backSubContr.matrixD = (this->IPntSc_B - this->mass * this->rTilde_ScB_B * rTilde_ScS_B)
//            * this->sHat_B * this->bTheta.transpose();
//    backSubContr.vecRot = -omegaTilde_SN_B * this->IPntSc_B * this->omega_SB_B
//            - this->mass * this->omegaTilde_BN_B * this->rTilde_ScB_B * this->rPrime_ScB_B
//            - this->mass * this->rTilde_ScB_B * this->omegaTilde_SB_B * this->rPrime_ScS_B
//            - (this->IPntSc_B - this->mass * this->rTilde_ScB_B * rTilde_ScS_B) * this->sHat_B * this->cTheta;
//}
//
///*! This method is used to find the derivatives for the SB stateEffector: thetaDDot and the kinematic derivative */
//void linearTranslationOneDOFStateEffector::computeDerivatives(double integTime,
//                                                         Eigen::Vector3d rDDot_BN_N,
//                                                         Eigen::Vector3d omegaDot_BN_B,
//                                                         Eigen::Vector3d sigma_BN)
//{
//    // Update dcm_BN
//    this->sigma_BN = sigma_BN;
//    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
//
//    // Grab omegaDot_BN_B
//    Eigen::Vector3d omegaDotLocal_BN_B;
//    omegaDotLocal_BN_B = omegaDot_BN_B;
//
//    // Find rDDotLoc_BN_B
//    const Eigen::Vector3d& rDDotLocal_BN_N = rDDot_BN_N;
//    Eigen::Vector3d rDDotLocal_BN_B;
//    rDDotLocal_BN_B = this->dcm_BN * rDDotLocal_BN_N;
//
//    // Compute Derivatives
//    this->thetaState->setDerivative(this->thetaDotState->getState());
//    Eigen::MatrixXd thetaDDot(1, 1);
//    thetaDDot(0, 0) = this->aTheta.dot(rDDotLocal_BN_B)
//            + this->bTheta.dot(omegaDotLocal_BN_B) + this->cTheta;
//    this->thetaDotState->setDerivative(thetaDDot);
//}
//
///*! This method is for calculating the contributions of the SB state effector to the energy and momentum of the spacecraft */
//void linearTranslationOneDOFStateEffector::updateEnergyMomContributions(double integTime,
//                                                                   Eigen::Vector3d & rotAngMomPntCContr_B,
//                                                                   double & rotEnergyContr,
//                                                                   Eigen::Vector3d omega_BN_B)
//{
//    // Update omega_BN_B and omega_SN_B
//    this->omega_BN_B = omega_BN_B;
//    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
//    this->omega_SN_B = this->omega_SB_B + this->omega_BN_B;
//
//    // Compute rDot_ScB_B
//    this->rDot_ScB_B = this->rPrime_ScB_B + this->omegaTilde_BN_B * this->r_ScB_B;
//
//    // Find rotational angular momentum contribution from hub
//    rotAngMomPntCContr_B = this->IPntSc_B * this->omega_SN_B + this->mass * this->rTilde_ScB_B * this->rDot_ScB_B;
//
//    // Find rotational energy contribution from the hub
//    rotEnergyContr = 1.0 / 2.0 * this->omega_SN_B.dot(this->IPntSc_B * this->omega_SN_B)
//            + 1.0 / 2.0 * this->mass * this->rDot_ScB_B.dot(this->rDot_ScB_B)
//            + 1.0 / 2.0 * this->k * (this->theta - this->thetaRef) * (this->theta - this->thetaRef);
//}
//
///*! This method computes the spinning body states relative to the inertial frame */
//void linearTranslationOneDOFStateEffector::computeSpinningBodyInertialStates()
//{
//    // inertial attitude
//    Eigen::Matrix3d dcm_SN;
//    dcm_SN = (this->dcm_BS).transpose() * this->dcm_BN;
//    this->sigma_SN = eigenMRPd2Vector3d(eigenC2MRP(dcm_SN));
//
//    // inertial position vector
//    this->r_ScN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_ScB_B;
//
//    // inertial velocity vector
//    this->v_ScN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_ScB_B;
//}
//
///*! This method is used so that the simulation will ask SB to update messages */
//void linearTranslationOneDOFStateEffector::UpdateState(uint64_t CurrentSimNanos)
//{
//    //! - Read the incoming command array
//    if (this->motorTorqueInMsg.isLinked() && this->motorTorqueInMsg.isWritten()) {
//        ArrayMotorTorqueMsgPayload incomingCmdBuffer;
//        incomingCmdBuffer = this->motorTorqueInMsg();
//        this->u = incomingCmdBuffer.motorTorque[0];
//    }
//
//    //! - Read the incoming lock command array
//    if (this->motorLockInMsg.isLinked() && this->motorLockInMsg.isWritten()) {
//        ArrayEffectorLockMsgPayload incomingLockBuffer;
//        incomingLockBuffer = this->motorLockInMsg();
//        this->lockFlag = incomingLockBuffer.effectorLockFlag[0];
//    }
//
//    //! - Read the incoming angle command array
//    if (this->spinningBodyRefInMsg.isLinked() && this->spinningBodyRefInMsg.isWritten()) {
//        HingedRigidBodyMsgPayload incomingRefBuffer;
//        incomingRefBuffer = this->spinningBodyRefInMsg();
//        this->thetaRef = incomingRefBuffer.theta;
//        this->thetaDotRef = incomingRefBuffer.thetaDot;
//    }
//
//    /* Compute spinning body inertial states */
//    this->computeSpinningBodyInertialStates();
//
//    this->writeOutputStateMessages(CurrentSimNanos);
//}
