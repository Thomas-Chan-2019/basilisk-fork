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
	this->pHat_B.setIdentity();
	this->r_PcP_P.setZero();
	this->r_P0B_B.setIdentity();
	this->IPntPc_P.setIdentity();
	this->dcm_PB.setIdentity();
	this->nameOfRhoState = "linearTrsanslationRho" + std::to_string(this->effectorID);
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

/*! Method for translating body to access the states that it needs. It needs gravity and the hub states */
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

/*! This is the method for the translating body to register its states: rho and rhoDot */
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

	return;
}



/*! This is the method for the translating body to add its contributions to the mass props and mass prop rates of the vehicle */
void linearTranslationOneDOFStateEffector::updateEffectorMassProps(double integTime)
{
	// - Grab rho from state manager and define r_PcB_B
	this->rho = this->rhoState->getState()(0,0);
    this->r_PcP0_B = this->dcm_PB.transpose() * this->r_PcP_P;
	this->r_PcB_B = this->r_P0B_B + this->rho * this->pHat_B + this->r_PcP0_B;

	// - Update the effectors mass
	this->effProps.mEff = this->mass;
	// - Update the position of CoM
	this->effProps.rEff_CB_B = this->r_PcB_B;
	// - Update the inertia about B
	this->rTilde_PcB_B = eigenTilde(this->r_PcB_B);
	this->IPntPc_B = this->dcm_PB.transpose()*this->IPntPc_P*this->dcm_PB;
	this->effProps.IEffPntB_B = this->IPntPc_B + this->mass * this->rTilde_PcB_B * this->rTilde_PcB_B.transpose();

	// - Grab rhoDot from the stateManager and define rPrime_PcB_B
	this->rhoDot = this->rhoDotState->getState()(0, 0);
	this->rPrime_PcB_B = this->rhoDot * this->pHat_B;
	this->effProps.rEffPrime_CB_B = this->rPrime_PcB_B;

	// - Update the body time derivative of inertia about B
	this->rPrimeTilde_PcB_B = eigenTilde(this->rPrime_PcB_B);
	this->effProps.IEffPrimePntB_B = -this->mass*(this->rPrimeTilde_PcB_B*this->rTilde_PcB_B
                                                                          + this->rTilde_PcB_B*this->rPrimeTilde_PcB_B);

    return;
}


/*! This method is for the translating body to add its contributions to the back-sub method */
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
    Eigen::Vector3d F_g;
    gLocal_N = *this->g_N;
    g_B = dcm_BN*gLocal_N;
    F_g = this->mass*g_B;

	//  aRho
    this->aRho = -this->mass*this->pHat_B.transpose();

    //  bRho
    this->bRho = this->mass*this->pHat_B.transpose()*this->rTilde_PcB_B;

    //  cRho
    // grabbing omega
    Eigen::Vector3d omega_BN_B_local = this->omegaState->getState();
    Eigen::Matrix3d omegaTilde_BN_B_local;
    omegaTilde_BN_B_local = eigenTilde(omega_BN_B_local);
	cRho = 1.0/(this->mass)*(this->pHat_B.transpose() * F_g - this->k*this->rho - this->c*this->rhoDot
		         - 2 * this->mass*this->pHat_B.transpose() * (omegaTilde_BN_B_local * this->rPrime_PcB_B)
		                   - this->mass*this->pHat_B.transpose() * (omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->r_PcB_B));

	// - Compute matrix/vector contributions, this excludes the general contributions
	backSubContr.matrixA = this->mass*this->pHat_B*this->aRho.transpose();
    backSubContr.matrixB = this->mass*this->pHat_B*this->bRho.transpose();
    backSubContr.matrixC = this->mass*this->rTilde_PcB_B*this->pHat_B*this->aRho.transpose();
	backSubContr.matrixD = this->mass*this->rTilde_PcB_B*this->pHat_B*this->bRho.transpose();
	backSubContr.vecTrans = -this->mass*this->cRho*this->pHat_B;
	backSubContr.vecRot = -this->mass*omegaTilde_BN_B_local * this->rTilde_PcB_B *this->rPrime_PcB_B -
                                                             this->mass*this->cRho*this->rTilde_PcB_B * this->pHat_B;
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

    return;
}

/*! This method is for the translating body to add its contributions to energy and momentum */
void linearTranslationOneDOFStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                          double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    //  - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();
    Eigen::Vector3d rDotPcB_B;

    // - Find rotational angular momentum contribution from hub
    rDotPcB_B = this->rPrime_PcB_B + omegaLocal_BN_B.cross(this->r_PcB_B);
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

    // - Calculate force that the translating body is applying to the spacecraft
    this->forceOnBody_B = -(this->mass*this->pHat_B*rhoDDotLocal + 2*omegaLocalTilde_BN_B*this->mass*this->rhoDot*this->pHat_B);

    // - Calculate torque that the translating body is applying about point B
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