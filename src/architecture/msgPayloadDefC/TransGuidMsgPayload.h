#ifndef TRANS_GUID_MESSAGE_H
#define TRANS_GUID_MESSAGE_H

/*! @brief Structure used to define the output definition for translational guidance or error*/
typedef struct {
    double r_BR_B[3];       //!< [m] relative error from actual to reference translational distance
    double v_BR_B[3];       //!< [m/s]  relative error velocity
    // double omega_RN_B[3];       //!< [r/s]  
    // double domega_RN_B[3];      //!< [r/s2] 
}TransGuidMsgPayload;

#endif