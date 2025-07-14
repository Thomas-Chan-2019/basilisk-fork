/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "fswAlgorithms/effectorInterfaces/forceTorqueThrForceMapping/forceTorqueThrForceMapping.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"
#include "architecture/utilities/linearAlgebra.h"

/*!
    This method initializes the output messages for this module.

 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig  *configData, int64_t moduleID)
{
    THRArrayCmdForceMsg_C_init(&configData->thrForceCmdOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.

 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    if (!THRArrayConfigMsg_C_isLinked(&configData->thrConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping.thrConfigInMsg was not connected.");
    }
    if (!VehicleConfigMsg_C_isLinked(&configData->vehConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping.vehConfigInMsg was not connected.");
    }

    VehicleConfigMsgPayload vehConfigInMsgBuffer;  //!< local copy of message buffer
    THRArrayConfigMsgPayload thrConfigInMsgBuffer;  //!< local copy of message buffer
    THRArrayCmdForceMsgPayload thrForceCmdOutMsgBuffer;  //!< local copy of message buffer

    //!< read the rest of the input messages
    thrConfigInMsgBuffer = THRArrayConfigMsg_C_read(&configData->thrConfigInMsg);
    vehConfigInMsgBuffer = VehicleConfigMsg_C_read(&configData->vehConfigInMsg);

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    configData->numThrusters = (uint32_t) thrConfigInMsgBuffer.numThrusters;
    v3Copy(vehConfigInMsgBuffer.CoM_B, configData->CoM_B);
    if (configData->numThrusters > MAX_EFF_CNT) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping thruster configuration input message has a number of thrusters that is larger than MAX_EFF_CNT");
    }

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    for(uint32_t i = 0; i < configData->numThrusters; i++)
    {
        v3Copy(thrConfigInMsgBuffer.thrusters[i].rThrust_B, configData->rThruster_B[i]);
        v3Copy(thrConfigInMsgBuffer.thrusters[i].tHatThrust_B, configData->gtThruster_B[i]);
        if(thrConfigInMsgBuffer.thrusters[i].maxThrust <= 0.0){
            _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping: A configured thruster has a non-sensible saturation limit of <= 0 N!");
        }
    }

    /* zero the thruster force command output message */
    thrForceCmdOutMsgBuffer = THRArrayCmdForceMsg_C_zeroMsgPayload();
    THRArrayCmdForceMsg_C_write(&thrForceCmdOutMsgBuffer, &configData->thrForceCmdOutMsg, moduleID, callTime);
}


/*! This method reallocates the thruster inputs to ensure non-negative values
    while satisfying the desired force/torque outputs.
 @return void
 @param DG The configuration matrix relating thruster inputs to force/torque outputs
 @param rows The number of rows in the DG matrix
 @param cols The number of columns in the DG matrix
 @param force_B The original thruster input vector (can contain negative values)
 @param forceTorque_B The desired force/torque output vector
 @param force_B_mod The modified thruster input vector (non-negative values)
*/
void reallocate_thrusters(double DG[][MAX_EFF_CNT], size_t rows, size_t cols, double *force_B, double *forceTorque_B, double *force_B_mod) {
    const double tolerance = 1e-6;
    const int max_iterations = 100;
    
    // Initialize force_B_mod to be the same as force_B
    for (uint32_t i = 0; i < cols; i++) {
        force_B_mod[i] = force_B[i];
    }

    // Iterative approach to handle negative thrusters
    for (int iter = 0; iter < max_iterations; iter++) {
        bool has_negative = false;
        
        // Find the most negative thruster
        int worst_thruster = -1;
        double min_force = 0.0;
        for (uint32_t i = 0; i < cols; i++) {
            if (force_B_mod[i] < min_force) {
                min_force = force_B_mod[i];
                worst_thruster = i;
                has_negative = true;
            }
        }
        
        if (!has_negative) {
            break; // All thrusters are non-negative
        }
        
        // Zero out the worst thruster and redistribute its contribution
        double deficit = -force_B_mod[worst_thruster];
        force_B_mod[worst_thruster] = 0.0;
        
        // Calculate the force/torque deficit caused by zeroing this thruster
        double deficit_wrench[6] = {0};
        for (uint32_t i = 0; i < rows; i++) {
            deficit_wrench[i] = DG[i][worst_thruster] * deficit;
        }
        
        // Find available thrusters to compensate
        uint32_t available_thrusters[MAX_EFF_CNT];
        uint32_t num_available = 0;
        for (uint32_t i = 0; i < cols; i++) {
            if (i != worst_thruster && force_B_mod[i] > tolerance) {
                available_thrusters[num_available] = i;
                num_available++;
            }
        }
        
        if (num_available == 0) {
            // No available thrusters to redistribute - this is a degenerate case
            break;
        }
        
        // Redistribute the deficit proportionally among available thrusters
        // Use a simple weighted distribution based on current force levels
        double total_weight = 0.0;
        for (uint32_t i = 0; i < num_available; i++) {
            total_weight += force_B_mod[available_thrusters[i]];
        }
        
        if (total_weight > tolerance) {
            for (uint32_t i = 0; i < num_available; i++) {
                uint32_t thruster_idx = available_thrusters[i];
                double weight = force_B_mod[thruster_idx] / total_weight;
                
                // Redistribute the deficit proportionally
                for (uint32_t j = 0; j < rows; j++) {
                    if (fabs(DG[j][thruster_idx]) > tolerance) {
                        force_B_mod[thruster_idx] += weight * deficit_wrench[j] / DG[j][thruster_idx];
                        break; // Use the first non-zero row for this thruster
                    }
                }
            }
        }
    }
    
    // Final check: ensure we still satisfy the force/torque requirements
    // If not, scale all thrusters proportionally to meet requirements
    double actual_wrench[6] = {0};
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < cols; j++) {
            actual_wrench[i] += DG[i][j] * force_B_mod[j];
        }
    }
    
    // Check if we need to scale to meet requirements
    double max_error = 0.0;
    for (uint32_t i = 0; i < rows; i++) {
        double error = fabs(actual_wrench[i] - forceTorque_B[i]);
        if (error > max_error) {
            max_error = error;
        }
    }
    
    // If error is significant, apply a correction
    if (max_error > tolerance) {
        // Simple correction: find the largest thruster and adjust it
        uint32_t largest_thruster = 0;
        for (uint32_t i = 1; i < cols; i++) {
            if (force_B_mod[i] > force_B_mod[largest_thruster]) {
                largest_thruster = i;
            }
        }
        
        // Apply correction to the largest thruster (if possible)
        for (uint32_t i = 0; i < rows; i++) {
            if (fabs(DG[i][largest_thruster]) > tolerance) {
                double correction = (forceTorque_B[i] - actual_wrench[i]) / DG[i][largest_thruster];
                if (force_B_mod[largest_thruster] + correction >= 0) {
                    force_B_mod[largest_thruster] += correction;
                    break;
                }
            }
        }
    }
}


/*! Add a description of what this main Update() routine does for this module

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    CmdTorqueBodyMsgPayload cmdTorqueInMsgBuffer;  //!< local copy of message buffer
    CmdForceBodyMsgPayload cmdForceInMsgBuffer;  //!< local copy of message buffer
    THRArrayCmdForceMsgPayload thrForceCmdOutMsgBuffer;  //!< local copy of message buffer

    // always zero the output message buffers before assigning values
    thrForceCmdOutMsgBuffer = THRArrayCmdForceMsg_C_zeroMsgPayload();

    /* Check if torque message is linked and read, zero out if not*/
    if (CmdTorqueBodyMsg_C_isLinked(&configData->cmdTorqueInMsg)) {
        cmdTorqueInMsgBuffer = CmdTorqueBodyMsg_C_read(&configData->cmdTorqueInMsg);
    } else{
        cmdTorqueInMsgBuffer = CmdTorqueBodyMsg_C_zeroMsgPayload();
    }

    /* Check if force message is linked and read, zero out if not*/
    if (CmdForceBodyMsg_C_isLinked(&configData->cmdForceInMsg)) {
        cmdForceInMsgBuffer = CmdForceBodyMsg_C_read(&configData->cmdForceInMsg);
    } else{
        cmdForceInMsgBuffer = CmdForceBodyMsg_C_zeroMsgPayload();
    }

    /*
    // REMOVE WHEN DEBUGGING FINISHED
    printf("cmdForceInMsgBuffer: \n");
    int numElements = sizeof(cmdForceInMsgBuffer.forceRequestBody) / sizeof(cmdForceInMsgBuffer.forceRequestBody[0]);
    for (int i = 0; i < numElements; i++) {
        // Print each float followed by a space
        printf("%.3f ", cmdForceInMsgBuffer.forceRequestBody[i]);
    }
    printf("\n");
    // REMOVE WHEN DEBUGGING FINISHED 
    */
    
    /* Initialize variables */
    double DG[6][MAX_EFF_CNT];
    double rThrusterRelCOM_B[MAX_EFF_CNT][3];
    double rCrossGt[3];
    double zeroVector[MAX_EFF_CNT];
    uint32_t zeroRows[6];
    uint32_t numZeroes;
    double force_B[MAX_EFF_CNT];
    double forceTorque_B[6];
    double forceSubtracted_B[MAX_EFF_CNT];
    vSetZero(force_B, (size_t) MAX_EFF_CNT);
    vSetZero(forceSubtracted_B, (size_t) MAX_EFF_CNT);

    for (uint32_t i = 0; i < 6; i++) {
        for (uint32_t j = 0; j < MAX_EFF_CNT; j++) {
            DG[i][j] = 0.0;
        }
    }

    /* Create the torque and force vector */
    for (uint32_t i = 0; i < 3; i++) {
        forceTorque_B[i] = cmdTorqueInMsgBuffer.torqueRequestBody[i];
        forceTorque_B[i+3] = cmdForceInMsgBuffer.forceRequestBody[i];
    }

    /* - compute thruster locations relative to COM */
    for (uint32_t i = 0; i<configData->numThrusters; i++) {
        v3Subtract(configData->rThruster_B[i], configData->CoM_B, rThrusterRelCOM_B[i]);
    }

    /* Fill DG with thruster directions and moment arms */
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        /* Compute moment arm and fill in */
        v3Cross(rThrusterRelCOM_B[i], configData->gtThruster_B[i], rCrossGt);
        for(uint32_t j = 0; j < 3; j++) {
            DG[j][i] = rCrossGt[j];
        }

        /* Fill in control axes */
        for(uint32_t j = 0; j < 3; j++) {
            DG[j+3][i] = configData->gtThruster_B[i][j];
        }
    }

    /* Check DG for zero rows */
    vSetZero(zeroVector, configData->numThrusters);
    numZeroes = 0;
    for(uint32_t j = 0; j < 6; j++) {
        if (vIsEqual(zeroVector, 6, DG[j], 0.0000001)) {
            zeroRows[j] = 1;
            numZeroes += 1;
        } else {
            zeroRows[j] = 0;
        }
    }

    /* Create the DG w/ zero rows removed */
    double DG_full[6*MAX_EFF_CNT];
    vSetZero(DG_full, (size_t) 6*MAX_EFF_CNT);
    uint32_t zeroesPassed;
    zeroesPassed = 0;
    for(uint32_t i = 0; i < 6; i++) {
        if (!zeroRows[i]) {
            for(uint32_t j = 0; j < MAX_EFF_CNT; j++) {
                DG_full[MXINDEX(MAX_EFF_CNT, i-zeroesPassed, j)] = DG[i][j];
            }
        } else {
            zeroesPassed += 1;
        }
    }

    /* Compute the minimum norm inverse of DG*/
    double DGT_DGDGT_inv[6*6];
    mMinimumNormInverse(DG_full, (size_t) 6-numZeroes, (size_t) MAX_EFF_CNT, DGT_DGDGT_inv);

    /* Add the computed pseudoinverse values back into the correct positions*/
    double DG_inv_full[MAX_EFF_CNT * 6];
    vSetZero(DG_inv_full, (size_t) MAX_EFF_CNT*6);
    uint32_t rowIndex = 0;
    for (uint32_t i = 0; i < 6; ++i) {
        if (!zeroRows[i]) {
            for (uint32_t j = 0; j < configData->numThrusters; ++j) {
                DG_inv_full[j * 6 + i] = DGT_DGDGT_inv[j * (6 - numZeroes) + rowIndex];
            }
            rowIndex++;
        }
    }

    /* Compute the force for each thruster */
    // First create the reduced force/torque vector (without zero rows)
    double forceTorque_reduced[6];
    uint32_t reducedIndex = 0;
    for (uint32_t i = 0; i < 6; ++i) {
        if (!zeroRows[i]) {
            forceTorque_reduced[reducedIndex] = forceTorque_B[i];
            reducedIndex++;
        }
    }
    
    // Now multiply with the correct dimensions
    mMultV(DGT_DGDGT_inv, (size_t) configData->numThrusters, (size_t) 6-numZeroes, forceTorque_reduced, force_B);

    // /* Subtract the minimum force */
    // for(uint32_t i = 0; i < configData->numThrusters; i++) {
    //     forceSubtracted_B[i] = force_B[i] - min_force;
    // }

    // Elias: Reallocate thrusters to ensure non-negative values
    // Reallocate thrusters to ensure non-negative values
    reallocate_thrusters(DG, (size_t) 6, (size_t) configData->numThrusters, force_B, forceTorque_B, forceSubtracted_B);

    /*
    // REMOVE WHEN DEBUGGING FINISHED
    printf("Force allocated before applying saturation: \n");
    int numUnsat = sizeof(forceSubtracted_B) / sizeof(forceSubtracted_B[0]);
    for (int i = 0; i < numUnsat; i++) {
        // Print each float followed by a space
        printf("%.3f ", forceSubtracted_B[i]);
    }
    printf("\n");
    // REMOVE WHEN DEBUGGING FINISHED
    */
    
    // Thomas as of 20240805: Apply thruster saturation after mapping the Cmd forces to thrusters by reading the `THRArrayConfigMsg`:
    THRArrayConfigMsgPayload thrConfigInMsgBuffer;  //!< local copy of `THRArrayConfigMsg` buffer from `configData`
    thrConfigInMsgBuffer = THRArrayConfigMsg_C_read(&configData->thrConfigInMsg);
    for(uint32_t i = 0; i < configData->numThrusters; i++)
    {
        // Apply saturation based on maxThrust of each thrusters:
        if(forceSubtracted_B[i] > thrConfigInMsgBuffer.thrusters[i].maxThrust){
            forceSubtracted_B[i] = thrConfigInMsgBuffer.thrusters[i].maxThrust;
        }
    }

    /* Write to the output messages */
    vCopy(forceSubtracted_B, configData->numThrusters, thrForceCmdOutMsgBuffer.thrForce);
    THRArrayCmdForceMsg_C_write(&thrForceCmdOutMsgBuffer, &configData->thrForceCmdOutMsg, moduleID, callTime);

    /*
    // REMOVE WHEN DEBUGGING FINISHED
    printf("thrForceCmdOutMsg: \n");
    int num = sizeof(thrForceCmdOutMsgBuffer.thrForce) / sizeof(thrForceCmdOutMsgBuffer.thrForce[0]);
    for (int i = 0; i < num; i++) {
        // Print each float followed by a space
        printf("%.3f ", thrForceCmdOutMsgBuffer.thrForce[i]);
    }
    printf("\n");
    // REMOVE WHEN DEBUGGING FINISHED
    */
}
