
# ISC License
#
# Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
#   Unit Test Script
#   Module Name:        translating Bodies
#   Author:             Peter Johnson
#   Creation Date:      December 3, 2023
#

import inspect
import os
import matplotlib.pyplot as plt
import numpy
import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, linearTranslationOneDOFStateEffector, gravityEffector
from Basilisk.architecture import messaging


# uncomment this line if this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail()
# provide a unique test method name, starting with test_

# runs 4 tests, each time changing the input
@pytest.mark.parametrize("cmdForce, lock, rhoRef", [
    (0.0, False, 0.0),
    (0.0, True, 0.0),
    (1.0, False, 2.0),
    (0.0, False, 5.0)])
def test_translatingBody(show_plots, cmdForce, lock, rhoRef):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with a single-axis translating rigid body attached to a rigid hub. The position
    of the boom axis is arbitrary. The scenario includes gravity acting on both the spacecraft and the effector.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained when conservative forces like gravity are present.
    Therefore, the values of the variables

    - ``finalOrbAngMom``
    - ``finalOrbEnergy``
    - ``finalRotAngMom``
    - ``finalRotEnergy``

    should be constant when tested against their initial values.
    """
    [testResults, testMessage] = translatingBody(show_plots, cmdForce, lock, rhoRef)
    assert testResults < 1, testMessage

# NEED TO ADD FLAGS BACK IN (REF SPINNING BODIES)
def translatingBody(show_plots, cmdForce, lock, rhoRef):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the spacecraft module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Create two hinged rigid bodies
    translatingBody = linearTranslationOneDOFStateEffector.linearTranslationOneDOFStateEffector()

    # Define properties of spinning body
    translatingBody.mass = 50.0
    translatingBody.rhoInit = 20.0
    translatingBody.rhoDotInit = 2.0
    translatingBody.pHat_B = [[3.0/5.0], [4.0/5.0], [0.0]]
    translatingBody.r_PcP_P = [[-1.0], [1.0], [0.0]]
    translatingBody.r_P0B_B = [[-5.0], [4.0], [3.0]]
    translatingBody.IPntPc_P = [[50.0, 0.0, 0.0],
                                [0.0, 80.0, 0.0],
                                [0.0, 0.0, 60.0]]
    translatingBody.dcm_PB = [[0.0, -1.0, 0.0],
                              [0.0, 0.0, -1.0],
                              [1.0, 0.0, 0.0]]
    translatingBody.k = 1.0
    if lock:
        translatingBody.rhoDotInit = 0
    if rhoRef != 0.0:
        translatingBody.c = 50
    translatingBody.ModelTag = "translatingBody"

    # Add spinning body to spacecraft
    scObject.addStateEffector(translatingBody)

    # create lock message
    if lock:
        lockArray = messaging.ArrayEffectorLockMsgPayload()
        lockArray.effectorLockFlag = [1]
        lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
        translatingBody.LockInMsg.subscribeTo(lockMsg)

    # Create the reference message
    translationRef = messaging.TranslatingRigidBodyMsgPayload()
    translationRef.rho = rhoRef
    translationRef.rhoDot = 0.0
    translationRefMsg = messaging.TranslatingRigidBodyMsg().write(translationRef)
    translatingBody.translatingBodyRefInMsg.subscribeTo(translationRefMsg)

    # # Create the force cmd force message
    # cmdArray = messaging.ArrayMotorForceMsgPayload()
    # cmdArray.motorForce = [cmdForce]  # [Nm]
    # cmdMsg = messaging.ArrayMotorForceMsg().write(cmdArray)
    # translatingBody.motorForceInMsg.subscribeTo(cmdMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBody)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    earthGravBody.useSphericalHarmParams = False
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)
    
    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add states to log
    rhoData = translatingBody.translatingBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, rhoData)

    # Setup and run the simulation
    stopTime = 25000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)
    rho = rhoData.rho
    rhoDot = rhoData.rhoDot

    # Setup the conservation quantities
    # to compare with previous quantities (ensure the conservation of mom and energy are fulfilled at each timestep
    initialOrbAngMom_N = [[orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]]
    finalOrbAngMom = [orbAngMom_N[-1]]
    initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]
    finalRotAngMom = [rotAngMom_N[-1]]
    initialOrbEnergy = [[orbEnergy[0, 1]]]
    finalOrbEnergy = [orbEnergy[-1]]
    initialRotEnergy = [[rotEnergy[0, 1]]]
    finalRotEnergy = [rotEnergy[-1]]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1],
             orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2],
             orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:, 0] * 1e-9, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]) / rotAngMom_N[0, 1],
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]) / rotAngMom_N[0, 2],
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]) / rotAngMom_N[0, 3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:, 0] * 1e-9, (rotEnergy[:, 1] - rotEnergy[0, 1]) / rotEnergy[0, 1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure()
    plt.clf()
    plt.plot(rhoData.times() * 1e-9, rho)
    plt.xlabel('time (s)')
    plt.ylabel('theta')

    plt.figure()
    plt.clf()
    plt.plot(rhoData.times() * 1e-9, rhoDot)
    plt.xlabel('time (s)')
    plt.ylabel('thetaDot')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-12
    finalOrbAngMom = numpy.delete(finalOrbAngMom, 0, axis=1)  # remove time column
    finalRotAngMom = numpy.delete(finalRotAngMom, 0, axis=1)  # remove time column
    finalRotEnergy = numpy.delete(finalRotEnergy, 0, axis=1)  # remove time column
    finalOrbEnergy = numpy.delete(finalOrbEnergy, 0, axis=1)  # remove time column

    for i in range(0, len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i], initialOrbAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: Translating Body integrated test failed orbital angular momentum unit test")

    for i in range(0, len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i], initialRotAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: Translating Body integrated test failed rotational angular momentum unit test")

    # Only check rotational energy if no damping and no forces are applied (WHY) cmdForce needed here
    if cmdForce == 0 and rhoRef == 0.0:
        for i in range(0, len(initialRotEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i], initialRotEnergy[i], 1, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Translating Body integrated test failed rotational energy unit test")

    for i in range(0, len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i], initialOrbEnergy[i], 1, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Translating Body integrated test failed orbital energy unit test")

# if damper given
    if rhoRef != 0.0:
        if not unitTestSupport.isDoubleEqual(rho[-1], rhoRef, 0.01):
            testFailCount += 1
            testMessages.append("FAILED: Spinning Body integrated test failed angle convergence unit test")

    if testFailCount == 0:
        print("PASSED: " + " Translating Body gravity integrated test")

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    translatingBody(True,0.0,False,0.0)
