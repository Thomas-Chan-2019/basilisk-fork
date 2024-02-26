# Configuration file for defining simulation-used spacecrafts, denoted as "S/C".
# Defines: 
# - S/C geometry -> mass, moment of inertia, centre of mass
# - Actuators -> thrusters (impulsive/continuous burn modes), reaction wheels
import sys
from Basilisk.simulation import spacecraft
from Basilisk.utilities import unitTestSupport, macros  # general support file with common unit test functions

class SCConfig(object):
    def __init__(self, mHub, IHub, ModelTag="spacecraftBody", r_BcB_B=[[0.0], [0.0], [0.0]]):
        self.mHub = mHub
        self.IHub = IHub
        self.ModelTag = ModelTag
        self.r_BcB_B=r_BcB_B
        
# Astrobee Config:
Astrobee = SCConfig(
    mHub=9.4,
    IHub=[0.153, 0., 0.,
     0., 0.143, 0.,
     0., 0., 0.162],
    ModelTag="astrobee"
)

# Prisma Config:
# TODO

# @params:
# m - [kg] mass
# I - [kg*m^2, 3x3 array] moment of inertia
# ModelTag - simulation model tag for model mapping
# r_BcB_B - [m, 1x3 array] position vector of body-fixed point B relative to CM
def createSC(scName="astrobee"):
    if scName=="astrobee":
        sc = Astrobee
        print("Astrobee selected.")
    else: 
        print('Error: S/C not implemented')
        exit(1)
    
    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = sc.ModelTag
    # define the simulation inertia
    scObject.hub.mHub = sc.mHub  # kg - spacecraft mass
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(sc.IHub)
    scObject.hub.r_BcB_B = sc.r_BcB_B  # m - position vector of body-fixed point B relative to CM
    
    return scObject
    

### Example RW:

#   Honeywell HR16 (100Nm, 75Nm, 50Nm)
#
#   RW Information Source:
#   http://www51.honeywell.com/aero/common/documents/Constellation_Series_Reaction_Wheels.pdf
#
#   There are 3 momentum capacity options for this RW type.  The maximum momentum
#   capacity must be set prior to creating the HR16 RW type using
#       maxMomentum = 100, 75 or 50
#
def Honeywell_HR16(self, RW):

    # maximum allowable wheel speed
    RW.Omega_max = 6000.0*macros.RPM
    # maximum RW torque [Nm]
    RW.u_max = 0.200
    # minimum RW torque [Nm]
    RW.u_min = 0.00001
    # static friction torque [Nm]
    RW.fCoulomb = 0.0005
    # RW rotor mass [kg]
    # Note: the rotor mass here is set equal to the RW mass of the above spec sheet.
    # static RW imbalance [kg*m]
    # dynamic RW imbalance [kg*m^2]
    large = 100
    medium = 75
    small = 50

    if self.maxMomentum == large:
        RW.mass = 12.0
        RW.U_s = 4.8E-6
        RW.U_d = 15.4E-7
    elif self.maxMomentum == medium:
        RW.mass = 10.4
        RW.U_s = 3.8E-6
        RW.U_d = 11.5E-7
    elif self.maxMomentum == small:
        RW.mass = 9.0
        RW.U_s = 2.8E-6
        RW.U_d = 7.7E-7
    else:
        if self.maxMomentum > 0:
            print('ERROR: ' + sys._getframe().f_code.co_name + '() does not have a correct wheel momentum of '\
                    +str(large)+', '+str(medium)+' or '+str(small)+' Nm. Provided ' + str(self.maxMomentum) + ' Nm')
        else:
            print('ERROR: ' + sys._getframe().f_code.co_name \
                    + '() maxMomentum option must be set prior to calling createRW()')
        exit(1)

    return

# For trial only:
# def run():
#     a = Astrobee
#     print(a.IHub)

# if __name__ == "__main__":
#     run()