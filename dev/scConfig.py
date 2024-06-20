# Configuration file for defining simulation-used spacecrafts, denoted as "S/C".
# Defines: 
# - S/C geometry -> mass, moment of inertia, centre of mass
# - Actuators -> thrusters (impulsive/continuous burn modes), reaction wheels
import sys, json, copy, math, numpy as np
from Basilisk.simulation import spacecraft
from Basilisk.utilities import unitTestSupport, macros, orbitalMotion  # general support file with common unit test functions

sc_config_path = "dev/sc_config.json"
init_config_path = "dev/init_config.json"

class SCConfig(object): # TODO - Add RW & Thruster configs
    def __init__(self, mHub, IHub, scName="bskSC", ModelTag="spacecraftBody", 
                 thrLocation=[[0.0, 0.0, 0.0], 
                            [0.0, 0.0, 0.0], 
                            [0.0, 0.0, 0.0], 
                            [0.0, 0.0, 0.0], 
                            [0.0, 0.0, 0.0], 
                            [0.0, 0.0, 0.0]], 
                 thrDirection = [[1.0, 0.0, 0.0], 
                                [-1.0, 0.0, 0.0], 
                                [0.0, 1.0, 0.0], 
                                [0.0, -1.0, 0.0], 
                                [0.0, 0.0, 1.0], 
                                [0.0, 0.0, -1.0]],
                 MaxThrust = 4.5, 
                 r_BcB_B=[[0.0], [0.0], [0.0]], 
                 RW_gsHat_B_Matrix=[[1, 0, 0],[0, 1, 0],[0, 0, 1]],
                 THR_gsHat_B_Matrix=[[1, 0, 0],[0, 1, 0],[0, 0, 1]]
                 ):
        self.mHub = mHub
        self.IHub = IHub
        self.scName = scName
        self.ModelTag = ModelTag
        # Thruster implementation:
        self.MaxThrust = MaxThrust
        self.thrLocation = thrLocation
        self.thrDirection = thrDirection
        
        self.r_BcB_B=r_BcB_B
        self.RW_gsHat_B_Matrix = RW_gsHat_B_Matrix
        self.THR_gsHat_B_Matrix = THR_gsHat_B_Matrix

class InitConfig(object):
    def __init__(self, isTarget, scName, init_dr_hill, init_dv_hill, target_dr_hill, target_dv_hill, oe,
                 sigma_BNInit = [[0.0],[0.0],[0.0]], omega_BN_BInit = [[0.0],[0.0],[0.0]]):
        self.isTarget = isTarget
        self.scName = scName
        self.sigma_BNInit = sigma_BNInit
        self.omega_BN_BInit = omega_BN_BInit
        self.init_dr_hill = init_dr_hill
        self.init_dv_hill = init_dv_hill
        self.target_dr_hill = target_dr_hill
        self.target_dv_hill = target_dv_hill
        self.oe = oe

# @params:
# TODO        
def loadSCConfig(sc_config_path):
    with open(sc_config_path, 'r') as file:
        data = json.load(file)
    sc_configs = [SCConfig(**config) for config in data.get('SCConfig', [])]
    # other_configs1 = [OtherConfig1(**config) for config in data.get('OtherConfigs1', [])]
    # other_configs2 = [OtherConfig2(**config) for config in data.get('OtherConfigs2', [])]
    return sc_configs

# @params:
# TODO        
def loadInitConfig(init_config_path):
    with open(init_config_path, 'r') as file2:
        try:
            data2 = json.load(file2)
        except Exception as e:
            print("\nInit Config not found (with error ", e, "), please check the path again.")
    
    # Process target OE:
    targetOE_raw = data2.get('targetOE')
    targetOE = orbitalMotion.ClassicElements()
    targetOE.a = targetOE_raw[0] # [m]
    targetOE.e = targetOE_raw[1]
    targetOE.i = targetOE_raw[2] * macros.D2R
    targetOE.Omega = targetOE_raw[3] * macros.D2R
    targetOE.omega = targetOE_raw[4] * macros.D2R
    targetOE.f = targetOE_raw[5] * macros.D2R
    
    # Process initConfig:
    init_configs = [InitConfig(**config) for config in data2.get('InitConfig', [])]
    
    return targetOE, init_configs

# @params:
# TODO
def getObject(objects, key, value):
    for obj in objects:
        if hasattr(obj, key) and getattr(obj, key) == value:
            return obj
    print("\Object with", key, "=", value, "not found in config JSON. Please revise your search key/value.")
    raise ValueError(f"No object found with {key} = {value}")

# @params: TODO
# May change to pass the "initConfig" directly instead of path...
def setInitialCondition(EnvModel, DynModels, targetOE, init_configs):
    # targetOE_raw, init_configs = loadInitConfig(init_config_path)
    # targetOE, init_configs = loadInitConfig(init_config_path)
    oe_list = []
    
    mu_Earth = orbitalMotion.MU_EARTH * math.pow(1000,3) # Convert to S.I.: m^3/s^2
    omega_earth_rot = math.sqrt(mu_Earth / math.pow(targetOE.a,3))
    
    # Get RV for initialization & position deviation preparation:
    rN0, vN0 = orbitalMotion.elem2rv(EnvModel.mu, targetOE)
    orbitalMotion.rv2elem(EnvModel.mu, rN0, vN0)
    DCM_NH = orbitalMotion.hillFrame(rN0, vN0).transpose()
    for spacecraftIndex in range(len(DynModels)):
        # if config.isTarget == 1:
        #     pass
        # else:
        #     pass:
        omega_earth_rot_vec = np.array([0,0,omega_earth_rot])
        dr_hill = init_configs[spacecraftIndex].init_dr_hill
        dv_hill = init_configs[spacecraftIndex].init_dv_hill
        dv_hill += np.cross(omega_earth_rot_vec, dr_hill) # Add dv = omega x r_hill term!
        dr = DCM_NH @ dr_hill
        dv = DCM_NH @ dv_hill
        rN = rN0 + dr
        vN = vN0 + dv
        orbitalMotion.rv2elem(EnvModel.mu, rN, vN)
        
        oe_list.append(copy.deepcopy(targetOE))
        DynModels[spacecraftIndex].scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels[spacecraftIndex].scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels[spacecraftIndex].scObject.hub.sigma_BNInit = init_configs[spacecraftIndex].sigma_BNInit  # sigma_BN_B
        DynModels[spacecraftIndex].scObject.hub.omega_BN_BInit = init_configs[spacecraftIndex].omega_BN_BInit  # rad/s - omega_BN_B
    
    return oe_list
    
    
# The following is necessary!
# Load SC or other configs:
sc_configs = loadSCConfig(sc_config_path)

# @params:
# m - [kg] mass
# I - [kg*m^2, 3x3 array] moment of inertia
# ModelTag - simulation model tag for model mapping
# r_BcB_B - [m, 1x3 array] position vector of body-fixed point B relative to CM
def createSC(scName="Astrobee", RWConfig=None, THRConfig=None):
    try:
        sc_config = getObject(sc_configs, "scName", scName) 
    except ValueError as e:
        print("\nError: ", e)
    #     print('Error: S/C not implemented')
    #     exit(1)
    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = sc_config.ModelTag
    # define the simulation inertia
    scObject.hub.mHub = sc_config.mHub  # kg - spacecraft mass
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(sc_config.IHub)
    scObject.hub.r_BcB_B = sc_config.r_BcB_B  # m - position vector of body-fixed point B relative to CM
    
    if RWConfig != None:
        # TODO - Implement RWConfig and pass it out, possibly a `create RW call?`
        pass
    
    if THRConfig != None:
        # TODO - Implement RWConfig and pass it out, possibly a `create Thruster call?`
        # Do not add it to SC yet, perhaps pass it out when we return the scObject
        pass    
    
    # scObject - spacecraft.Spacecraft() object for Basilisk SC creation;
    # sc_config - SCConfig object for Thruster/RW creation.
    return scObject, sc_config

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

if __name__ == "__main__":
    sc = createSC(scName="Prima", RWConfig=None, THRConfig=None)
    print("SC ModelTag: ",sc.ModelTag)
    print("SC Mass: ",sc.hub.mHub)