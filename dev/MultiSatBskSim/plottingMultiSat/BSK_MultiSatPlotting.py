#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
import os
import mplfig # save plt plots like MATLAB
from scipy.io import savemat
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

plt.rcParams['axes.titlesize'] = 16
plt.rcParams['font.size'] = 14

# --------------------------------- COMPONENTS & SUBPLOT HANDLING ----------------------------------------------- #

def show_all_plots():
    plt.show()

def clear_all_plots():
    plt.close("all")

def save_all_plots(fileName, figureNames):
    figureList = {}
    numFigures = len(figureNames)
    for i in range(0, numFigures):
        pltName = fileName + "_" + figureNames[i]
        figureList[pltName] = plt.figure(i+1)
    return figureList


# Per S/C index: Export ONE matrice-dictionary to .mat files:
def export_data_mat(dataPath, data_filename, data_dict, index_string='index_0'):
    dataPathCase = dataPath + data_filename + '/'
    try:
        # Check if the folder exists
        if not os.path.exists(dataPathCase):
            # Try to create the folder
            os.makedirs(dataPathCase)
            print(f"Folder '{dataPathCase}' created.")
    except OSError as e:
        print(f"Error creating folder '{dataPathCase}': {e}")
    
    export_path = dataPathCase + index_string + "_plot_data.mat"
    savemat(export_path, data_dict)
    print("Exported data to given path: ", export_path)
    return

# For ALL S/C index: One single .mat saving all data of S/Cs for single MATLAB plotting!
def export_dicts_to_mat(data_dicts, dataPath, data_filename):
    """
    Export a list of dictionaries with the same keys into a single .mat file.
    """
    dataPathCase = dataPath + data_filename + '/'
    try:
        # Check if the folder exists
        if not os.path.exists(dataPathCase):
            # Try to create the folder
            os.makedirs(dataPathCase)
            print(f"Folder '{dataPathCase}' created.")
    except OSError as e:
        print(f"Error creating folder '{dataPathCase}': {e}")
    
    export_path = dataPathCase + data_filename + "_plot_data.mat"
    
    # Create a dictionary to hold combined data for exporting to .mat
    mat_data = {}
    
    # Get the keys from the first dictionary (assuming all dictionaries have the same keys)
    keys = data_dicts[0].keys()
    
    for key in keys:
        # For each key, stack the corresponding values from all dictionaries along a new dimension
        values = [d[key] for d in data_dicts]
        
        # Convert to numpy array if not already
        values_array = np.array(values)
        
        # Add the combined array to the mat_data dictionary
        mat_data[key] = values_array
    
    # Export the combined data to a .mat file
    savemat(export_path, mat_data)

def matrices_to_dict(**matrices):
    """
    Converts multiple matrices into a dictionary where keys are the names of the matrices 
    and values are the matrices themselves.

    Parameters:
    **matrices: Named matrices as keyword arguments.

    Returns:
    dict: A dictionary with matrix names as keys and the matrices as values.
    """
    return matrices


def save_plots_to_path(figurePath, figure_filename, index_folder_string='index_0', fileNames=None):
    figurePathCase = figurePath + figure_filename + "/" + index_folder_string + "/"
    # Subfolder for particular scenarios: 
    try:
        # Check if the folder exists
        if not os.path.exists(figurePathCase):
            # Try to create the folder
            os.makedirs(figurePathCase)
            print(f"Folder '{figurePathCase}' created.")
    except OSError as e:
        print(f"Error creating folder '{figurePathCase}': {e}")
    
    for i in plt.get_fignums():
        fig = plt.figure(i)
        mplfig.save_figure(fig, figurePathCase+'figure_%d.mplpkl' % i)
        if i==1:
            mplfig.save_figure(fig, figurePathCase+'blank.mplpkl')
        # plt.savefig(figurePath+'/figure_%d.png' % i)
        # plt.savefig(fileNames+'_figure_%d.png' % i)
    return

# ------------------------------------- MAIN PLOT HANDLING ------------------------------------------------------ #
color_x = 'dodgerblue'
color_y = 'salmon'
color_z = 'lightgreen'
m2km = 1.0 / 1000.0

def plot_attitude(timeData, dataSigmaBN, id=None):
    """Plot the spacecraft attitude."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude $\sigma_{B/N}$')
    return

def plot_attitude_error(timeData, dataSigmaBR, id=None):
    """Plot the spacecraft attitude error."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Tracking Error $\sigma_{B/R}$')
    return

def plot_attitude_reference(timeData, dataSigmaRN, id=None):
    """Plot the spacecraft attitude reference."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataSigmaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx+1) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Reference $\sigma_{R/N}$')
    return

def plot_rate(timeData, dataOmegaBN, id=None):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Angular Rate (rad/s)')
    return

def plot_rate_error(timeData, dataOmegaBR, id=None):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s)')
    return

def plot_rate_reference(timeData, dataOmegaRN, id=None):
    """Plot the body angular velocity rate reference."""
    plt.figure(id)
    for idx in range(3):
        plt.plot(timeData, dataOmegaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{RN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Reference Rate (rad/s)')
    return

def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW, id=None):
    """Plot the RW motor torques."""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx+1) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')
    return

def plot_rw_speeds(timeData, dataOmegaRW, numRW, id=None):
    """Plot the RW spin rates."""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')
    return

def plot_rw_voltages(timeData, dataVolt, numRW, id=None):
    """Plot the RW voltage inputs."""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataVolt[:, idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$V_{' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Voltage (V)')
    return

def plot_rw_power(timeData, dataRwPower, numRW, id=None):
    """Plot the RW power drain."""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataRwPower[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$p_{rw,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Power (W)')
    return

def plot_power(timeData, netData, supplyData, sinkData, id=None):
    """Plot the power inputs and outputs"""
    plt.figure(id)
    plt.plot(timeData, netData, label='Net Power')
    plt.plot(timeData, supplyData, label='Panel Power')
    plt.plot(timeData, sinkData, label='Power Draw')
    plt.xlabel('Time [min]')
    plt.ylabel('Power [W]')
    plt.grid(True)
    plt.legend(loc='lower right')
    return

def plot_battery(timeData, storageData, id=None):
    """Plot the energy inside the onboard battery"""
    plt.figure(id)
    plt.plot(timeData, storageData)
    plt.xlabel('Time [min]')
    plt.ylabel('Stored Energy [W-s]')
    return

def plot_rw_temperature(timeData, tempData, numRW, id=None):
    """Plot the reaction wheel temperatures"""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, tempData[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$T_{rw,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Temperatures [ºC]')
    return

def plot_cmd_force(timeData, dataCmdForce, id=None):
    """Plot the cmd force"""
    plt.figure(id)
    labels = ['$F_x$', '$F_y$', '$F_z$']
    for F_arr, label in zip(dataCmdForce.T, labels): # Taking dataCmdForce.transpose to arrange with "labels" for plotting purposes only.
        plt.plot(timeData, F_arr, label=label)
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Cmd Force (N) ')
    plt.title('Controller Command Force 3D')
    return

def plot_thrust(timeData, thrustData, numThr, id=None):
    """Plot the thrusters net force output"""
    plt.figure(id)
    for idx in range(numThr):
        plt.plot(timeData, thrustData[idx],
                 color=unitTestSupport.getLineColor(idx, numThr),
                 label='$F_{thr,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Thrust [N]')
    plt.title('Thruster net force output')
    return

def plot_thrust_percentage(timeData, thrustData, numThr, id=None):
    """Plot the thrust as a percentage of maximum"""
    plt.figure(id)
    for idx in range(numThr):
        plt.plot(timeData, thrustData[idx],
                 color=unitTestSupport.getLineColor(idx, numThr),
                 label='$F_{thr,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.ylim([0, 1.1])
    plt.xlabel('Time [min]')
    plt.ylabel('Thrust Percentage')
    return

def plot_OnTimeRequest(timeData, dataSchm, numTh, id=None):
    """Plot the thruster on time requests."""
    plt.figure(id)
    for idx in range(numTh):
        plt.plot(timeData, dataSchm[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label=r'$OnTimeRequest_{' + str(idx) + r'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('OnTimeRequest [sec]')

def plot_fuel(timeData, fuelData, id=None):
    """Plot the fuel mass information"""
    plt.figure(id)
    plt.plot(timeData, fuelData)
    plt.xlabel('Time [min]')
    plt.ylabel('Stored Fuel Mass [kg]')
    return

def plot_orbit(r_BN, id=None):
    """Plot the spacecraft inertial orbitS."""
    plt.figure(id, figsize=(6, 5))
    ax = plt.axes(projection='3d')
    ax.plot(r_BN[:, 0] * m2km, r_BN[:, 1] * m2km, r_BN[:, 2] * m2km,
            label="Spacecraft")
    ax.set_xlim3d(-8000, 8000)
    ax.set_ylim3d(-8000, 8000)
    ax.set_zlim3d(-8000, 8000)
    ax.scatter(0, 0, 0, c=color_x)
    ax.set_title('Spacecraft Inertial Orbit')
    ax.set_xlabel('x [km]')
    ax.set_ylabel('y [km]')
    ax.set_zlabel('z [km]')
    ax.legend(loc=2)
    return

def orbit_xyz_time_series(timeData, r_BH, SCIndex, id=None):
    fig, axs = plt.subplots(3,1)
    if id:
        fig.num = id
    fig.suptitle('Spacecraft Hill-frame Relative Position from Target $r_{BH}$: SC Index ' + str(SCIndex))
    axs[0].plot(timeData, r_BH[SCIndex][:, 0] * m2km)
    axs[1].plot(timeData, r_BH[SCIndex][:, 1] * m2km)
    axs[2].plot(timeData, r_BH[SCIndex][:, 2] * m2km)
    
    axs[0].set_title('x-axis')
    axs[1].set_title('y-axis')
    axs[2].set_title('z-axis')

    for ax in axs.flat:
        ax.set(xlabel='time', ylabel='km')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for ax in axs.flat:
        ax.label_outer()
    return

def plot_orbits(r_BN, numberSpacecraft, id=None):
    """Plot the spacecraft inertial orbits."""
    plt.figure(id, figsize=(6, 5))
    ax = plt.axes(projection='3d')
    for i in range(numberSpacecraft):
        ax.plot(r_BN[i][:, 0] * m2km, r_BN[i][:, 1] * m2km, r_BN[i][:, 2] * m2km,
                label="Spacecraft " + str(i),
                c=unitTestSupport.getLineColor(i, numberSpacecraft))
    ax.set_xlim3d(-8000, 8000)
    ax.set_ylim3d(-8000, 8000)
    ax.set_zlim3d(-8000, 8000)
    ax.scatter(0, 0, 0, c=color_x)
    ax.set_title('Spacecraft Inertial Orbits')
    ax.set_xlabel('x [km]')
    ax.set_ylabel('y [km]')
    ax.set_zlabel('z [km]')
    ax.legend(loc=2)
    return

# Added animated plot to relative orbits:
def plot_relative_orbits(r_BN, numberSpacecraft, id=None):
    """Plot the spacecraft inertial orbits."""
    fig = plt.figure(id, figsize=(6, 5))
    # ax = plt.axes(projection='3d')
    ax = fig.add_subplot(111, projection='3d')
    
    # Initialize the lines for each spacecraft
    lines = []
    for i in range(numberSpacecraft):
        line, = ax.plot(r_BN[i][:, 0] * m2km, r_BN[i][:, 1] * m2km, r_BN[i][:, 2] * m2km,
                label="Spacecraft " + str(i),
                c=unitTestSupport.getLineColor(i, numberSpacecraft))
        lines.append(line)
    ax.set_box_aspect((np.ptp(r_BN[i][:, 0]), np.ptp(r_BN[i][:, 1]), np.ptp(r_BN[i][:, 2])))
    ax.scatter(0, 0, 0, c=color_x)
    ax.set_title('Spacecraft Relative Orbits in Hill Frame')
    ax.set_xlabel('$i_r$ [km]')
    ax.set_ylabel('$i_{\theta}$ [km]')
    ax.set_zlabel('$i_h$ [km]')
    ax.legend(loc=2)
    
    # Initialization function
    def init():
        for line in lines:
            line.set_data([], [])
            line.set_3d_properties([])
        return lines
    
    # Update function for animation
    def update(frame):
        for i, line in enumerate(lines):
            # Update the data of the line
            line.set_data(r_BN[i][:frame, 0] * m2km, r_BN[i][:frame, 1] * m2km)
            line.set_3d_properties(r_BN[i][:frame, 2] * m2km)
        return lines
    
    interval = r_BN[0].shape[0]/100
    # Create the animation
    ani = FuncAnimation(fig, update, frames=r_BN[0].shape[0], init_func=init, blit=False, interval=interval)
    return ani
    # plt.show()

def plot_orbital_element_differences(timeData, oed, id=None):
    """Plot the orbital element difference between the chief and another spacecraft."""
    plt.figure(id)
    plt.plot(timeData, oed[:, 0], label="da")
    plt.plot(timeData, oed[:, 1], label="de")
    plt.plot(timeData, oed[:, 2], label="di")
    plt.plot(timeData, oed[:, 3], label="dOmega")
    plt.plot(timeData, oed[:, 4], label="domega")
    plt.plot(timeData, oed[:, 5], label="dM")
    plt.legend()
    plt.xlabel("time [orbit]")
    plt.ylabel("Orbital Element Difference")

