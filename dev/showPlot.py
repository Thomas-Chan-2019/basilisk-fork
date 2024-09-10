import mplfig
import matplotlib.pyplot as plt
import pylustrator

plotPath = "dev/MultiSatBskSim/ResultPlots/control_case_5_yr/index_1/" # Replace with absolute path when necessary.
# plotPath = "dev/MultiSatBskSim/ResultPlots/control_case_7_xyzr/index_1/" # Replace with absolute path when necessary.

# pylustrator.load(plotPath+"/figure_9.png")
# pylustrator.load(plotPath+"/figure_10.png", offset=[1, 0])
def showPlotsFromMPL():
    # This blank plot is to remove a buggy plot at `mplfig`'s module initialization...
    figBug = mplfig.load_figure(plotPath+"blank.mplpkl") # Load the saved figure    
    
    # Load plots for plotting:
    fig = mplfig.load_figure(plotPath+"figure_1.mplpkl") 
    fig2 = mplfig.load_figure(plotPath+"figure_2.mplpkl") 
    _ = mplfig.load_figure(plotPath+"figure_3.mplpkl") 
    _ = mplfig.load_figure(plotPath+"figure_4.mplpkl") 
    _ = mplfig.load_figure(plotPath+"figure_5.mplpkl") 
    _ = mplfig.load_figure(plotPath+"figure_6.mplpkl") 
    # fig7 = mplfig.load_figure(plotPath+"figure_7.mplpkl") 

    # _ = mplfig.load_figure(plotPath+"/figure_8.mplpkl") 

    # Change the axis as wanted:
    ax2 = fig2.get_axes()
    ax2[0].set_title('$Test^{3}$')
    
    # ax7 = fig7.get_axes()
    # ax7[0].set_title('$Test^{2}$')
    # fig7.suptitle("This is a somewhat long figure title", fontsize=16)

    plt.close(figBug)
    plt.show()
    return

# DONE - if use MATLAB plot, go to dev/showPlot.m:
r"""

from scipy.io import savemat

# Create a dictionary to store your data
data_dict = {
    'time': time,
    'data1': data1,
    'data2': data2,
    'data3': data3
}

# Save the data to a .mat file
savemat('data.mat', data_dict)

# In a new MATLAB plotting file:
% Load the data
data = load('data.mat');

% Access the variables
time = data.time;
data1 = data.data1;
data2 = data.data2;
data3 = data.data3;

% Plotting examples
figure;
plot(time, data1);
title('Plot of data1 against time');

figure;
plot(time, data2(:,1), time, data2(:,2));
title('Plot of data2 columns against time');
"""


if __name__ == "__main__":
    showPlotsFromMPL()