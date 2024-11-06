#!/bin/bash

# List of input paths (common to both Linux/MacOS and Windows)
input_paths=(
    # "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/base_case_1_xr.json"
    # "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/base_case_2_xyrdot.json"
    # "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/base_case_3_zr_zrdot.json"
    # "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/base_case_4_xyzr.json"
    "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/control_case_5_yr.json"
    "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/control_case_6_xyr.json"
    "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/control_case_7_xyzr.json"
    "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/control_case_8_xyzr_dot.json"
)

simTimeHours=1.5 # Hours, one orbit for given orbit in each json for now.

# turnOnController=0
turnOnController=1

simRate=0.1
dataSamplingTimeSec=0.1

# Path to your Python script (common to both platforms)
python_script="dev/MultiSatBskSim/scenariosMultiSat/MultiSat_test_scenario.py"

# Detect the OS
OS_TYPE=$(uname)

if [[ "$OS_TYPE" == "Linux" || "$OS_TYPE" == "Darwin" ]]; then
    echo "Running on Linux/MacOS"
    
    # Loop through the input paths and run the Python script
    for input_path in "${input_paths[@]}"
    do
      echo "Running Python script with input: $input_path"
      # python3 $python_script "$input_path" $simTimeHours $turnOnController
      python3 "$python_script" "$input_path" "$simTimeHours" "$turnOnController" "$simRate" "$dataSamplingTimeSec" # Opt for more options when needed.
    done

elif [[ "$OS_TYPE" == *"MINGW"* || "$OS_TYPE" == *"CYGWIN"* ]]; then
    echo "Running on Windows using the batch script"

    # Convert array to a space-separated string for passing to the batch script
    input_paths_string=$(IFS=" "; echo "${input_paths[*]}")

    # Call the batch script with all the variables passed as arguments
    cmd.exe /c run_simulation.bat "$input_paths_string" "$simTimeHours" "$turnOnController" "$simRate" "$dataSamplingTimeSec"
else
    echo "Unsupported OS detected."
    exit 1
fi
