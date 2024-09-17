#!/bin/bash

# List of input paths
input_paths=(
    "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/base_case_1_xr.json"
    "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/base_case_2_xyr.json"
    "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/base_case_3_zr_zrdot.json"
    "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/base_case_4_xyzr.json"
    # "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/control_case_5_yr.json"
    # "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/control_case_6_xyr.json"
    # "dev/MultiSatBskSim/scenariosMultiSat/simInitConfig/control_case_7_xyzr.json"
            )

simTimeHours=1.5 # Hours, one orbit for given orbit in each json for now.

turnOnController=0
# turnOnController=1

simRate=0.1
dataSamplingTimeSec=10.0

# Path to your Python script
python_script="dev/MultiSatBskSim/scenariosMultiSat/MultiSat_test_scenario.py"

# Loop through the input paths and run the Python script
for input_path in "${input_paths[@]}"
do
  echo "Running Python script with input: $input_path"
  # python3 $python_script "$input_path" $simTimeHours $turnOnController
  python3 $python_script "$input_path" $simTimeHours $turnOnController $simRate $dataSamplingTimeSec # Opt for more options when needed.
done