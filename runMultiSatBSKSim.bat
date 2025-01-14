@echo off
:: Check if the number of arguments is correct
if "%~1"=="" (
    echo No arguments provided. Exiting.
    exit /b 1
)

:: Capture the arguments passed from the shell script
set input_paths=%~1
set simTimeHours=%~2
set turnOnController=%~3
set simRate=%~4
set dataSamplingTimeSec=%~5

:: Convert space-separated paths into a batch-friendly list
for %%i in (%input_paths%) do (
    echo Running Python script with input: %%i
    python dev\MultiSatBskSim\scenariosMultiSat\MultiSat_test_scenario.py --path %%i --simTime %simTimeHours% --control %turnOnController% --simRate %simRate% --sampleTime %dataSamplingTimeSec%
)

pause
