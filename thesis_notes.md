Run basic simulations by:
source .venv/bin/activate
python3 scenarioBasicOrbitStream.py

Viz-support:
Socket: 
tcp://localhost:5556

Loading other CAD files for visualization:
scenarioDataToViz.py shows how (line 178 / https://hanspeterschaub.info/basilisk/Vizard/VizardGUI.html#import-a-custom-shape-model)
e.g. viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scList
                                                  , saveFile=fileName
                                                  , liveStream=True
                                                  )
{Enable "liveStream=True" for Viz tool to work!}
{Enable "saveFile=fileName"}

Basilisk video takeaways:
- scSim/Process/scObject as SC, add the essential lines as covered in the Basic orbit case + specify moment of inertias (as Astrobee)
- Gravity factory gravFactory -> create Earth or other planets, map these using the line to attach Gravity model to SC (photo underlined)
- From video Basilisk: attitude feedback with RW devices: RW factory -> can specify RW spec here, see code 2nd photo (scenarioAttitudeFeedbackRW.py, probably same for thruster?); create 3 wheels with same/different initial speed and direction; seems also can map with VoltageIO, check if can related to power consumption/EPS things
- Orbital motions module for state vectors r,v & rotational + attitude sigma & omega
- Always use line: scSim.AddModelToTask() and add all used objects, gravity bodies, RWEffectors, etc for the the logging to work