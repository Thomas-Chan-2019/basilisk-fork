 # Basilisk Thesis Notes

 ## Run basic simulations by:
source .venv/bin/activate
python3 scenarioBasicOrbitStream.py

## Viz-support:
Viz Socket: [tcp://localhost:5556](tcp://localhost:5556)

### Loading other CAD files for visualization:
[scenarioDataToViz.py](examples/scenarioDataToViz.py) shows how (line 178 / https://hanspeterschaub.info/basilisk/Vizard/VizardGUI.html#import-a-custom-shape-model)

Example: 
```
viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scList
                                                  , saveFile=fileName
                                                  , liveStream=True
                                                  )
# Enable "liveStream=True" for Viz tool to work!
# Enable "saveFile=fileName"
```

# Basilisk video takeaways:
- `scSim/Process/scObject` as SC, add the essential lines as covered in the Basic orbit case + specify moment of inertias (as Astrobee)
- Gravity factory `gravFactory` -> create Earth or other planets, map these using the line to attach Gravity model to SC (photo underlined)
- From video Basilisk: attitude feedback with RW devices: RW factory -> can specify RW spec here, see [scenarioAttitudeFeedbackRW.py](examples/scenarioAttitudeFeedbackRW.py) (probably same for thruster?); create 3 wheels with same/different initial speed and direction; seems also can map with VoltageIO, check if can related to power consumption/EPS things
- Orbital motions module [orbitalMotion.py](dist3/Basilisk/utilities/orbitalMotion.py) for state vectors r,v & rotational + attitude sigma & omega (?)
- Always use line: scSim.AddModelToTask() and add all used objects, gravity bodies, RWEffectors, etc for the the logging to work


Useful Basilisk *utilities* modules:
- [RigidBodyKinematics.py](dist3/Basilisk/utilities/RigidBodyKinematics.py): EP, Gibbs vector (or Classical Rodrigues Parameters), MRP, RV, Euler angles conversions***
- [orbitalMotion.py](dist3/Basilisk/utilities/orbitalMotion.py): planetary constants, state r,v to element & vice versa, Atmospheric Drag functions (need c_D, area, mass inputs), J-Perturbations of Earth & diff. planets, Solar Radiation pressure {***Mind the units when use them, some uses km/s^2 !}, Orbital Element conversions (classicial <=> equinoctial <=> hill frame <=> RV)
- [astroFunctions.py](dist3/Basilisk/utilities/astroFunctions.py): 
*To create thrusters/RWs:*
- [simIncludeThruster.py](dist3/Basilisk/utilities/simIncludeThruster.py) & [simIncludeRW.py](dist3/Basilisk/utilities/simIncludeRW.py), see files for interfacing.
