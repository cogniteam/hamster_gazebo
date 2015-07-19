Simulation environment
======================
---

### SYNOPSIS
---
```bash
rosrun mrm_sim simulation_launcher.sh [-r=NUMBER_OF_ROBOTS] 
                                      [-s=SCENARIO_NAME] 
                                      [-rp=x1,y1;x2,y2;x3,y3;...]
```

### DESCRIPTION
---
Loads the simulation environment.
*Note: To close the simulation, type **Q** and hit enter.

### OPTIONS
---
**-r=NUMBER_OF_ROBOTS**
> Number of robots to load, the value must be between 1 and 10 inclusive.

**-s=SCENARIO_NAME**
> Scenario name to load. A scenario consists of two things: blobs count and positions and the malfunctions lists.
> To create a scenario, create malfunctions-SCENARIO_NAME.yaml and blobs-SCENARIO_NAME.yaml files in
> **mrm_sim/config** directory.

**-rp=ROBOT_POSITIONS**
> The initial positions of the robots

### EXAMPLES
Load scenario named **test** with 3 robots, at positions (0,0), (1,0) and (2,0)
```bash
rosrun mrm_sim simulation_launcher.sh -r=3 -s=test -rp=0,0;1,0;2,0
```
