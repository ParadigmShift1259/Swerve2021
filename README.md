<img src="logo.png"
     alt="Paradigm Shift logo"
     align="right"
     style="margin-right: 10px; margin-top: 80px" />

# Swerve 2021
### Paradigm Shift #1259

Templates for Falcon/NEO or double NEO swerve drive trains.  
Falcon/NEO is on [**main**](https://github.com/ParadigmShift1259/Swerve2021), double NEO is on branch [**NEO**](https://github.com/ParadigmShift1259/Swerve2021/tree/NEO)

### Doxygen documentation for main branch using Travis can be found [here](https://paradigmshift1259.github.io/Swerve2021).


## Running Paths
Team 1259 is currently using Team 3015's PathPlanner software to generate swerve paths.  
PathPlanner generates a C++ array which is converted into an frc::Trajectory, an object that carries desired states via timestamps for our Auto runs.  
### Setup
- Download PathPlanner latest release (2021 uses **v1.6**) executable for Windows `pathplanner-win.exe` from [**here**](https://github.com/mjansen4857/pathplanner/releases/tag/v1.6.0) and install the .exe
- Read through the [**Path README**](PathREADME.md) (Copied from PathPlanner v1.6)
- Get example swerve paths for the 2021 robot [**here**](https://github.com/ParadigmShift1259/FRC_Robot_2021/tree/paths)
### Config
- Open up PathPlanner
- In the top left settings menu, set the following:

| Name                  	| Value                       	|
|-----------------------	|-----------------------------	|
| Team Number           	| 1259                        	|
| RoboRIO Path Location 	| (Don't change)              	|
| Units                 	| Metric (will be in meters)  	|
| Game Year             	| Your year                   	|
| Max Velocity          	| Robot dependent - calculate 	|
| Max Acceleration      	| Robot dependent - calculate 	|
| Wheelbase Width       	| Robot dependent - calculate 	|
| Robot Length          	| Robot dependent - calculate 	|
| Time Step             	| 0.02                        	|
| Drive Train           	| Holonomic                   	|

<br/>

- In the bottom left menu, click Generate Path (ctrl+G) to create the C++ array. Set the following:

| Name            	| Value                   	|
|-----------------	|-------------------------	|
| Output Type     	| C++ array               	|
| Path Name       	| Name of autonomous path 	|
| Output Format   	| t,v,a,X,Y,hh            	|
| Reversed Output 	| false (Off)             	|
| Split Path      	| false (Off)             	|

<br/>

- You should now be able to generate example paths from 2021 or your own paths!
