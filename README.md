### Getting Started
This project depends on MoveIt! framework and Gazebo 9. In order to get all necessary packages, run the command:

```Shell
sudo apt-get install ros-kinetic-moveit* 
```

```Shell
sudo apt-get install gazebo9 libgazebo9-dev ros-kinetic-gazebo9*
```

After that, the repository should be cloned with all its submodules:

```Shell
git clone --recurse-submodules https://github.com/tahsinkose/hector-moveit.git
```

Since, Github does not support partial submoduling yet, the package `hector_gazebo_termal_camera` should be deleted after above command is executed.

In order to build the project execute `catkin build`. Since there are roughly 30 packages, build time may be around 5 minutes.  
To run the project, launch consecutively: 
```Shell
roslaunch hector_moveit_gazebo orchyard_navigation.launch
```

```Shell
roslaunch hector_moveit_exploration explore.launch
```
# hector-moveit
Hector Quadrotor with MoveIt! Motion Planning Framework.

This project aims a generic application that can autonomously manipulate a quadcopter with the minimal human-operator intervention.
In order to implement Motion Planning primitives, MoveIt! framework is being used. This work highly inspires from the references outlied below.

## What is New?
The new thing is the usage of MoveIt! code API so as to implement a 3D autonomous navigation stack for a Quadcopter. Referenced projects and the several others not referenced directly but have similar context use only GUI. As known, despite being a moderately skilled one, MoveIt! GUI is there for just testing purposes. For those who seek industrial applications with a really complex architecture (e.g. several drones as swarms or teams, with image processing or machine learning modules), usage of code API is the inevitable task to consider. In that sense, this project possesses quite a big potential and aims to have a frontier role. Any who interested is greatly welcomed to contribute!

## Simulation View
<img src="/images/garden4.jpg" alt="Garden View" width="435" height="435"/><img src="/images/garden5.jpg" alt="Garden View" width="435" height="435"/>
<img src="/images/garden6.jpg" alt="Garden View" width="435" height="435"/><img src="/images/garden7.jpg" alt="Garden View" width="435" height="435"/>
<img src="/images/garden_sky1.jpg" alt="Garden View" width="435" height="435"/><img src="/images/garden_sky2.jpg" alt="Garden View" width="435" height="435"/>
### Exploration v1
[![Orchard Exploration](http://img.youtube.com/vi/ZWn9N9Y_tb8/0.jpg)](https://www.youtube.com/watch?v=ZWn9N9Y_tb8 "Orchard Exploration")

### Exploration v2
Exploration version 2 is explained in further detail in the Development log. In summary, missions that were postponed into this version are implemented except one particular mission. According to the log, orientation fixation and trajectory action is succesfully implemented. Only task delayed to version 3 is the grid approach. Also, a very simple metrics is devised to quantify the success of the stack itself. Initial statistics are outlined in <a href="https://discourse.ros.org/t/precision-agriculture-simulation-with-a-quadcopter-in-gazebo/6025/5?u=tahsin_kose">this comment</a>. They are very important and will provide the basis for enhancements on version 3.

### Exploration v3

[![Orchard Exploration v3](http://img.youtube.com/vi/-Hrpk1CQATs/0.jpg)](https://www.youtube.com/watch?v=-Hrpk1CQATs "Orchard Exploration Version 3")

In this version, the last proposed feature is implemented, i.e. there is a grid heuristics in which the drone does not navigate back into the cells that it previously visited. I have observed an inversely linear relation between the grid size and exploration percentage in 10 minutes. For example, for a 25x25 grid, the exploration rate was at most 32% whereas, in 15x15 case it boosted up to 37% degrees. In the final case, 13x13 grid allowed the drone to explore 40.7% of the volume in 10 minutes. With much more advanced heuristics, it is possible to increase this number.

Consequently, a new RViz panel for Exploration stack is developed in which the grid can be interactively inspected and the exploration rate can be observed. In future, there might be some user interface logic for particular areas to be investigated, but in the foreseeable future it is not on the To-Do list.

<img src="/images/orchard_exploration_v3_gui.png" alt="Exploration GUI" width="435" height="435"/><img src="/images/orchard_exploration_v3_gui2.png" alt="Exploration GUI" width="435" height="435"/>

In order to welcome newcomers, status quo of simulation is the best advertisement. For further detail, go to Development log.

### Object Detection v1
It has officially started! I will try to implement a ROS node that detects and collects valuable information (centers in 3D WCS, their dimensions) of trees and fruits. Current candidate is YOLO. I aim to produce a custom dataset consisting of snapshots from simulation view.

## References
<a href="https://github.com/wilselby/ROS_quadrotor_simulator">ROS Quadrotor Simulator by Wil Selby</a>

<a href="https://github.com/AlessioTonioni/Autonomous-Flight-ROS">Autonomous Flight by Alessio Tonioni</a>

<a href="https://github.com/RiccardoGrin/darknet">Darknet Retraining with Custom Dataset by Riccardo Grin</a>

_**Note**: If you know other similar projects that have the same context with the proposed ability, please inform me so that I can list them as a reference._
