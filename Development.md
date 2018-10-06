## Development
Since we are adapting Hector quadcopter into MoveIt! framework, there are various things that we have to consider differently than normal MoveIt! way.

* There exists only one joint, which is *virtual_joint*. Joint\_state\_publisher checks URDF to parse real joints. On the contrary, virtual joint is a semantic information that is defined in SRDF as suggested by the initial work of Wil Selby. Therefore, we would not be able to see any output on /joint_states topic.
    
* Having considered that the joint states are not available to MoveIt! API, we have to come up with a workaround. Actually, I have one already! MoveIt! does not always strictly require `joint_states` topic to be available. Hence, we can directly use MoveIt! API to plan paths with the constraint of keeping `start_state` valid.

Two items above stresses important points of the project. Firstly, there are various similar projects on the web; but none of them exploits the MoveGroup API through code. They just use GUI, and the things that can be made through GUI are very limited. For people who look for real applications that use these technologies, this project possesses quite a good potential. Note that, allowing code API to work as in the GUI is a non-trivial task and corresponding commit(s) and issue(s) should be inspected.

---

### Perception
From the system design point of view, there is only a Kinect2 sensor attached to the base of Quadcopter with downward orientation. In fact, any stereo camera with a good PointCloud2 generation ability would be adequate. 
<a href="http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/perception_configuration.html">MoveIt! Perception</a> requires sensors that produce either Point Cloud or Depth Image of the environment. There is no need to provide both sensory input to the pipeline, since either would suffice. When this configuration is made, OccupanyMapUpdater Plugin of MoveIt! constructs an <a href="http://octomap.github.io/">OctoMap</a> representation of the environment as robot moves. OctoMap is only a mapping approach, it is **not** a **SLAM** method. It is an efficient way of storing and manipulating 3D maps. In fact, this is reasonable for MoveIt! since it does not have to localize its robots because it was mainly developed for manipulators. For simulation purposes at the moment, it is not yet required to have Localization ability. But in the future, a real application definitely would require one. 

As Wil Selby had briefly illustrated in its <a href="https://www.wilselby.com/research/ros-integration/3d-mapping-navigation/">3D Navigation Tutorial</a>, <a href="http://introlab.github.io/rtabmap/RTAB-Map"> RTAB-Map</a> is a good SLAM candidate. It optionally requires an additional Laser Scanner for better performance. In <a href="http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot">SetupOnYourRobot</a> webpage, it is denoted that there should be at least a Kinect-like sensor. Therefore, it is a Stereo camera (RGB-D) SLAM approach. There is a <a href="https://github.com/Notou/Moveit-External-Octomap-Updater">custom Octomap updater plugin for MoveIt!</a> written by @Notou, which enables the OctoMap output from RTAB-Map to be directly fed into MoveIt! Pipeline.

In below, related papers are added for future reference. More will be added (hopefully) as project proceeds.

---

### Scenarios
#### Orchard fruit picking with programmatic environment generation
In order to have a particular aim for the project, some scenarios should be devised. For that purpose, a garden with apple trees is the first candidate. In that world, the drone first try to explore the space and then picks the fruits that it recognized. There will be Exploration, Image Recognition, Fruit Manipulation and Path Planning tasks in technical aspect. 

After determining what to do, the next task was to develop (if there exists already, directly use) a proper world. Actually, there are a bunch of decent 3D models in the <a href="https://3dwarehouse.sketchup.com/">Sketchup Warehouse</a> with COLLADA extension. They are directly usable in the Gazebo environment. Unfortunately, all the trees with fruits have problematic rendering in Gazebo. Meaningly, only one fruit is being rendered and the others are not. I couldn't find any relevant bug, thus any fix. Even further, it might be a machine-dependent problem that only affects me. Nevertheless, I decided to come up with a work-around. That is, current implementation reads the mesh file attached to the tree model, fetchs all vertices and randomly cherry picks some fixed amount of them (in my case, it is 50). After extracting which positions to place apples, a Gazebo plugin dynamically spawns the apples into world. Despite the apples are generated non-perfectly (i.e. not perfectly aligned with branches), it produces a good approximation to a tree with fruits.

<img src="/images/garden1.jpg" alt="Garden View" width="435" height="435"/><img src="/images/garden2.jpg" alt="Garden View" width="435" height="435"/>
<img src="/images/garden3.jpg" alt="Garden View"/>

#### Orchard fruit picking with static environment generation
In order to avoid any confusion, static relates in here to simple world files, i.e. all the models seen in the environment (except the quadcopter) are generated beforehand through their declaration in the world file. This reduces a lot of complexity with respect to the above approach. However, the apples are not distinct entities in this case; so, fruit-picking might be more difficult. One work-around that I can thought of is to spawn instantaneous distinct apples at the location where a fruit is detected to be picked. 

All the models and world files are separated into a brand-new package. Therefore, they can be used in any project with agricultural purposes. In future, it is aimed to create new ones particular to the tasks and improve current ones with further details. I mentioned about the problematic tree generation with fruits in the previous section. That's why I dived into Blender and created my own apple tree model and exported COLLADA file to use in Gazebo. One problem with the Blender COLLADA generation is that it drops transparency of meshes during exportation process. **[This is a global problem, many developers from the communities of Unity and other game development frameworks reported that, but I couldn't find a way out. Since I'm a too newbie to Blender and 3D modelling, I gave up and looked for work-arounds]**. Therefore all meshes of the tree had a black background. I have resolved that with the Gazebo's own transparency support. Even with a very small number (e.g. 1e-5), black background vanishes and all remains is the tree itself. Nevertheless, tree has a tiny amount of transparency that can be distinguished when looked from very close distance.

<img src="/images/garden4.jpg" alt="Garden View" width="435" height="435"/><img src="/images/garden5.jpg" alt="Garden View" width="435" height="435"/>
<img src="/images/garden6.jpg" alt="Garden View" width="435" height="435"/><img src="/images/garden7.jpg" alt="Garden View" width="435" height="435"/>

Resolution of the trees might be a problem during the fruit detection. I will decide whether to increase the resolution of the model based on the accuracy of detection algorithm.

---
### Exploration

After the simulation environments are ready, first goal was to properly, safely and of course autonomously navigate in the orchard so as to map it comprehensively. Current architecture uses MoveIt! and <a href="http://docs.ros.org/kinetic/api/hector_quadrotor_actions/html/classhector__quadrotor__actions_1_1PoseActionServer.html">Pose Action Server</a> for simultaneous planning and mapping. MoveIt! has a simple motion planning pipeline that can be used to integrate Octomap data generated via Kinect2 Pointcloud input and motion planner plugins provided by OMPL. Furthermore, one can also implement an Execute Path Service plugin as a controller that MoveIt! can talk to. For now, I only use the planners to extract valid paths to the goal state and feed them into Pose Action Server special to the Hector Quadcopter platform. In future, I'm aiming to provide a generic controller than can generalize over different platforms.

#### Version 1
[![Orchard Exploration](http://img.youtube.com/vi/ZWn9N9Y_tb8/0.jpg)](https://www.youtube.com/watch?v=ZWn9N9Y_tb8 "Orchard Exploration") As it can be seen, it successfully replans after noticing the obstacles popped out in the path of latched trajectory. It is definitely a dynamic collision avoidance routine and can respond under 1 second. My aim for version 2 is to implement:
* An orientation fixer for the Quadcopter. Since Kinect (or any other stereo camera) does not have 360 degree of FOV, the velocity vector and the orientation of the camera should be equal. Otherwise, drone might not see its front exactly; thus couldn’t notice the obstacle in front and thinks the latched trajectory is still valid, whereas it isn’t.

* A velocity controller or directly using existing alternatives, if any. By this, motions will be much smoother with respect to position control.

* A Frontier approach that determines the next goal from extracted frontiers. Currently, the goals are hardcoded in a way that traverses the faces and corners of the rectangular volume.

#### Version 2
- [x] Orientation Fixation is now perfectly handled.

- [x] Velocity controller produces motion commands according to whole trajectory instead of separate waypoints.

- [ ] A Grid-based frontier approach that uses the frontiers chosen randomly with respect to their distances is determined to be implemented for Version 3. 

### References
* <a href="https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2005/projects/1aslam_blas_repo.pdf">SLAM for Dummies</a>

* <a href= "https://vision.in.tum.de/_media/spezial/bib/caruso2015_omni_lsdslam.pdf">Omnidirectional LSD-SLAM</a>

* <a href= "https://vision.in.tum.de/_media/spezial/bib/engel14eccv.pdf"> Direct Monocular LSD-SLAM</a>

* <a href= "http://www2.informatik.uni-freiburg.de/~hornunga/pub/hornung13auro.pdf"> OctoMap</a>

* <a href= "http://www2.informatik.uni-freiburg.de/~endres/files/publications/endres13tro.pdf"> RGB-D SLAM </a>

* <a href= "https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/e/eb/Labbe14-IROS.pdf"> RTAB-Map </a>
---

_For the scope of the project, this configuration is enough. However, there are many advanced and promising technologies out there. For example, I'm making a research to remove the limitation of Stereo camera. There are SLAM variants that work with monocular cameras as well. <a href="https://vision.in.tum.de/research/vslam/lsdslam">LSD-SLAM</a> developed by TUM is one prominent cutting-edge candidate. It works even with an embedded smartphone camera._
