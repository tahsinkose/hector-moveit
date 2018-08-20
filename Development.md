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

### Scenarios
#### Orchard fruit picking
In order to have a particular aim for the project, some scenarios should be devised. For that purpose, a garden with apple trees is the first candidate. In that world, the drone first try to explore the space and then picks the fruits that it recognized. There will be Exploration, Image Recognition, Fruit Manipulation and Path Planning tasks in technical aspect. 

After determining what to do, the next task was to develop (if there exists already, directly use) a proper world. Actually, there are a bunch of decent 3D models in the <a href="https://3dwarehouse.sketchup.com/">Sketchup Warehouse</a> with COLLADA extension. They are directly usable in the Gazebo environment. Unfortunately, all the trees with fruits have problematic rendering in Gazebo. Meaningly, only one fruit is being rendered and the others are not. I couldn't find any relevant bug, thus any fix. Even further, it might be a machine-dependent problem that only affects me. Nevertheless, I decided to come up with a work-around. That is, current implementation reads the mesh file attached to the tree model, fetchs all vertices and randomly cherry picks some fixed amount of them (in my case, it is 50). After extracting which positions to place apples, a Gazebo plugin dynamically spawns the apples into world. Despite the apples are generated non-perfectly (i.e. not perfectly aligned with branches), it produces a good approximation to a tree with fruits.
### References
* <a href="https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2005/projects/1aslam_blas_repo.pdf">SLAM for Dummies</a>

* <a href= "https://vision.in.tum.de/_media/spezial/bib/caruso2015_omni_lsdslam.pdf">Omnidirectional LSD-SLAM</a>

* <a href= "https://vision.in.tum.de/_media/spezial/bib/engel14eccv.pdf"> Direct Monocular LSD-SLAM</a>

* <a href= "http://www2.informatik.uni-freiburg.de/~hornunga/pub/hornung13auro.pdf"> OctoMap</a>

* <a href= "http://www2.informatik.uni-freiburg.de/~endres/files/publications/endres13tro.pdf"> RGB-D SLAM </a>

* <a href= "https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/e/eb/Labbe14-IROS.pdf"> RTAB-Map </a>
---

_For the scope of the project, this configuration is enough. However, there are many advanced and promising technologies out there. For example, I'm making a research to remove the limitation of Stereo camera. There are SLAM variants that work with monocular cameras as well. <a href="https://vision.in.tum.de/research/vslam/lsdslam">LSD-SLAM</a> developed by TUM is one prominent cutting-edge candidate. It works even with an embedded smartphone camera._
