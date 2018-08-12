## Development
Since we are adapting Hector quadcopter into MoveIt! framework, there are various things that we have to consider differently than normal MoveIt! way.

* There exists only one joint, which is *virtual_joint*. Joint\_state\_publisher checks URDF to parse real joints. On the contrary, virtual joint is a semantic information that is defined in SRDF as suggested by the initial work of Wil Selby. Therefore, we would not be able to see any output on /joint_states topic.
    
* Having considered that the joint states are not available to MoveIt! API, we have to come up with a workaround. Actually, I have one already! MoveIt! does not always strictly require `joint_states` topic to be available. Hence, we can directly use MoveIt! API to plan paths with the constraint of keeping `start_state` valid.

Two items above stresses important points of the project. Firstly, there are various similar projects on the web; but none of them exploits the MoveGroup API through code. They just use GUI, and the things that can be made through GUI are very limited. For people who look for real applications that use these technologies, this project possesses quite a good potential. Note that, allowing code API to work as in the GUI is a non-trivial task and corresponding commit(s) and issue(s) should be inspected.

---

### Perception
From the system design point of view, there is only a Kinect2 sensor attached to the base of Quadcopter with downward orientation. In fact, any stereo camera with a good PointCloud2 generation ability would be adequate. 
<a href="http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/perception_configuration.html">MoveIt! Perception</a> requires sensors that produce either Point Cloud or Depth Image of the environment. There is no need to provide both sensory input to the pipeline, since either would suffice. When this configuration is made, OccupanyMapUpdater Plugin of MoveIt! constructs an <a href="http://octomap.github.io/">OctoMap</a> representation of the environment as robot moves. This is a 3D SLAM method.

_For the scope of the project, this configuration is enough. However, there are many advanced and promising technologies out there. For example, I'm making a research to remove the limitation of Stereo camera. There are SLAM variants that work with monocular cameras as well. <a href="https://vision.in.tum.de/research/vslam/lsdslam">LSD-SLAM</a> developed by TUM is one prominent cutting-edge candidate. It works even with an embedded smartphone camera.
