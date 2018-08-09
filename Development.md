## Development
Since we are adapting Hector quadcopter into MoveIt! framework, there are various things that we have to consider differently than normal MoveIt! way.

* There exists only one joint, which is *virtual_joint*. Joint\_state\_publisher checks URDF to parse real joints. On the contrary, virtual joint is a semantic information that is defined in SRDF as suggested by the initial work of Wil Selby. Therefore, we would not be able to see any output on /joint_states topic.
    
* Having considered that the joint states are not available to MoveIt! API, we have to come up with a workaround. Actually, I have one already! MoveIt! does not always strictly require `joint_states` topic to be available. Hence, we can directly use MoveIt! API to plan paths with the constraint of keeping `start_state` valid.

Two items above stresses important points of the project. Firstly, there are various similar projects on the web; but none of them exploits the MoveGroup API through code. They just use GUI, and the things that can be made through GUI are very limited. For people who look for real applications that use these technologies, this project possesses quite a good potential. Note that, allowing code API to work as in the GUI is a non-trivial task and corresponding commit(s) and issue(s) should be inspected.
