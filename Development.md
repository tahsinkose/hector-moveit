## Development
Since we are adapting Hector quadcopter into MoveIt! framework, there are various things that we have to consider differently than normal MoveIt! way.

* There exists only one joint, which is *virtual_joint*. Joint\_state\_publisher checks URDF to parse real joints. On the contrary, virtual joint is a semantic information that is defined in SRDF as suggested by the initial work of Wil Selby. Therefore, we would not be able to see any output on /joint_states topic.
    
* Having considered that the joint states are not available to MoveIt! API, we have to come up with a workaround. Actually, I have one already! MoveIt! does not always strictly require `joint_states` topic to be available. Hence, we can directly use MoveIt! API to plan paths with the constraint of keeping `start_state` valid.
