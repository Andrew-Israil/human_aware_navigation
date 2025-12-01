This work presents a human-aware navigation framework for an Autonomous Mobile
Robot (AMR) in a warehouse setting, implemented using the ROS2 navigation stack.
Unlike conventional collision avoidance methods that rely on adding contours around
obstacles detected by laser scanners, this approach integrates data from visual human
detection and tracking algorithms. It predicts the trajectories of nearby humans and
checks for potential collisions with the AGV’s path.

To include this info in the navigation process, the framework uses the layered costmap
concept in NAV2 by adding a custom costmap layer for social navigation. This layer
penalizes the vehicle’s traversal through predicted human trajectories, considering uncertainties,
potential collisions, and human proxemic zones. The cost in this layer is
added to the global costmap, making the path planner aware of the human motion.
In this work, the human-aware global planner approach was adopted to benefit from
its long planning horizon. To compensate the non-temporal planners’ limitations in
dynamic environments, the decision-making system was utilized to control the planning
and execution sequence, adjusting the vehicle’s behavior based on the situation. The
final outcome is a modular structure that can function with any type of global and local
planners.

The data flow between the implemented ROS nodes and servers is shown in the following diagram

![alt text](https://github.com/Andrew-Israil/human_aware_navigation/blob/main/system_architecture.png)

