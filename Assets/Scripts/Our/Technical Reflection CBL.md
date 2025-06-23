\# Technical Reflection

\#\# Lab Login Details

Username: \`team28\`

Password: \`AnythingCreative\!3===\`

\#\# 1 General Notes

While we demonstrate the feasibility of a BOB prototype through our proof of concept, multiple technical areas could be improved with more time and resources. 

Our primary challenge from the start was with the development environment. At first, we succeeded with Docker, which was chosen for its native performance with the applications we needed, such as Unity, while also providing an environment for ROS 2\. However, due to the lack of official support from Robotis for Docker with TurtleBot3 and online guides/demonstrations being very superficial, we were unable to achieve a permanently working solution. Our setup for the first few weeks suddenly broke and began to show errors related to an incorrect GPG key for one of the ROS 2 repositories (GNU Privacy Guard key, used for authentication and security reasons during package installation on Ubuntu), rendering our Docker solution unbuildable. Furthermore, even when our Docker setup worked, simulating the robot in Unity proved ineffective as bugs (which we suspect are related to the Unified Robot Description Format or URDF) gave us issues when running the project provided in Canvas, and caused mapping via the cartographer to immediately crash.

This resulted in us having to scrap a virtual city environment we built for testing in Unity. After experiencing so many difficulties with Docker, we switched to the Ubuntu Virtual Machine (VM) provided in Canvas. In its original state, this was unacceptably slow, taking up to a minute to load the Unity dependencies in VS Code. We eventually remedied this issue by tweaking the VM (manually allocating more disk space, disabling OS power throttling on VMware Workstation, disabling Hyper-V, enabling VT-x in the BIOS of the host machine, and disabling core isolation, despite it being inadvisable), which significantly improved the effectiveness of our setup.

In the end, for development, we resorted to using Gazebo to simulate the robot itself and Unity to act as the digitally twinned environment (which handles visualizing the robot, its states, and the environment it sees as well as path planning and handling of virtual obstacles added post factum). In practice, the TurtleBot3 would replace Gazebo and communicate with Unity (our digitally twinned environment). Despite the aforementioned recurring challenges, which slowed progress for the first weeks, we believe that the foundations for bidirectional communication, state synchronization, and environmental/object interaction were achieved.

\#\# 2 Bidirectional Communication

\*\*Goal:\*\* Establish a data exchange between the real and virtual TurtleBot3 using ROS 2 and a simulation platform (e.g., Gazebo or Unity).

\#\#\# 2.1 What Was Achieved

\- \*\*We established bidirectional communication:\*\* The physical TurtleBot3 communicates with the Unity-based digital twin using ROS 2 and the ROS TCP Connector Unity package.

\- \*\*Goal pose commands from Unity were successfully sent to the TurtleBot3:\*\*   
In accordance with FR-3 (user inputs destination). Specifically, clicking a point in the Unity environment publishes a goal pose, after which the path to that point will be calculated and the physical TurtleBot3 will take the calculated path to the equivalent point in the physical world.  

\- \*\*The digital twin reflects the changes in the real environment:\*\* Environmental changes detected by the TurtleBot3’s LiDAR sensor were sent back into the digital twin and used to map the surrounding space in near-real time. This covers FR-1 (Detect obstacles).

\- \*\*Functional Requirements FR-4 (Navigates to target position) and FR-5 (Send obstacle information to the digital twin)\*\*:. Obstacle data refers to locations of obstacles in relation to the TurtleBot3.

\#\#\# 2.2 What Was Not Completed

\- \*\*Data logging was never implemented\*\*:  NFR-3 (logging of data) was not achieved in a meaningful way.

\#\#\# 2.3 Bottlenecks / Challenges

\- \*\*Had to debug cartography/navigation nodes for longer than expected:\*\* Due to issues relating to our setup (See General section), we were stuck debugging all of the nodes relevant to cartography and navigation for a week. 

\- \*\*Wasted one lab session (external circumstances):\*\* Due to the incorrect shutdown of the robot by a previous group, one lab session was wasted because we were unable to connect and test the navigation/cartographer stacks. 

\- \*\*Time constraints hindered progress on a unified data logging solution (for NFR-3):\*\* This solution would have encompassed all the useful data generated when using the robot.

  

\#\#\# 2.4 Improvements & Expansions

\- \*\*Full data logging capabilities to the proof of concept:\*\* With more time, we could add this for research and debugging purposes. For example, this could include logging user and robot positions; accessibility data, and the density of surrounding crowds and environments. All of this information can be written to CSV or JSON files. If this system reaches end users, a user agreement will also have to be made to enable the collection of such data under data protection laws (e.g. GDPR). 

\#\# 3 State Synchronization

\*\*Goal:\*\* Ensure the robots remain in a consistent state (between the physical and virtual robots), including movements, sensor data, and operational conditions with real-time (or near-real-time) responsiveness.

\#\#\# 3.1 What Was Achieved

\- \*\*We synchronized the robot’s physical position and orientation\*\*: Between the digital twin and the TurtleBot3 in near real-time.

\- \*\*Synchronization was improved by using the odometry topic\*\*: It provides the robot’s relative position to its environment (via LiDAR data) as opposed to simulating the robot’s movement by applying forces based on velocity topics. The latter approach significantly reduces desynchronization between the TurtleBot3 and simulation, caused by discrepancies between environments and cumulative errors from sensors and wireless data transfer.

\- \*\*The digital twin can estimate navigation range in accordance with FR-7 (track battery and estimate navigation range) from a mocked battery percentage:\*\* The mocked data is sent to the virtual environment, which shows the battery percentage and estimates the robot’s movement range. When a goal is specified, the TurtleBot3 only begins moving if it can reach it without running out of battery.  
    
\- \*\*Mirrored wheel rotation:\*\* Done by reading the wheel rotation values from the base link. 

\#\#\# 3.2 What Was Not Completed

\- \*\*We were unable to read the real battery capacity from the robot:\*\* Instead, the navigation range was estimated with a mocked battery percentage, which we tested using a publisher to the /battery\_state topic. This would publish a mock battery state for the subscriber in Unity, which the subscriber could receive successfully. This, however, failed when using battery state data from the real TurtleBot3, despite it supposedly publishing to the same topic.

  

\- \*\*Full mirroring of all internal states, including error states, onto the digital twin\*\* 

\#\#\# 3.3 Bottlenecks / Challenges

\- \*\*Our battery subscriber only worked with the mock publisher:\*\* This happened despite the mock battery publisher and the actual battery publisher from the TurtleBot3 publishing to the same topic. We mitigated this by using the mock publisher to showcase range estimation instead.

\#\#\# 3.4 Improvements & Expansions

\- \*\*Fully synchronizing internal error states and displaying related information:\*\* This would be in the form of notifications in the Unity UI and could be fully implemented within a few more weeks.

\- \*\*Test and debug the battery subscriber:\*\* During future time spent with the hardware, we can perform tests to determine why the subscriber did not successfully gather information from the TurtleBot3, and refine it to work with the actual robot, as we are currently unsure about the cause of the issue.   
  

\#\# 4 Environmental & Object Interaction

\*\*Goal:\*\* Implement a system where both the physical and virtual twin detect and avoid obstacles in a synchronized manner. Demonstrate the ability to interact with objects, ensuring actions are reflected in both environments. The system should accurately reflect actions such as navigating around obstacles or pushing objects.

\#\#\# 4.1 What Was Achieved

\- \*\*Static obstacle avoidance of both physical and virtual sources was implemented and tested:\*\* The implementation of pathfinding utilizes a Unity NavMesh, which updates as physical objects are represented in the virtual environment and as virtual objects are placed in the scene. As such, the shortest path from the robot to a goal (that the user can also pass through) can be found as long as a valid path exists. In the case that there is no way to get to a specific point, a message will be shown saying that the location is unreachable. This covers the static objects part of FR-2.

\- \*\*We demonstrated how data from multiple BOB units can continuously build a digitally twinned environment:\*\* This illustrates the purpose of digital twinning in our solution. In the technical solution, multiple physical robots can update the same virtual environment by adding the obstacles they detect as virtual obstacles, so that other robots will take those obstacles into account in their routes without needing to encounter them directly. Although we could only work with one robot for the proof of concept, we successfully implemented digital object avoidance. This made it possible to add a virtual-only obstacle in Unity such that the physical robot will avoid it as if the obstacle is actually there.

\- \*\*The path can be dynamically recalculated when new obstacles are introduced in Unity or in the real environment:\*\* If the robot’s path is suddenly obstructed by a newly introduced or previously undetected obstacle, whether physical or virtual, the path will be recalculated so that the robot avoids the obstacle. This showcases that the pathing dynamically responds to updates in the environment (both physical and virtual).

\- \*\*The distance from the TurtleBot3 to the user can be estimated using Bluetooth:\*\* When the user and the TurtleBot3 are separated (e.g. the distance between them is greater than 5 meters), the TurtleBot3 will cancel its path and return to the user. This covers FR-6 (return to the user when too far away).

\#\#\# 4.2 What Was Not Completed

\- \*\*The LiDAR data should be used to validate whether virtual obstacles exist in the physical world\*\*: And remove them if they do not exist (there is no need to avoid them if what they represent is no longer there). This was not implemented; instead, a path is generated that avoids the virtual obstacle, regardless of whether it exists in the physical world.

\- \*\*We did not fully achieve user location tracking:\*\* We had to simplify our goals due to Bluetooth Low Energy limitations, namely the lack of directional data with our hardware (an ESP32 microcontroller). Initially, we aimed to determine both the direction and distance from the physical robot to the user using Bluetooth technology. This way, we would always track the user’s location in relation to the TurtleBot3. While we achieved distance tracking, we were unable to implement directional user tracking within the time allocated.

\-  \*\*NFR-1 (reliability in extended operation) cannot be guaranteed:\*\* Since testing for extended periods was not possible within our constraints. We noticed strange mapping behavior from the cartographer, where the map was continuously added to but not deleted from, resulting in a malfunctioning navigation environment after some operation. This only happened with the real robot in lab sessions, not with the Gazebo simulated robot. 

  

\#\#\# 4.3 Bottlenecks / Challenges

\- \*\*Limited information on the type of object that was encountered by the robot:\*\* This is mainly due to hardware constraints (e.g., lack of camera or 3D LiDAR). The lack of height and precise shape data made it impossible to differentiate or classify object types. Initially, we planned to implement a high-level object recognition algorithm based on the shape of objects detected by the LiDAR sensor (mainly to aid in user detection), but we had to reconsider this due to the hardware. The robot is unable to detect obstacles that are below or above the LiDAR sensor’s range (e.g., very short or hanging obstacles). 

\- \*\*Bluetooth Received Signal Strength Indicator (RSSI) was quite noisy:\*\* Mostly since it depended on the environmental conditions (e.g. metallic object interference, signal congestion, etc).

\#\#\# 4.4 Improvements

\- \*\*Improving the path-finding implementation:\*\* This is currently somewhat buggy, and NFR-2 (robustness of route planning) is therefore not fully complete. The robot occasionally stops if it cannot find an immediate path, resulting in confusing behavior. 

\- \*\*Improving the reliability of virtual dynamic obstacle detection:\*\* This issue stems from the latency between when the pathfinding algorithm and the moving obstacle data are received. It is worth noting that it may only be necessary to take virtual objects into account when they are out of the robot’s “view”, generally circumnavigating the issue, as the LiDAR sensor will be able to detect them in the physical environment and verify whether they are still there when the robot reaches them.

\- \*\*Improving Bluetooth hardware (Directional Bluetooth Antenna):\*\* This would allow us to achieve directional user tracking and signal reliability.

\- \*\*Further testing and debugging of Cartographer issues:\*\* This can be done with more time to iron out the strange mapping behaviour. 

\- \*\*Use vision-based detection (e.g., YOLOv12 or OpenCV):\*\* This would improve obstacle understanding and track moving objects more accurately by adding a camera. This would enable us to distinguish between various types of obstacles, such as humans, roads, and vehicles, as well as other unsafe environments, thereby contributing to a safer BOB.

\- \*\*Add virtual test agents to simulate other robots and users:\*\* This can be done in the Unity environment for testing/demonstration purposes. This would enable the simulation of complex scenarios with multiple robots without requiring additional hardware.  
