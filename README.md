# Automatic Apple Harvester System

## Thesis
Using a DUCO Cobot, we hope to create an automatic system for harvesting apples. This repository is primarily built by Devan Neil, with hardware support from Randy Allard and the Washington Fruit Tree Commission. 

### Assumptions and Conditions
  * To automatically harvest apples we are tackling this project in phases. This phase is a proof of concept, not a fully    realized system. This is the leading assumption, that the rest of the system will be in place for a fully realized system. This assumption lends itself to a pipeline as such:
    * Localize an apple
    * Plan and execute a picking motion
    * Place the apple in a chute
  We do not solve the problem of how to store and move the apple afterwards. 
  * The next major assumption we are making is targeting growing wall orchards. Orchards will often grow their apple trees in fruiting walls. We are only going to solve for this type of orchard. This allows us to tread the apple localization problem as a continuous scan of a wall instead of searching and segmenting an individual tree. This assumption has lead itself to these resources
    * Apples in localized clusters
    * Apples will always be in a rough plane
    * Apples will always be hanging from a stem vertically
  * The final large assumption we've made is red apples. Our vision model can absolutely handle multispectral apples, however it is most reliable with traditional red apples. Our fake apples are approximately red delicios. 
  
  * In this system we tested against false apples that are weighted to match real apples weights, and hung them on a rack with hooks. This is to emulate the visual and spacial distribution of apples and to emulate the picking motion required to harvest an apple, which we found to be roughly twisting upwards and then pulling. We understand that not every apple can be harvested as such.

  * This system also implements a QR Scanning system to localize the bin itself. Because we are not working with a rigid platform to mount everything on, we don't have the specifics of the trailer yet, we are using this QR localization system to find the chute accurately every time.

### Previous Work
Randy Allard has previous experience developing a picking system and has served as the technical advisor and hardware engineer on this project. Randy designed our bin filler as well as the toolhead on the arm. 

Devan Neil has experience with computer vision, developing and implementing object detection pipelines and with localization in 3D space. Devan also has experience developing in ROS2 and Linux.

#### Literature
Previous Papers will go here, once we've collected our research

## Methods
### Vision
Our vision system relies on this model: https://github.com/eugeneyjy/apple-3d-localization
We use this model to find apples by their corners. To calculate the exact position of the apples we tried several methods.
* Projecting to plane:
This was the first method we implemented, because we know roughly how far away the plane should be we attempted to project the center of the vision consensus into the plane. This system was innefective, however, as the plane variance between apples is significant.
* Centroid calculation:
This method used the infrared data from the camera, taken as the latest available, and calculated the centroid of the apple from it. We collected all infrared points between the corners of the consensus, and took the centroid of this. We had originally attempted to find the closest point in the consensus ROI, the region bounded by the corners of the vision consensu, but this method was not effective as the front surface of the apples don't register on the infrared camera. The centroid calculation method was effective and reliable between 70 and 80% of the time, depending on conditions such as lighting. We implemented a controlled lighting scheme, using the spotlight and a diffuser, to attempt to control this error but it proved to be more effective to investigate further methods.
* RANSAC Best fit Sphere:
This method relies on the ROI extraction from the previous method. We did refine the ROI extraction here, as the previous method used PCL in C++, and we were aiming to reduce ROS calls, thus keeping the entire method in a single ROS process. The RANSAC method then creates sample spheres of aproximately the correct radius for an apple and returns the sphere which closest fits the ROI data. Each method required a constant tool offset in all 3 coordinate systems, as they were all rough methods, however this method only requied an offset in the z axis, which can be explained by the 9mm separation from the visual orign and the infrared orign. Our camera drivers, which were made by the camera manufacturer, were transposing data from the IR frame to the camera frame for us, however the physical frame the data was coming from was likely the IR frame, not the camera frame, explaining the need for correction. This method proved to be the most reliable method for localizing apples in 3D space.

### Movement
Early on, we decided to use ROS2 and moveit to handle controlling the physical arm. DUCO, the manufacturer's of our arm, have provided a ROS2 driver for us to use and a moveit package. We have not included this package in our repository, as it is not our code to share, and no public facing link exists. 
Part of using moveit was figuring out the interface. We began by programming in Python, using constraints to create a goal and sending that to moveit. This was never successful. We could make the arm move, but it was at best unpredictable. Predictability was a significant problem to solve, as this arm will be working around crews. 
We migrated our code into C++ to use Moveit's move_group interface, which was far more intuitive and reliable. After solving how to move the arm, we needed to solve how to move the arm consistently. In our final code, we have several fail protection methods, as there is strange behavior between moveit and the DUCO driver. This is likely associated with our development environemnt being on WSL, instead of native Linux, however we were not able to definitively investigate these issues. We tried several methods to reduce the inconsistency of the movement, including changing our OMPL planner, and breaking trajectories into smaller steps, however the biggest win was moving to RTTStar for planning, and applying workstation bounds with each move command. While this does lead to more fail to plan errors, it mean the toolhead of our arm won't plan on leaving a bounding box closed by the start and end position of a movement. This does not solve every movement issue, as when the arm fails it's failure state is still undefined. We solved this by forcing the arm to stop if it fails, and giving the driver a respawn delay if it shuts down. 
Note, there are still cases where the arm can do undefined behavior, which can be dangerous. Never leave the system unattended.

As part of this process, we investigated using an asyncronous plannging pipeline, where large amounts of this system are deterministic and thus can be preplanned. The only non deterministic part of this system in the actual apple position, where it could be moved by the system moving, but if we enforce that the system does not move while the arms are in motion we could theoretically preplan every movement. Moveit actually does support this, including mutliple arms, however we were not able to implement it with our architecture.

### Control
The most natural control method for this system is a state machine. Ours currently implements several self contained states and some transition states to control behavior flow. Our behavior flow is roughly as follows:
* Monitor: Initialize here and wait for user confirmation
* Hold:
  * If there are apple poses, move to approach
  * If there are no apple poses, move to heatscan
* Approach:
  * Fetch the latest apple pose, move to x -= 0.1 from the pose
  * Move to pick
* Pick:
  * Enable suction
  * Move to apple pose
  * If suction timeout move to Hold
  * If suction succeed move to retreat
* Retreat:
  * Perform twist pick motion
  * Move to Chute
* Chute:
  * Move to z += 0.1 from chute
  * Disable suction
  * Move to hold
* HeatScan:
  * If scan pose available move to scan pose else move to Monitor
  * Move to CloseScan
* CloseScan:
  * Trigger vision scan
  * If vision scan timeout move to HeatScan
  * If vision scan success add poses to apple poses
  * Move to hold

### Scout Camera
This is the novelty of our system. In other apple harvesting systems, they continuall scan the wall with cameras pointed at where the picker is working. In our system we rely on the scout camera to generate scan poses that will cover every apple on the wall. While we are scanning apples twice, we found the scout camera's not able to generate accurate apple poses at it's distance and the closer scan from the arm is still required. We generate scan poses by collecting all of the consensus we can see and running a least covering circle algorithm based on the arm camera's scan radius. This algorithm returns a set of circles which will cover the entire set of apples from that scan, which are then projected into the arm camera's xy plane and attached to the ROS2 map frame. We move map with the arrow keys to simulate odometry from the actual trailer. When a scout pose moves to the arm's workspace it become available for the arm to request. Once the arm requests it, no other arm can request it, and once the arm is done with it it gets removed from the list of available poses and deleted. This way we can cover the entire wall without needed to search empty space.

## Startup Procedure
### Initial Setup
These steps are not necessarily in order, so long as they are all completed
* Set up the ESP32 following the diagram below.
* Connect the J2 control box to the robot using the CAN bus cable.
* Connect the teach pendant to the J2 control box.
* Connect the J2 control box (LAN 1 port), cameras, and the laptop to the POE ethernet switch. The order does not matter
* Plug in the ethernet power switch to 120V power.
* See below for verifying the cameras are connected and working.
* Plug in the J2 control box to 220V power.
* Turn on the power switch on the J2 control box.
* Press and hold the power button on the teach pendant.
* Once the teach pendant is active, click the second symbol on the left hand side menu, beneath the robotic arm. Go to Net. Verify you are connected to LAN1. Select the `Static IP` option, and set the IP to `192.168.1.10` and the Subnet Mask to `255.255.0.0`. Select `Confirm`.
* To verify the connection between the robot and the laptop run `ping 192.168.1.10`
* Connect the USB cable between the ESP32 and the laptop. The port does not matter.
* Plug in the load resistor to 120V power. The spotlight on the arm should turn on.
* Connect the air compressor to 120V and turn it on.
* Make sure that the IP address on your laptop's ethernet port is set to `192.168.1.xxx`. In our system we used `99`.
### ESP32 wiring diagram

![image info](./images/ESP32%20Diagram.png)

### Robot and bin layout
**THIS MUST BE FOLLOWED**

![image info](./images/Arm_and_Bin_Layout.png)

### Verify the Cameras
This is not necessarily required, but it can be useful.
* Begin by opening the ScepterGUI Tool. This tool can be found here: https://github.com/ScepterSW/ScepterGUITool

![image info](./images/ScepterGUIHome.png)

* From this screen, select the `Scan` button.
* You should get a screen which lists all the connected Scepter cameras. If you do not see your cameras, they are not connected correctly. Verify that your laptop's network adapter has been set to `192.168.1.xxx` and that the cameras are plugged into the ethernet POE switch, and verify the cable run is connected the whole way. You may need to disable the firewall. 
* Once the camera is connected from the scan screen, select the `Stream` toggle switch to see a live feed from the camera.
* Go to the `Network` tab from the list along the top of the window.
* Verify the `Arm Camera` has an IP address of `192.168.1.101` and a Subnet mask of `255.255.255.0`.
* Verify the `Scout Camera` has an IP address of `192.168.1.201` and a Subnet mask of `255.255.255.0`.

### Starting the Demo
To start the demo, do these steps in this order (Windows).
* Firstly, set the mode on the teach pendant to Manual and the speed to 50%. If you want to move the arm faster, set the mode to Automatic and the speed to 100%. These settings can be found at the top right of the teach pendant screen.
* Open the Executables folder.
* Run `Initialize System.bat`.
* At this point you may run `Suction Test.bat`, which just verifies the suction system is working.
* This step may fail. Below are debugging steps.
  * `No CP210x USB device is currently connected.`: Make sure the USB cable is connected to the ESP32.
  * `Failed to attach USB device.`: Unplug the USB cable and plug it back in.
  * Holding at `[Launch] Waiting for robot at 192.168.1.10...`: Verify your network setup is correct and that the arm is on.
  * Watch the terminal for this log statement:
    ```   
    [suction_commander-7] Failed to open serial port: No such file or directory
    [suction_commander-7] [WARN] [1787764825.897966833] [suction_commander]: Unable to open serial port!
    ``` 
    It means that something has failed in initialization and the system needs restarted. When you restart, reseat the USB device before running the executable.
  * Watch the terminal for log statements like this from any camera:
    ```
    [component_container-3] [INFO] [1787699358.211737362] [arm1_cam]: scGetFrameReady failed! -23
    ```
    This is a stale camera error. We don't know what causes it, but to fix it you have to restart the system
* Wait for the arm to entirely activate. It will make noise, and a chattering sound.
* You may see log statements like these:
```
[scout_camera_scanner-6] [WARN] [1787765171.013004085] [scout_camera]: Move forward by one pane!
[robot_state_publisher-8] [WARN] [1787765166.711975152] [robot_state_publisher]: Moved backwards in time, re-publishing joint transforms!
```
You may safely ignore these.
* Run the `Do QR Scan.bat` executable. THIS WILL MOVE THE ARM. Pay attention to where the arm ends up after the second movement, it should be directly over the chute.
* IF the arm successfully found the chute, you may run `Scan In Place.bat` to scan the apples the arm can currently see, which you can find on the Camera View window. 
* This step may fail with `[ERROR] [1787765587.115700842] [qr_check_node]: QR check failed: no /qr_valid state received.`. IF you have ran `Do QR Scan.bat` after running `Initialize System.bat`, you may safely rerun this code and it should work. This issue is because the computer's CPU is struggling to keep up with the processes running.
* If you want to move the arm, you may either use the buttons on the teach pad while in Manual mode, or run `Start RViz.bat` to move the arm.
* To see the results of the scout camera, follow the above instruction to run RViz.

![image info](./images/RVizHome.png)

* From this screen, select `Add` in the top left display. This will open a new window. In this window, select `By Topic` and look for `/pose_visualizer`, where you'll see `PoseArray` beneath it. Double click on `Pose Array`. This will add the scan poses to the 3D view based on where the scout camera thinks the arm needs to move to to scan the entire wall. 

![image info](./images/PoseArray.png)

* From the `Initialize System.bat` terminal, you can use the up and down arrow keys to simulate the robot moving.
* Run the `Move To ScanPose.bat` executable to move the arm to one of these scan poses, if it can reach.
* Running `Pick In Place.bat` after this emulates the intended system behavior.

## Results

<p align="center">
  <video src="./images/PickAndPlace.mp4" controls width="800">
    Your browser does not support the video tag.
  </video>
</p>

![image info](./images/ArmCamera1.png)

This is the results from our computer vision system, where it identified the apples. This system is run through Oregon State's models, which can be found here: https://github.com/eugeneyjy/apple-3d-localization

![image info](./images/ScoutCamera1.png)

This is the results from the Scout Camera, which runs the same model, and takes a least covering circle algorithm to determine the scan poses based on the Arm Camera's scan radius.

![image info](./images/ScoutPoses1.png)

This is these poses projected into the Arm Camera's xy plane.

## Future Work

### List of work

* State Machine Redesign

The state machine requires states to be defined as hard concepts, and in this system they are just not. Any future development of this system should redesign the state machine to use a class based state instead of an enum, where each state owns it's entrance/exit behaviour, and each state owns it's transitions.

* Arm Controller Redesign

A major limitation of this system is the arm controller architecture. We are relying on the move_group interface to handle all arm controls, which is neither testable nor exensible enough for commercialization. Moveit supports a task system which is worth investigating, otherwise the arm controller should be broken into several smaller systems. These systems should include a controller class, movement controller class, and an interface class. The controller should dictate logical flow, the movement class should dictate async planning and execution (see async branch), and the interface class should hold the ROS node.

* Arm Driver Redesign
 
While the DUCO Arm has served us well, it's driver is far from ROS standard, it doesn't respect the timestamps created by moveit. I know the rough process to build a new TrajectoryController for the arm, this controller should interpolate trajectory points and send them one at a time at the appropriate timestamp. 

* Camera System Debugging

Currently the cameras themselves appear to produce stale timestamps. That's my leading theory on why moving the arm while scanning produces bad results. I'm not sure what else could be causing, but we can condense all of our camera system into a singular node, using the ScepterSDK instead of relying on their ROS implementation. I also am not fully convinced their camera calibration matix is accurate

* Scout Camera implementation

The Scout Camera system theoretically works. If we move the main camera system to a singular node, we'll do the same for the scout camera. The Scout Camera should produce the correct scan positions to cover the entire wall, however we don't actually have the odometry to prove it. 

* Vision Inference Pipeline

The vision inference system needs work. I think we can actually get rid of the wsu models, and instead build our own discrete model. I'd like to fuse circle detection with the IR cloud data to make more concrete predictions. Ideally users should select a ripe apple, and using the HSV data from that we can pick the rest of the row. This system desparately needs GPU acceleration. I'd also like to investigate making the scout camera the source of truth. The arm can have a camera if we need it, but we might not. This is dependent on odometry from an encoder and IMU, and may require a LIDAR scanner. 

* Bin Manager

This entire system can go. We won't need to localize the bin on the trailer once the trailer is rigid. Testing on a garage floor we can't bolt the bin down, so we won't, but testing on a real trailer we can use much more concrete numbers. We won't actually need the pin position itself, as the bin will probably move, but the chute won't.

* Serial Interface

This system is great, and I love it, but if we move to a LattePanda or Jetson for our main computer, we can get rid of it and use the GPIO bus on those. 

## Out of Scope work

* Complete automation

This entire system will never be completely automatic. It's not viable and it's not practical. A truly automatic system will go down each row automatically, pick every apple (~95%), and return the apples to a centralized collection point. Something like this is viable for a small garden but this is not practical for an entire orchard and should always have human oversight.

* Pick Every Apple

This isn't viable for any robotic system, there will always be edge cases. This is not practical either. Theoretically to pick every apple you just run through the orchard enough times or wait for the apples to fall. Our current work is shooting for 80% however 95% is a successful result. 

* No produce bruising/loss

This isn't viable for our system as we can only handle what happens before the bin filler. While we are making every effort to reduce this number, there are going to be some we can't protect against. 