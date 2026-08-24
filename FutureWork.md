### Future Work
In this document I will list off the work remaining to be done with this system before it's ready for commercialization.

## List of work

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

The vision inference system needs work. I think we can actually get rid of the wsu models, and instead build our own discrete model. I'd like to fuse circle detection with the IR cloud data to make more concrete predictions. Ideally users should select a ripe apple, and using the HSV data from that we can pick the rest of the row. This system desparately needs GPU acceleration.

* Bin Manager

This entire system can go. We won't need to localize the bin on the trailer once the trailer is rigid. Testing on a garage floor we can't bolt the bin down, so we won't, but testing on a real trailer we can use much more concrete numbers. We won't actually need the pin position itself, as the bin will probably move, but the chute won't.

* Serial Interface

This system is great, and I love it, but if we move to a LattePanda or Jetson for our main computer, we can get rid of it and use the GPIO bus on those. 

## Out of Scope work

* Complete automation

This entire system will never be completely automatic. It's not viable and it's not practical. A truly automatic system will go down each row automatically, pick every apple (~95%), and return the apples to a centralized collection point. Something like this is viable for a small garden but this is not practical for an entire orchard and should always have human oversight.

* Pick Every Apple

This isn't viable for any robotic system, there will always be edge cases. This is not practical either. Theoretically to pick every apple you just gotta run through the orchard enough times or wait for the apples to fall. Our current work is shooting for 80% however 95% is a successful result. 

* No produce bruising/loss

This isn't viable for our system as we can only handle what happens before the bin filler. While we are making every effort to reduce this number, there are going to be some we can't protect against. 