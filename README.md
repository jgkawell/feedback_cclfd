# CAIRO Feedback LfD
A system that recognizes fear in collaborative human agents and then demonstrates alternative actions for the human to correct via simple, verbal feedback. The human feedback is translated into constraints that modify a learned LfD skill.

Contribution: A system that recognizes undesirable effects caused by the way it executed a task and can update its policy based on human feedback.

Application: A collaborative human-robot task where the robot performs task via CC-LfD and receives feedback in the form of verbal boolean responses.

## How to use

1. Install ROS
2. Create a catkin workspace
3. Clone this repo into `<catkin_ws>/src`
4. Also clone the `master` branch of CAIRO LfD (https://github.com/cairo-robotics/cairo-lfd) as well as all of it's requirements
5. Run `catkin_make` within the root of your workspace
6. Run `roslaunch lfd_experiments cairo_lfd.launch` to start all of the necessary services for the CC-LfD system
7. Run `roslaunch feedback_master full.launch` to start the feedback process

NOTE: This system currently works only on a Sawyer arm, using the webcam in the primary computer and must be connected to an OptiTrack motion capture system. The steps above assume that you are connected to the same network as both the Sawyer and the OptiTrack and that all necessary setup on those systems have been made.
