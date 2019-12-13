[![Build Status](https://travis-ci.com/jgkawell/cairo-feedback-cclfd.svg?branch=master)](https://travis-ci.com/jgkawell/cairo-feedback-cclfd)

# CAIRO Feedback LfD
A system that recognizes fear in collaborative human agents and then demonstrates alternative actions for the human to correct via simple, verbal feedback. The human feedback is translated into constraints that modify a learned LfD skill.

Contribution: A system that recognizes undesirable effects caused by the way it executed a task and can update its policy based on human feedback.

Application: A collaborative human-robot task where the robot performs task via CC-LfD and receives feedback in the form of verbal boolean responses.

## Installation

The "easy" way to run this is to do everything in simulation within the provided Docker images. You can find the instructions for this at [this repository](https://github.com/jgkawell/docker-scripts). You'll want to follow the instructions for setting up the `jgkawell/ompl:lfd` tag (`lfd-nvidia` if you're running Nvidia graphics).

If you'd rather run things locally, [the same repository](https://github.com/jgkawell/docker-scripts) also has scripts within [a subdirectory](https://github.com/jgkawell/docker-scripts/tree/master/tools/linux) that will allow you to install everything on a clean Ubuntu 16.04 system (either running locally or within WSL). Simply run the scripts in this order: `ros` -> `ompl` -> `sawyer` -> `lfd`.

## How to use

You'll need to have three terminals open to run this system. In each, you'll need to run the following commands which I have numbered below:

1. `sim` and then `roslaunch lfd_experiments feedback.launch`
2. `sim` and then `roslaunch feedback_visualizations demo.launch`
3. `sim` and then `roslaunch feedback_master demo.launch`

The first part of each of those commands simply sets you up in the simulated Sawyer robot environment. `roslaunch lfd_experiments feedback.launch` sets up all the needed robot subsystems for running Sawyer within Gazebo and initializes it for the CC-LfD skill execution system. `roslaunch feedback_visualizations demo.launch` sets up an Rviz window to display path planning and scene objects. `roslaunch feedback_master demo.launch` actually runs the simulation as a demo of a cup hand off skill repair scenario.


NOTE: If you're not running this in simulation, the system currently works only on a Sawyer arm, using the webcam in the primary computer and must be connected to an OptiTrack motion capture system. This requires a lot more setup and instructions than can be covered here.
