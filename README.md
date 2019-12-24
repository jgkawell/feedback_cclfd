[![Build Status](https://travis-ci.com/jgkawell/cairo-feedback-cclfd.svg?branch=master)](https://travis-ci.com/jgkawell/cairo-feedback-cclfd)

# Feedback CC-LfD

In the field of human-robot interaction (HRI), there is much interest in training methods that teach robot collaborators how to effectively execute tasks alongside human teammates. However, because of the complexity of deployment environments and the personal preferences of human teammates, these pre-trained policies often lead to behavior that cause a human collaborator to be forced off of a desired path to task completion or even to experience discomfort. Here, we introduce a system that can perform in-task, active learning to adapt its policy to a human teammate. This is accomplished by recognizing when a human teammate deviates from a desired task path or shows signs of discomfort due to robotic behavior and then augmenting a pre-trained policy based off of human feedback. Further, we demonstrate an implementation of this technique through a task where a simple handoff task fails and is repaired through a simulated environment.

Our most recent work focuses on the integration of NLP methods to both convert text to speech (for the robot to ask the user for feedback) as well as converting speech to text (for the user to give that feedback). For the NLP functions we utilized Google Cloud's APIs which provide a fairly simple interface to deploy STT/TTS solutions. The whole system is deployed using Docker and Travis runs continuous integration (CI) performing automated Docker builds as well as code style checking (using `pycodestyle`).

Additionally, we ported everything into a simulated environment that utilizes the Gazebo simulation of a Sawyer robot instead of a physical system. This allows for quick development that can happen away from the physical robot systems in our lab. This actually occupied a lot of the development time for this last effort as the process of porting everything into a working Docker environment was quite a task. All of this is fully documented in [this repository](https://github.com/jgkawell/docker-scripts).

This project has a demo built in that allows for a simple walkthrough of the system. The demo runs everything in simulation and demonstrates the Sawyer arm attempting a simple handoff task with a mug. The robot initially fails the handoff by flipping the cup upside down and the user can trigger a skill repair scenario using a teleop key (on a physical system this is done using computer vision and motion capture). This then prompts the robot to ask for feedback from the user and then show a variety of different ways it can perform the skill until the user confirms that it has been repaired. The robot then shows the relearned skill to the user.

As a side bonus feature, some extra visualization of the robot's movement and NLP systems have been integrated into Rviz. These are displayed as the demo is running to show what the NLP system is processing and how the robot is path planning throughout the scene.

Some current limitations of this project are that within Docker the audio from the NLP system does not get broadcast to the user but instead is simply written to an audio file local to the Docker container. Obviously this will need to be added for proper user trials in the future. Additionally, the learning system of the robot is still in the implementation phase and so the demo actually is more of a Wizard of Oz approach than a real implementation. Both of these features are in development with the intention of completion during the next few months.

Steps to install this project and subsequently run the demo are provided below.

## Installation

The "easy" way to run this is to do everything in simulation within the provided Docker images. You can find the instructions for this at [this repository](https://github.com/jgkawell/docker-scripts). You'll want to follow the instructions for setting up the `jgkawell/ompl:lfd` tag (`lfd-nvidia` if you're running Nvidia graphics).

If you'd rather run things locally, [the same repository](https://github.com/jgkawell/docker-scripts) also has scripts within [a subdirectory](https://github.com/jgkawell/docker-scripts/tree/master/tools/linux) that will allow you to install everything on a clean Ubuntu 16.04 system (either running locally or within WSL). Simply run the scripts in this order: `ros` -> `ompl` -> `sawyer` -> `lfd`.

## How to use

You'll need to have three terminals open to run this system. In each, you'll need to run the following commands which I have numbered below:

1. `sim` and then `roslaunch lfd_experiments feedback.launch`
2. `sim` and then `roslaunch feedback_cclfd demo_visuals.launch`
3. `sim` and then `roslaunch feedback_cclfd demo_main.launch`

The first part of each of those commands simply sets you up in the simulated Sawyer robot environment. `roslaunch lfd_experiments feedback.launch` sets up all the needed robot subsystems for running Sawyer within Gazebo and initializes it for the CC-LfD skill execution system. `roslaunch feedback_visualizations demo.launch` sets up an Rviz window to display path planning and scene objects. `roslaunch feedback_master demo.launch` actually runs the simulation as a demo of a cup hand off skill repair scenario.


NOTE: If you're not running this in simulation, the system currently works only on a Sawyer arm, using the webcam in the primary computer and must be connected to an OptiTrack motion capture system. This requires a lot more setup and instructions than can be covered here.
