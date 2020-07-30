# Deployment for motion_planners library.
In the folder scripts/test_kuka_iiwa example scritps are given on how to deploy motion_planners library.
For the example scripts to work, we need  [joint_control](https://git.hb.dfki.de/dfki-control/orogen-joint_control) component.
The [joint_control](https://git.hb.dfki.de/dfki-control/orogen-joint_control) component will create a robot based on a urdf. 
To install [joint_control](https://git.hb.dfki.de/dfki-control/orogen-joint_control) in rock, please type:

- amake control/orogen/joint_control

The joint_control library need "gsl". So incase of any error during installation of joint_control. Type 

- sudo apt-get install libgsl-dev

Now you go the scripts/test_kuka_iiwa folder and start the following two scripts:

- ruby start_kuka_viz.rb
- ruby planner_controlGUI.rb


