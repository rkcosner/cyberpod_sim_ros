# cyberpod_sim_ros
<<<<<<< HEAD
The majority of this code was original written by Andrew Singleterry. Adjustments were made by Ryan Cosner and Sarah Dean for use in producing the experimental results for their CoRL 2020 submission titled "Guaranteeing Safety of Learned Perception Modulesvia Measurement-Robust Control Barrier Functions" [arxiv link](https://arxiv.org/pdf/2010.16001.pdf).

To install and run this package, you'll need to clone the [**rviz_camera_stream**](https://github.com/lucasw/rviz_camera_stream.git) repo and the [**rviz_lighting**](https://github.com/mogumbo/rviz_lighting.git) repo into the src file of your catkin workspace. You'll also need the Python [**ECOS**](https://github.com/embotech/ecos-python) package which can be installed via: 
> 'pip install ecos'
=======
The majority of this code was original written by Andrew Singleterry. Adjustments were made by Ryan Cosner and Sarah Dean for use in producing the experimental results for their CoRL 2020 submission entitled "Guaranteeing Safety of Learned Perception Modulesvia Measurement-Robust Control Barrier Functions".

To install and run this package, you'll need the "ASIF++" and "OSQP_embedded" packages from AMBER Lab's bitbucket installed on your computer. Additionally, clone the rviz_camera_stream repo (https://github.com/lucasw/rviz_camera_stream.git) and the rviz_lighting repo (https://github.com/mogumbo/rviz_lighting.git) into the src file of your catkin workspace.
>>>>>>> 62653d16e6a1c503baf06f2756b058c7414c8868

<!--
## Install OSQP_embedded 
*clone from bitbucket*
*enter directory*
mkdir build 
cd build
cmake ..
sudo make install
 
## Install ASIF++ 
*clone from bitbucket* 
*go to directory*
mkdir build
cd build
cmake ..
ccmake . 
*change osqp_embedded to true*
sudo make install
-->

## Running repo
<<<<<<< HEAD
<!--
* go to top level of catkin workspace
* 'catkin_make' whenever code changes -->
* *cd catkin_workspace* go to the top level of the catkin workspace
* *source devel/setup.bash*
* *roslaunch cyberpod_sim_ros ecos.launch*
* open a new terminal 
* *source devel/setup.bash* in the new terminal 
* start the simulation using the command: *rosservice call /cyberpod_sim_ros/integrator/ui "cmd: 1 
data:-0"*
  * The commands values are: 0 stops, 1 starts, 4 resets

## Recordings
Recordings will appear in the */bags* folder in the repo and in that folder is another readme for how to format the data.

## Relevant Portions of the Code 
* cyberpod_sim_ros/
    * include/ 
        * dynamics.hpp
*   launch/ 
    * ecos.launch 
        * sample_state_space.launch
    * msg/ 
        * cmd.msg 
        * input.mcg
        * learning_data.msg
        * state.msg
    * src/
        * integrator_node.cpp
        * grid_state_space_node.cpp
        * sensor_node.cpp
        * controller_node.cpp
        * src_python/
            * data_logger.py
            * ecos_fitler_node.py
            * perception_node.py 
    * srv/
        * ui.srv
    * URDF/
        * cyberpod.urdf







## Key Features of Main Simulation: 
***ecos.launch*** 
    launches the main simulation 

***dynamics.hpp*** 
    the dynamics for the cyberpod as derived from the Euler-Lagrange equations

***msg/***
    the four messages listed above are the custom messages used to relay messages between the nodes of simulation and also between the user and the system 

***srv/ui.srv***
    the service for the user to interact with the simulation

***URDF***
    the system's visual model rviz visualisations 

**Nodes:**
* integrator_node.cpp
    * serves as the main physics engine for the system, receives inputs from ecos_filter_node and outputs integrated states 
* perception_node.py 
    * recieves image data and outputs state estimates
* sensor_node.cpp
    * receives state values from integrator node state estimates from perception_node and outputs "measured states" with true state values used whenever they cannot be attained from the image, such as velocity. 
* controller_node.cpp
    * receives state estimates and output desired input based on PD feedback gains
* ecos_filter_node.py 
    * receives state estimates and desired input and outputs minimally adjusted input to guarantee safety 


## Main Code for Data Collection 
* sample_state_space.launch
    * launch data collection procedure
* grid_state_space_node.cpp
    * iterates through a gridding of the state space 

=======
*go to top level of catkin workspace*
catkin_make *whenever code changes*
source devel/setup.bash *in every new terminal*
roslaunch cyberpod_sim_ros ecos.launch

*in new terminal*
rosservice call /cyberpod_sim_ros/integrator/ui "cmd: 1 
data:-0" *cmd: 0 stops, 1 starts, 4 resets*

## Recordings
Recordings will appear in the /bags folder in the repo and in that folder is another readme for how to format the data.

## Relevant Portions of the Code 
cyberpod_sim_ros/
    include/ 
        dynamics.hpp
    launch/ 
        ecos.launch 
        sample_state_space.launch
    msg/ 
        cmd.msg 
        input.mcg
        learning_data.msg
        state.msg
    src/
        integrator_node.cpp
        grid_state_space_node.cpp
        sensor_node.cpp
        controller_node.cpp
        src_python/
            data_logger.py
            ecos_fitler_node.py
            perception_node.py 
    srv/
        ui.srv
    URDF/







## Active Nodes in the Main Simulation: 
ecos.launch 
    launches the main simulation 

dynamics.hpp 
    the dynamics for the cyberpod as derived from the Euler-Lagrange equations

msg/
    the four messages listed above are the custom messages used to relay messages between the nodes of simulation and also between the user and the system 

srv/ui.srv
    the service for the user to interact with the simulation

URDF
    the simulation visual data used for the rviz visualisations 

Nodes: 
    integrator_node.cpp
        serves as the main physics engine for the system, receives inputs from ecos_filter_node and outputs integrated states 
    perception_node.py 
        recieves image data and outputs state estimates
    sensor_node.cpp
        receives state values from integrator node state estimates from perception_node and outputs "measured states" with true state values used whenever they cannot be attained from the image, such as velocity. 
    controller_node.cpp
        receives state estimates and output desired input based on PD feedback gains
    ecos_filter_node.py 
        receives state estimates and desired input and outputs minimally adjusted input to guarantee safety 


## Main Code for Data Collection 
    sample_state_space.launch
        launch data collection procedure
    grid_state_space_node.cpp
        iterates through a gridding of the state space 
>>>>>>> 62653d16e6a1c503baf06f2756b058c7414c8868

