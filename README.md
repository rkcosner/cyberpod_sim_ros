# cyberpod_sim_ros

To install and run this package, you'll need the "ASIF++" and "OSQP_embedded" packages from AMBER Lab's bitbucket installed on your computer. Additionally, clone the rviz_camera_stream repo (https://github.com/lucasw/rviz_camera_stream.git) and the rviz_lighting repo (https://github.com/mogumbo/rviz_lighting.git) into the src file of your catkin workspace.

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

## Running repo
*go to top level of catkin workspace*
catkin_make *whenever code changes*
source devel/setup.bash *in every new terminal*
roslaunch cyberpod_sim_ros main.launch recording:="on" *the recording variable is "off" by default*
*in new terminal*
rosservice call /cyberpod_sim_ros/integrator/ui "cmd: 1 
data:-0" *cmd: 0 stops, 1 starts, 4 resets*

## Recordings
Recordings will appear in the /bags folder in the repo and in that folder is another readme for how to format the data.
