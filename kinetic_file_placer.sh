read -n 1 -s -r -p "Please move all files to your ~/Downloads folder. Press any key to continue..."


source /opt/ros/kinetic/setup.bash

roscd husky_gazebo/worlds/
sudo mv ~/Downloads/final2019.world .
roscd husky_gazebo/launch/
sudo mv ~/Downloads/huskyfinal.launch .
sudo mv ~/Downloads/playpenfinal.launch .
roscd husky_navigation/maps/
sudo mv ~/Downloads/mapfinal.yaml .
sudo mv ~/Downloads/mapfinal.pgm .
roscd husky_navigation/launch/
sudo mv ~/Downloads/amcl_final.launch .

read -n 1 -s -r -p "To launch the simulation run roslaunch husky_gazebo huskyfinal.launch"
