# ROSsell
CSE-180 -- Robotics Spring 2019

#### Useful Commands 

launch world with 

```bash
roslaunch husky_gazebo huskyfinal.launch
```

#### launch nodes with

```bash
rosrun finalproj exploreNode
rosrun finalproj obRec
```
### rviz usage 

```
rosrun rviz rviz
```

Click `Add` in the bottom left, add the element `PointCloud2` and click `Okay`

Click the dropdown on the new `PointCloud2` visualization and set the topic to `/my_cloud2`

### Consider adding aliases to your `~/.bashrc`

```bash
alias ds='source devel/setup.bash'
export devel=devel/setup.bash
alias ghusky='roslaunch husky_gazebo husky_playpen.launch'
alias fhusky='roslaunch husky_gazebo huskyfinal.launch'
```
