# robocup-viz
A rviz remote visualization for Tobi and AMiRo based RSB2ROS bridging

CHECKOUT RECURSIVE
=
"git submodule update --init --recursive"

RUN
=
* Source `. ./devel/setup.sh`
* Run `spread &`
* Run the visualization
  * Common for testing : `roslaunch demo viz.launch`
  * Testing with fake transformations: `roslaunch demo viz_fake_tf.launch`
  * Finale stage demonstration: `roslaunch demo viz_final2016.launch`
  * Homecoming demonstration: `roslaunch demo viz_home2016.launch`


todo:
root@amiro21:~/senseLidar# ./senseHokuyo --outscope /lidar12
root@amiro28:~/senseLidar# ./senseSimple3DTo2D --justCenter --outscope /lidar18

bash> nano ~/.config/rsb.conf
host    = alpia.techfak.uni-bielefeld.de

cd /opt/repositories/twb-viz/catkin_ws
roslaunch demo viz_twb.launch