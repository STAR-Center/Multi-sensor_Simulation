#! /bin/bash
{
	gnome-terminal -t "roslaunch_main" -x bash -c "roslaunch platform_gazebo kinetic_main.launch; exec bash"
};
echo "Launching multi-sensor platform simulation..."
sleep 25s
{
	gnome-terminal -t "pub_camera_fr_traj" -x bash -c "rosrun get_groundtruth camera_fr_traj.py; exec bash"
};
{
	gnome-terminal -t "move_platform_py" -x bash -c " rosrun platform_gazebo move_platform_half_circle.py; exec bash"
};
    echo "Wait a moment to get ready..."
{
    sleep 19s
    MID=`xdotool search --name "move_platform_py"`
    for idm in $MID; do
      xdotool windowactivate --sync $idm key --clearmodifiers 3
      xdotool windowactivate --sync $idm key --clearmodifiers KP_Enter
    done
	gnome-terminal -t "rosbag_record" -x bash -c "source rosbag_record.sh; exec bash"

}
echo "Start moving the platform ..."
sleep 105s
echo "Stop everything ..."
WIDS=`xdotool search --name "rosbag_record"`
for idw in $WIDS; do
  xdotool windowactivate --sync $idw key --clearmodifiers ctrl+c
  sleep 1s 
  xdotool windowactivate --sync $idw key --clearmodifiers alt+F4
  sleep 1s
done
echo "Stop rosbag record."
PIDS=`xdotool search --name "pub_camera_fr_traj"`
for idi in $PIDS; do
  xdotool windowactivate --sync $idi key --clearmodifiers ctrl+c
  sleep 1s
  xdotool windowactivate --sync $idi key --clearmodifiers alt+F4
  sleep 1s
done
echo "Stop publishing groundtruth."
MIDS=`xdotool search --name "move_platform_py"`
for idim in $MIDS; do
  xdotool windowactivate --sync $idim key --clearmodifiers ctrl+c
  sleep 1s
  xdotool windowactivate --sync $idim key --clearmodifiers alt+F4
  sleep 1s
  xdotool windowactivate --sync $idim key --clearmodifiers alt+ctrl+w
  sleep 1s
done
echo "Stop move_platform_py"
RIDS=`xdotool search --name "roslaunch_main"`
for idii in $RIDS; do
  xdotool windowactivate --sync $idii key --clearmodifiers ctrl+c
  sleep 1s
  xdotool windowactivate --sync $idii key --clearmodifiers alt+F4
  sleep 1s
done
echo "Stop Gazebo."
sleep 3s
