In Rviz, display all three odometries

ros2 bag play your.bag

Wait end of first lap (protip: speed up/down the playback with up and down arrows)
Pause the playback with spacebar

python3 align_odom.py --ros-args --params-file config.yaml

Unpause the playback with spacebar

Wait until the script outputs a matrix. After that, you should see the aligned odometry appear on rviz.
Now don't close the script unless you want to recompute the matrix (e.g. change bag/parameters). You can replay your bag as many times as you wish.

---

min_samples: increase this if the matrix is computed too early - likewise, decrease it if it is computed too late.
use_position, use_orientation: leave these as is. use_orientation sucks.
registrator: control which algorithm is used to compute the matrix. See the functions in point_set_registration.py.