# course_ws
the THU course Intelligent-Systems-2022Fall workspace as TA



### How to run the demo

@TODO: currently only forward kinematics

+ build

  in current project folder

```sh
catkin_make
```

+ start roscore in a terminal

```sh
roscore
```

+ give the python file executable permission

```sh
chmod +x ./src/me_arm/script/MecArm.py
```

+ source & run in another terminal

```sh
source ./devel/setup.*sh	# bash or zsh or others, bash in default
rosrun me_arm MecArm.py
```

+ **[forward kinematics]** then the pkg can get a msg from `/joint_states` topic and give out the `/end_point` topic

  for test

<img src="/home/czx/course_ws/README.assets/image-20220923220908686.png" alt="image-20220923220908686" style="zoom: 80%;" />

  + in a terminal

  ```sh
  rostopic pub /joint_states sensor_msgs/JointState "header: 
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  name: ['']
  position: [0.785, 4.712]
  velocity: [0]
  effort: [0]" 
  ```

> the position[0]: θ1, position[1]: θ2

+ in a terminal

```sh
rostopic echo /end_point 
```

​		and then you can see the published endpoint msg, like:

<img src="/home/czx/course_ws/README.assets/image-20220923221627957.png" alt="image-20220923221627957" style="zoom:80%;" />

> x for px, y for py, z is meaningless
