# Anveshak_Nav Documentation
## newer_go_to_goal_flag.py
!(Navigation_newer_go_to_goal_flag.png)
### Class GoToGoal - 

**All the functions that belong to this class by default have assumed contain an input variable called self**

### __init__():
Defines the various variables in the class

### check_callback() - 
This is a callback belonging to the check subscriber, basically it is called, to avoid an object

### laser_callback() - 
Basically, msg.ranges refers, to an array with distances measured by the laser scanner, it is inf. when no object was detected, else it is a finite quantity. Angle_min and Angle_max refer to the start and end angles.
The scans are from -180 deg. to +180 deg. so, right_45 means 45 deg. to the right.

### moving_average_filter() - 
Used to calculate a moving average of the datapoints it will help in reducing the noise while preserving the essential filters. This also adds stability of measurements, basically it takes a window around each point and calculates the average.

### depth_callback(msg) - 
Used to display the image as seen by intel depth camera, although its subscriber has been commented out.

### angle_callback(msg) - 
Takes angle values from IMU, returns the angle turned by the rover, from the initial angle(reference), we are considering the yaw angles only.

### odom_callback(msg) - 
Takes the odometer values and finds the current position wrt an origin (reference)

### get_angles(desired_angle, rotation_dir) -
Given an angle and the direction to rotate in, it will turn that way. The turning happpens at a constant speed.

### orient_rover() - 
Used to align the rover along a direction so that number of objects to the right less than 6m and to the left less than 6m is the same, this essentially will prevent the rover from hitting random objects.

### check_if_object_in_front() - 
Used to check if there are objects at a distance of less than 1.5 m in front of the object. It concludes that an object is in front only when number of points in front is more than 40.

### move_straight(vel, count=0) - 
Takes in the velocity of the wheel and starts moving accordingly, if there is an object in the front, then no movement, else, it moves on.

### move_to_goal() - 
There is a variable called self.count, which essentially describes, number of iterations of forward movement. Depending on the value of this variable, we have multiple choices =>
self.count < 20 and there is an object in front then set various values to its default.
