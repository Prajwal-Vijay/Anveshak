# Anveshak Navigation Documentation
## newer_go_to_goal_flag.py

### Class GoToGoal - 

**All the functions that belong to this class by default have assumed contain an input variable called self**

### __init__():
	Defines the various variables in the class

### check_callback() - 
	This is a callback belonging to the check subscriber, basically it is called, to avoid an object



```
 def laser_callback(self,msg):
        self.tensor_prime = torch.tensor(msg.ranges).to(device)
        self.num_scans = len(msg.ranges)
        self.angle_max = msg.angle_max*180/math.pi
        self.angle_min = msg.angle_min*180/math.pi 
        self.right_45 = int(3*self.num_scans/8 )
        self.right_90 = int(self.num_scans/4)
        self.left_45 = int(5*self.num_scans/8)
        self.left_90 = int(3*self.num_scans/4)
        self.middle_0 = int(self.num_scans/2)
```
### laser_callback() - Basically, msg.ranges refers to an array with distances measured by the laser scanner, it is inf. when no object was detected, else it is a finite quantity. Angle_min and Angle_max refer to the start and end angles.
The scans are from -180 deg. to +180 deg. so, right_45 means 45 deg. to the right.




```
  def moving_average_filter(self):
        len_queue = 0
        print("Nigil")
        if(self.lidar_queue.full()):
            #print("Hello")
            for i in self.lidar_queue.queue:
                if len_queue == 0:
                    self.tensor = self.tensor_prime
                else:
                    self.tensor = self.tensor.to(device) + i.to(device)
                len_queue = len_queue+1
            self.tensor = self.tensor / len_queue
            _tensor = self.lidar_queue.get()
        else:
            self.tensor = self.tensor_prime
        self.lidar_queue.put(self.tensor_prime, True, 2)
```
### moving_average_filter() - Used to calculate a moving average of the data points it will help in reducing the noise while preserving the essential filters. This also adds stability of measurements, basically it takes a window around each point and calculates the average.

### depth_callback() - used to display the image as seen by intel depth camera, although its subscriber has been commented out.

### angle_callback() - takes angle values from IMU, returns the angle turned by the rover, from the initial angle(reference), we are considering the yaw angles only.

### odom_callback() - takes the odometer values and finds the current position wrt an origin (reference)

### get_angles(desired_angle, rotation_dir) - given an angle and the direction to rotate in, it will turn that way. The turning happens at a constant speed.





```
       def orient_rover(self):
        vel_msg = WheelRpm()
        a_right = self.tensor[self.right_45:self.middle_0]
        a_left = self.tensor[self.middle_0:self.left_45]
        a_right = a_right <6.0
        a_left = a_left <6.0
        #print(a_left)
        if(self.left_45 == 0 or self.right_45 == 0):
            print("Lidar is not giving points. Please rerun rplidar. ")
            return
        num_left = torch.count_nonzero(a_left)
        num_right = torch.count_nonzero(a_right)
        print(f"Number of points in the right less than 6m = {num_right}")
        print(f"Number of points in the left less than 6m = {num_left}")
        if (num_left-num_right >= self.lr_points):
            vel_msg.omega = self.turn_speed
        elif  (num_right-num_left >= self.lr_points):
            vel_msg.omega = -self.turn_speed
        elif not((num_left == 0) and (num_right == 0)):
            self.aligned_center = True
        print(f"WheelRpm Message given = {vel_msg} in orient_rover")
        self.vel_publisher.publish(vel_msg)
```
### orient_rover() - Used to align the rover along a direction so that the number of objects to the right less than 6m and to the left less than 6m is the same, this essentially will prevent the rover from hitting random objects.




```
   def check_if_object_in_front(self):
        a_straight = self.tensor[self.right_45:self.left_45]
        #print(self.left_45, self.right_45)
        if(self.left_45 == 0 or self.right_45 == 0):
            print("Lidar is not giving points. Please rerun rplidar. ")
            return
        a = a_straight < 1.5
        print(f"Checking if Object is in Front = {torch.count_nonzero(a)}")
        self.initial_less_than_2m_points = torch.count_nonzero(a)
        if(torch.count_nonzero(a) > 40):
            self.object_in_front =  True
        else:
            self.object_in_front = False
```
### check_if_object_in_front() - used to check if there are objects at a distance of less than 1.5 m in front of the object. It concludes that an object is in front only when the number of points in front is more than 40.



```
  def move_straight(self, vel, count =0) :

        vel_msg = WheelRpm()
        a_straight = self.tensor[self.right_45:self.left_45]
        #print(self.left_45, self.right_45)
        if(self.left_45 == 0 or self.right_45 == 0):
            print("Lidar is not giving points. Please rerun rplidar. ")
            return vel_msg
        a = a_straight < 1.5
        print(f"Number of points less than 1.5m = {torch.count_nonzero(a)}")
        self.initial_less_than_2m_points = torch.count_nonzero(a)
        if(torch.count_nonzero(a) > 10):
            self.startx = self.current_pose_x
            self.starty = self.current_pose_y
            vel_msg.vel = 0
            print(self.startx, self.starty)
            print("count = ", self.count)
            self.count = self.count + 1
        else:
            vel_msg.vel = self.move_speed

        a_prime = torch.nan_to_num(a_straight, posinf = 500, neginf = 500).to(device)
        avg_distance = torch.mean(a_prime[torch.nonzero(a_prime,as_tuple=True)].float())
        a_with_dist = torch.nonzero(a_prime).to(device)
        print(f"Average Distance = {avg_distance}")
        print(f"Points with distance: {a_with_dist.size(dim=0)} out of {self.num_scans} points")
        print("dimension = ",(a_with_dist.size(dim=0)/self.num_scans)*2*math.pi*avg_distance) #0.61 is multiplied as a constant factor for dimension estimation (?))

        #object_dim = avg_dist*(a_with_dist.shape)/(a_prime.shape)
        #print(object_dim)
        return vel_msg
```
### move_straight(vel, count=0) - takes in the velocity of the wheel and starts moving accordingly, if there is an object in the front, then no movement, else, it moves on.

### move_to_goal() - there is a variable called self.count, which essentially describes, number of iterations of forward movement. Depending on the value of this variable, we have multiple choices =>
```
          if self.count < 20:
            vel_msg = self.move_straight(1)
```
self.count < 20 and there is an object in front then it sets various values to its default.
```
            elif self.count >=20 and self.object_in_front == True: 
            self.count = 0
            self.turn_started = False
            self.turn_over = False
            self.second_stop = False
            self.third_stop = False
            self.first_stop = False
            self.object_in_front = False
            vel_msg.vel = 0
            vel_msg.omega  = 0
```
When self.count >= 20 and an object is detected in front of the rover (self.object_in_front == True), several flags are reset, including self.count, which is set back to 0. This indicates that the rover needs to stop and handle the obstacle before continuing its journey.

```
             elif self.first_stop == False and self.third_stop == False:
            if(self.turn_over == False): #turning over
                print(self.left_90, self.middle_0)
                a_straight = self.tensor[self.right_45:self.left_90]
                a = a_straight<2.0
                a_prime = torch.nan_to_num(a, posinf = 500, neginf = 500).to(device)
                a_with_dist = torch.nonzero(a_prime).to(device)

                print(f"Turning now. Number of points: {a_with_dist.size(dim=0)}")
                if(a_with_dist.size(dim=0) >10):
                    if(self.turn_timing_count_start == False):
                        self.last_time_turn = time.time()
                        self.turn_timing_count_start = True
                    else:
                        self.time_turn = self.time_turn + time.time()-self.last_time_turn
                        self.last_time_turn = time.time()
                    print(f"Time of turning = {self.time_turn}")
                    vel_msg.omega = self.turn_speed

                elif self.turn_over == False:
                    vel_msg.omega = 0
                    self.turn_over = True
```
If an Obstacle is detected then it analyzes the lidar data (self.tensor[self.right_45:self.left_90]).
If there are more than 10 points indicating obstacles within 2 meters, the rover continues turning (vel_msg.omega = self.turn_speed).The rover keeps track of the turn timing.
If turning is no longer needed, it stops turning (vel_msg.omega = 0) and sets self.turn_over =True.


```
              else:
                a_straight = self.tensor[self.right_90:self.middle_0]
                #print(a_straight)
                a = a_straight<2.5
                a_prime = torch.nan_to_num(a, posinf = 500, neginf = 500).to(device)
                a_with_dist = torch.nonzero(a_prime).to(device)
                self.check_if_object_in_front()
                
                print(f"Second Turn: Number of points less than 2.5m: {a_with_dist.size(dim=0)}")
                print("Second Turn Happening")
                
                if(a_with_dist.size(dim=0)>15):
                    vel_msg.vel = self.move_speed
                    self.displacement_y = self.displacement_y + math.sin(self.yaw_angle)#self.time_turn
                    print(f"Y Displacement = {self.displacement_y}")
                    vel_msg.omega  = 0
                else:
                    self.first_stop = True
                    if(self.displacement_y < 10):
                        self.y_disp_too_less = True
                    self.turn_over  = False
                    vel_msg.vel = 0
                    vel_msg.omega = 0
                    self.turn_timing_count_start = False
```
Now further the rover checks lidar data for obstacles within 2.5 meters (self.tensor[self.right_90:self.middle_0]).If obstacles are detected, it moves forward while avoiding them (vel_msg.vel = self.move_speed).If no obstacles are detected, it sets self.first_stop = True.

         
```
        elif self.first_stop == True and self.second_stop ==False and self.third_stop == False:
            self.check_if_object_in_front()
            a_prime = torch.nan_to_num(self.tensor[self.right_90:self.middle_0], posinf = 500, neginf = 500).to(device)
            #print(a_prime)
            dist_to_chk = 1.0
            if(self.y_disp_too_less == True):
                dist_to_chk = 2.0
            a = a_prime < dist_to_chk
            a_with_dist = torch.nonzero(a).to(device)
            print(a_with_dist.size())
            vel_msg = WheelRpm()
            
            if(abs(self.displacement_y) < 20) and self.y_disp_too_less == False:
                self.third_stop = True
            if(self.y_disp_too_less == True) and self.displacement_y > 10:
                self.y_disp_too_less = False
            
            if(a_with_dist.size(dim=0) > 10): #and self.current_pose_y > 0.05):
                vel_msg.vel = self.move_speed
                self.displacement_y = self.displacement_y + math.sin(self.yaw_angle)#self.time_turn
                vel_msg.omega  = self.turn_speed
                if(self.turn_timing_count_start == False):
                    self.last_time_turn = time.time()
                    self.turn_timing_count_start = True
                else:
                    self.time_turn = self.time_turn + time.time()-self.last_time_turn
                    self.last_time_turn = time.time()

                print("Obstacle detected!")
                print(f"Time of turning = {self.time_turn}")
                print(f"Y Displacement = {self.displacement_y}")

                #elif self.current_pose_y > 0.05:
            else:
                vel_msg.vel = self.move_speed
                self.displacement_y = self.displacement_y + math.sin(self.yaw_angle)#self.time_turn
                vel_msg.omega = -self.turn_speed
                if(self.turn_timing_count_start == False):
                    self.last_time_turn = time.time()
                    self.turn_timing_count_start = True
                else:
                    self.time_turn = self.time_turn - time.time()+self.last_time_turn
                    self.last_time_turn = time.time()

                print("No obstacle. Following wall")
                print(f"Time of turning = {self.time_turn}")
                print(f"Y Displacement = {self.displacement_y}")
```
The rover checks the lidar data to decide the next move.It adjusts the distance to check based on self.y_disp_too_less.If obstacles are detected, it continues moving and adjusting (vel_msg.vel = self.move_speed, vel_msg.omega = self.turn_speed).
If no obstacles are detected, it follows the wall while updating displacement and turn timing.


```
            elif self.third_stop == True:
            if(self.turn_over == False):
                if(self.yaw_angle <0):
                    vel_msg = self.get_angles(0,1)
                else:
                    vel_msg = self.get_angles(0,-1)
                print("Aligning to zero degrees. Turn Over? ", self.turn_over)
            else:
                vel_msg.vel = 0
                vel_msg.omega = 0
```
If  self.third_stop == True and further if self.turn_over == False then the rover aligns itself to zero degrees using self.get_angles(0,1) or self.get_angles(0,-1).If the rover is aligned then the rover stops completely (vel_msg.vel = 0, vel_msg.omega = 0).


.



 
