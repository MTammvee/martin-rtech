*Task1_1

-Running task 1 -> roslaunch test task1_1.launch

To see the pubhlished velocities -> rostopic echo /cmd_vel

---Description of task1-1

Getting the values from bag file. Bag files had message typed PointStamped.
I saved the values to vector, so I could calculate the distance between
two points in order to calculate the velocity.
Because time had 2 components seconds and nanoseconds, I converted them to 
second and as I (at least think so) was getting overflow problem with it 
I substraced the first digits of the time. In order to calculate delta t
it did not make any difference. 
Calculating the velocity with formula : v = s / t. For each axis I 
saved the values as Twist type messages and published them under topic "cmd_vel"



*Task1_2

-Running task 1  -> roslaunch test task1_2.launch

To see what the node is publishing -> rostopic echo /cmd_vel_30


---Description of task1-2

In order to publish the points with the rate of 30Hz,  I first divided 
the calculated distance (between 2 points) with 30. Then I added the division
to the initial distance with the frequency of 30Hz. The value keeps increasing 
until the new disntace and division is being calculated.



*Task2

-Running task 2 -> roslaunch test task2_1.launch

To see what the node is publishing -> rostopic echo /filtered_data

---Description of task2-1

To filter the data from IMU sensor I used running average filter. The filter
stores 10 values into the vector and devides the value with the number of samples(in my code it is 10). Then I publish the filtered data under topic name "filtered_data".


*Task2

-Running task 2 -> roslaunch test task2_2.launch

To see what the node is publishing -> rostopic echo /cmd_vel

---Description of task2-2

Reading the accelration values from IMU sensor. Getting the delta t between to accelerations. Using the formula v = v0 + at to estimate the velocity. Made an
assumption that v0 = 0 in the first place. After calculating the speed publishing 
the values as twist messages. 



