# Q-learning for Arduino-based Self-crawling robot 

## Description : 

The robot has 2 arms controlled by 2 servo motors and is equipped with un ultrasonic sensor. 

**Action space :** The range of servo1 is from 80 to 100 degrees ,and the range of servo2 Is from 90 to 145 degrees. 
The output of the microcontroller is an action elicited to one of the sevo arms telling it to move up or down by a pre-defined
increment. The increment of servo1 (closer to the body) is 10 degrees and the increment of servo2 (touches the ground) is 15degrees.

**State space :** the state of the robot is a combination between the angle of servo1 and servo2 .

**Reward signal :** The microcontroller takes input from an ultrasonic sensor that measures the distance to the wall behind it. 
We give to our robot a reward R proportional to the distance the robot moved after each action choosed.

## Material: 
Arduino Due , 2 servo motors , toy-vehicle body
