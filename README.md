# warmup_project

## Behaviors

### Drive Square

This task requires moving the robot in an approximate square. Since using physical measurements would be a bit intense for the starter project, I opted to use timing. To do this, I broke it into 3 parts: moving forward, turning, and timing. I first determined a decent speed that the robot could move forward and turn without slipping errors in gazebo or the real world. Then, I measured timings for how long the robot needed to move a decent distance, and make a 90º turn. From there, I simply called the movement and turning functions with the appropriate timings to form a square.

I wrote a main function, `run()`, which moves forwards and turns for each leg of the square, and stops at the end to simplify debugging. For the `go_forward()` and `turn()` functions, I made each one use the movement speeds I had previously determined were safe to move at. I then looped these twist messages until the timer had elapsed the previously measured times. At the end, the `stop()` function just sets the velocities to 0 to avoid runaway robots, and continuously sends this message to ensure it is received.

![](https://github.com/RoryMB/warmup_project/blob/main/square.gif?raw=true)

### Follow Person

This task requires following a person (or nearest object) without getting too close. To do this, I decided to handle turning and lateral motion steps separately to avoid making the code too complex. At a high level, the robot always turns towards the nearest object, then moves forward if it is already facing it.

I kept all of the code in the main `process_scan()` function since it was fairly simple. I start by finding the distance and angle to the closest point, which should be the person the robot is following. I then filter out any points that are too far away, which could be distant walls, and points that are much too close, which were noise. From there, the robot simply moves forward if it is already facing the point, or turns to the point if it is not.

![](https://github.com/RoryMB/warmup_project/blob/main/follow.gif?raw=true)

### Follow Wall

This task requires approaching a wall, then following the wall around any corners at a relatively constant distance. I ultimately decided to do a stateless system, where all frames are treated independently. At each timestep, the robot decides if it is much too far away from the wall and needs to approach, or if it can move along the wall. It always tries to keep the closest point to its right, which means that if it approaches a wall in front of it from a corner, that becomes the new wall and it automatically turns to handle it. Similar if the wall to its right dissapears, then the corner goes the other way, and it will try to rotate to find the wall again, automatically rounding the corner.

This program was a bit more complex, so I broke it down slightly differently from my other programs. I wrote a number of angle helper functions to simplify logic in the main function. The `angle_err()` function returns the error between the given angles, normalized to within ±180. This simplifies logic that needs the + or - sign to determine whether to turn left or right. This in turn made the `turn_towards()` function easier, since it can receive this normalized angle with sign information for direction, and the magnitude of the angle which is used for proportional control. The `angles_close()` function determines if two angles are within a certain offset of each other, and handles wrapping around 360º for the user. I used this function to determine if the angle to the wall was good enough to permit moving forward. `move_forward()` just sets the linear speed. Last, `find_nearest_point()` bundles all of the logic for interpreting the lidar scan and finding the distance and angle to the nearest object, which should be the point on the wall perpendicular to the bot, or the corner if going around the outside of a corner.

![](https://github.com/RoryMB/warmup_project/blob/main/wall.gif?raw=true)

## Challenges

Most of the initial challenges I faced stemmed from differences in the simulator and the real world. I was initially using a very laggy machine for the simulator, and found speeds and timings that worked for driving in a square, but these did not translate very well into the real world. In the later tasks, I just used the simulator to debug the overall structure of my code, and used a real world robot to fine tune the behavior. The most difficult challenge was on the wall following task, where the robot would receive messages on its state from too far back, and would run into walls before it realized it needed to move away, or would sail away from the wall when going around the outside of corners. Proportional control helped somewhat, but I had the best results when moving the robot and the machine with roscore into close proximity of each other and the wifi router.

## Future work

I would like to try the drive in a square task with physical measurements rather than timings, to ensure a more accurate square shape. For the person following task, I would allow turning and linear movement at the same time, although that would potentially require running more slowly due to the lag in communications. For the wall following task, I tried to speed the robot up, but ran out of time to get this working without running into walls. I would have liked to improve my proportional control here, as well as tried running the robot when noone else was in the room to see of that reduced communication lag or not. I might also be able to factor communication lag into the algorithm, and predict what state the robot will be in when it receives the message.

## Takeaways

- My main takeaway is that timing is not reliable. I had many issues getting the simulator to act consistently since I was running it in a slow VM. This was somewhat fixed when moving to a faster machine, but there were still discrepancies, so I did most of my fine-tuning on real world robots.
- Another takeaway is to set up network settings in advance. Trying to switch between simulator and robot cause me a lot of confusion since I would miss steps.
- My final takeaway is that the algorithm needs to be robust to robot errors and lag. I put this since the robot sometimes seems to sometimes miss a message or get it too late, and perform actions when it is too late.
