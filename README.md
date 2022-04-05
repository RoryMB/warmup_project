# warmup_project

## Behaviors

### Drive Square

For this task I broke it into 3 parts: moving forward, turning, and timing. For the movement forward and turning, I just set the velocities to whatever didn't cause slipping or errors. For timing, I just watched the robot at several different delays until it turned about 90 degrees, and kept that delay.

In the code, I put the functionality into two functions, moving forward and turning. These functions included timing logic. However, at the end, the robot continues forward. So, I added a stop function as well, which sends 0 velocity commands continuously.

![gif](https://github.com/RoryMB/warmup_project/square.gif "Test")

## Challenges

Timing correctly was a bit of a challenge since the simulator does not run in real time. I solved this somewhat by letting the simulator run for a minute or two to "warm up" and reduce the changes in delays throughout the program runtime.

## Future work

In future work I would use measurements instead of time, since they are more reliable. The simulator does not run in real time, so I had to try several times for it to map out a decent square.

## Takeaways

- My main takeaway is that timing is not reliable. I had many issues getting the simulator to act consistently since I am running it in a VM.
- My other takeaway is to set up network settings in advance. Trying to switch between simulator and robot cause me a lot of confusion since I would miss steps.
