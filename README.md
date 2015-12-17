# ece-470-590-final-tarro

Not entirely sure why IK starts off well and then suddenly jumps behind the
robot, but I have to go into work tomorrow and I don't have time to figure this
out. If I can get it working for the YouTube video I'll try to do that, but I
can't make any guarantees.

It's kind of working now. See the YouTube video. I had to disable half of the
joints in the arm just to make it nearly work, and the accuracy isn't very good
still. The same problem still applies - it will get very close and then jump
away. I made the function for getting the next point along the path try to
keep it from getting into corners where it might do that. It helped guide it
along the way, but it didn't entirely fix the problem.

In the video, the control loop was still running at 100 Hz because I didn't
want it to take to long while I was testing. But it should perform the same
(just slower) at 10 Hz.

YouTube: https://www.youtube.com/watch?v=m7IZ8Dgd86U
