## Overview
The PID controller project is to realize the PID controller. 
The PID class implementation doesn't cost very much time, while it is not for tuning PID parameters Kp, Ki, Kd.

In order to tune and find a good set values of Kp, Ki, Kd, the twiddle is used.

## Twiddle

#### counter

Unlike the lecture, there is no run() function just as the lecture does. So what is regarded as the run() function and then to update the array P[] and dP[]?

Here, counter is used. It added as the time elpased. When the counter had reached one threshold, it is regarded as that one try for the current parameter set had been done.

#### MCARO \_TWIDDLE\_

The MACRO '\_TWIDDLE\_' is used for open or close the code of tunning the parameters.

Comment the following code will close the finding parameters function.

#define \_TWIDDLE\_

#### acc_err

A acculated error is also used to judge the current paramters is good or not.


## Considerations

Due to that the car of the simulator may stop when it has been out of the road and blocked, the cte may change very much, and so the memeber variable of PID acc_err may incread a similar error each counter.

Consider another case, the car is running on the road for a long distance, the member variable of PID will increase too. Compare to the blocked case, the paramters may be better mostly. 

So the car speed is considered. When the speed is two low and the car had run for some time, it is considered to be blocked. Then, a additional error was added to the member variable acc_err.

Another consideration is that the value of member variable counter. The PID paramters found when counter is too small may not work well for a long distance.

The stop condition for the while loop to tune the parameters may not to reach, the three paramters can be found. Then, the three  parameters can be as the base paramters to run the simulator and then tune it manually if necessary.


