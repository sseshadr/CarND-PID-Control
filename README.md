# CarND-PID-Control
Use a PID controller to drive a simulated car on its track. Perform parameter tuning using the Twiddle (gradient descent) logic.

#### NOTE: This project involves the Term 2 Simulator which can be downloaded [here] (https://github.com/udacity/self-driving-car-sim/releases).

[//]: # (Image References)

[image1]: ./readme_media/optimizer1.png "Optimization Iterations 1"
[image2]: ./readme_media/optimizer2.png "Optimization Iterations 2"
[image3]: ./readme_media/ponly.gif "P only Simulation"
[image4]: ./readme_media/pdonly.gif "PD only Simulation"
[image5]: ./readme_media/result.gif "Final Simulation with optimized parameters"
[image6]: ./readme_media/pidbetter.gif "Final Simulation with manually tuned parameters"

---

#### Project Notes
Designing a PID controller included the following:

#### Choosing Controller Parameters:
Instead of starting with zero parameters, I guessed the inital controller parameter values and used the Twiddle (gradient descent) logic to fine tune the parameters. The initial parameters and the optimization routine are shown below. Because Twiddle has a tendency to get stuck in local minima, I decided to terminate the optimization prematurely and take the controller parameters of the best run yet.

![alt text][image1]
![alt text][image2]

#### Reflection:
##### Proportional controllers 
These help with reaching a particular set point as quickly as we can. However, having a gain too high could result in unstable (oscillating) behaviors. It is usually a good idea to start with just a P controller to see if the system somehow stabilizes even if the performance is not that great. If the controller is oscillatory right at the start, your P gain is probably too high. As you can see in the simulation above, we have got it just right. The initial guess for P was lower than this and intuitively we could see this resulting in kind of a sluggish in the sense that the car was weaving in long sweeping curves. You can all these characteristics in the P only simulator output below.

![alt text][image3]

##### Derivative controllers 
These help in 'anticipating' errors. Since it is a function of current and past errors, it provides a 'kick' whenever necessary. This would be a good addition when the P only controller is unable to bring the system back to stability during 'high' errors. In our case, these are the tight corners where we require high steering input without ramping up to the value. You could see that the without D, the car would just drive off the track. With D, it provides the extra controller effort to bring the car back on track (final simulation above). In general it is advisable to not use D controllers directly on noisy signals. D controllers amplify this noise and make the overall system unstable. It is very important to filter your input to the D controllers. In our case, our CTEs are not that noisy. In the real world, if the distance to curb is going to be returned by a noisy sensor, we need to implement appropriate filtering/tracking mechanisms before using D. You can all these characteristics in the PD only simulator output below.

![alt text][image4]

##### Integral controllers 
These help bring the system as close to zero error as possible. We do this by summing up the errors over the current and all the previous samples. Although this theoretically helps implement a more accurate system, this also might result in the system drifting away as time goes on since sum of really small errors over a long period of time, will result in a large controller output. Especially in a system like ours where controller effort on one part of the track is entirely independent on another part of the track, it is a good practice to keep I gains to a low value. You can all these characteristics in the PID only simulator output below. 

![alt text][image5]

Even though we have lower errors for the final optimized coefficients, we can see the result of higher I gain with a more oscillatory performance (above). In fact personally speaking, I liked the response of the system with the original I gain since it was a lot smoother ride (output below). 

![alt text][image6]

##### Error/Cost term 
We use mean squared error (MSE) of the CTE alone to determine the controller parameters. Although this is a fair metric, this does not take into account the lateral acceleration of the car. Swaying the car side to side does not result in a great ride. So terms that account for lateral acceleration and jerk should be taken into account while evaluating controller performance.

##### Suggested improvements
We currently actuate the car at constant throttle. We could make the throttle a function of the angle of the car such that, at higher curvatures, we go slower and at lower curvatures we go faster. This would not only result in lower errors but also a better ride. We could use a PID controller to do this as well.

#### Links that helped:

https://discussions.udacity.com/t/how-to-gracefully-quit-the-onmessage-loop-event/643264/3
