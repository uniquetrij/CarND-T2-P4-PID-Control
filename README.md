[//]: # (Image References)

[image1]: ./images/throttle-pid.png
[image2]: ./images/steer-pid.png
[image3]: ./images/steer-speed.png

# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

##Reflection

A PID (proportional–integral–derivative) controller is a control mechanism that may be used to continuously modulate control based on some kind of feedback from the system. For the purpose of the self driving car, the cross-track-error (cte) may be used as the basis of the control to modulate its steering angle so as to reduce the error as much as possible as the vehicle moves on the track.

The `P` term contributes to creating a steering angle proportional to the current `cte`. If the `cte` is large and positive, so will be the steering angle too, taking into account a gain factor `Kp`. Although the `P` factor reduces large errors very effectively, soon it caused the output to oscillate around the mean `cte`. 

The `I` factor corrects the accumulated error due to the error values of the past and integrates them over time. Essentially, it addresses any residual error in the system. The gain factor used here is `Ki`.

The `D` term takes the derivative of the present error i.e. based on its current rate of change it anticipates the future trend of the error, and thereby correcting it. This effectively dampens the oscillating effect of the `P` term. Here too, a gain factor is used, `Kd`.

While trying to tune the hyper-parameters automatically using twiddle, I realized that this approach would best work when the vehicle is ran on an ideal track (i.e. perfectly straight path) during the tuning operation. Since such a path was unavailable in the simulator, I fell back to manual tuning. The following values were found to produce good enough result on the simulator.

```c++
                    Kp = 0.2
                    Ki = 0.004
                    Kd = 3
```

Although the PID is able to keep the vehicle on track, since the track itself has a lot of turns the car tend to wobble too much and seemed very unsmooth as if a drunk driver is driving it, especially when there is a turn and the vehicle has a very high speed. To correct this behavior, it is necessary to regulate the throttle. Greater turn is generally accompanied by lesser throttle and speed. Also, very small turns need not be taken immediately on a turning road. We can wait until the error is big enough to create any practical implication. Also, if the speed is high, using the PID output directly for steering can lead to overturning. The following code snippet implements this regulation.

```c++
                    pid.UpdateError(cte);

                    double steer = pid.steer;
                    double throttle = 0.45;

                    double x = fabs(steer);
                    if (x < 0.0001)
                        x = 0.0001;

                    double y = fabs(speed / 50.0f);
                    if (y < 0.0001)
                        y = 0.0001;

                    throttle *= 1 / (0 + pow(M_E, 3 * x));
                    steer *= -1 / (0 + pow(M_E, 2 * x)) + 1;
                    steer *= 1 / (0 + pow(M_E, y));
```
The following functions represent how much the throttle and steering are to be regulated based on the based on the PID output.

|Throttle Gain w.r.t Predicted Steering Angle by PID |
|:--------------------------------------------------:|
|![alt text][image1]                                 |


|Steering Gain w.r.t Predicted Steering Angle by PID|
|:-------------------------------------------------:|
|![alt text][image2]                                |

|Steering Gain w.r.t Current Vehicle Speed          |
|:-------------------------------------------------:|
|![alt text][image3]                                |


## Conclusion
A video of the simulation results can be found [here](https://github.com/uniquetrij/CarND-T2-P4-PID-Control/blob/master/video.mp4).

The simulation result were quite impressive with the above implementation. The max speed of the car reached up to 33 mph, while automatically regulating it during the turns so as to prevent it from overturning, wobbling or otherwise moving out of the track. It was a great experience working with the project, especially while tuning the hyper-parameters and finding the functions to further regulate the steering and throttle based on the error and current speed.