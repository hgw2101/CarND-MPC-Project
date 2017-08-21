###Model overview
I built a model predictive control (MPC) system by following closely to quiz in the lectures. Basically, a model that minimizes the difference between the reference trajectory and the constantly predicted trajectory of the vehicle path. Once the model is implemented correctly, I only had to tune the cost metrics before I had a vehicle that was able to complete the entire track.

Here is a step by step overview of how I built the MPC:

* 1) Using the waypoints provided by the vehicle (presumably its mapping system), calculate the vehicle's **reference trajectory**, i.e. where we want the vehicle to go, by using a 3rd degree polynomial
* 2) Calculate the vehicle's minimum cost **predicted trajectory** using the `IPOPT` solver and a set of vehicle model equations
* 2a) First, we need to define the update function for each **state variable**, i.e. `x`, `y`, `psi`, `v`, cross track error, or `cte`, and orientation error, or `epsi`. 
* 2b) Then, the solver will use the update function as constraints and try out different `steer_value` and `throttle_value`, i.e. the accuator inputs, to find the predicted trajectory with the minimum cost compared to the reference trajectory.
* 2c) The predicted trajectory will be `N` timesteps into the future with each timestep `dt` seconds long.
* 3) We then grab the first `steer_value` and `throttle_value` and use that to update our MPC, the MPC will go through steps 1-2 again to calculate a new minimum cost predicted trajectory, we then repeat the process by grabbing the first accuator inputs in this new predicted trajectory to update the model.

####Coordinate transformation
Since the waypoints, `px`, `py`, `psi` values are all given in vehicle coordinates, while the `steer` and `throttle` values are in vehicle coordinates, I had to transform the inputs, this would also help with visualization. I decided to transform everything from map coordinates to vehicle coordinates. This was done using something similar to the vehicle to map transformation formula I used in project 3, i.e. 

```
transformed_x = x * cos(theta) - y * sin(theta) + x_translation
transformed_y = x * sin(theta) + y * cos(theta) + y_translation
```

Here we set the new `px` and `py` to 0 since the vehicle is always at (0,0) from the vehicle's perspective, so we only need to update the waypoints. The `transformed_x` and `transformed_y` values in the formula above are the transformed waypoints coordinates. The theta is negative `psi`, since we are going from map to vehicle coordinates this time, while `x_translation` and `y_translation` are both 0 since they represent the vehicle's current position, which is (0,0).

Once the new transformed waypoints are calculated, we update `psi` to 0 as well.

####Model tuning
#####Cost
I found that the key to metrics tuning was to give the highest weight to `cte` and `epsi` to ensure the vehicle stays on the track, then the accuators for smooth controls, while worrying about reference velocity last. I did not, however, include any other cost metrics besides those mentioned in the lecture.

Based on feedback from my reviewer, I decided to experiment with higher cost metrics for smoother accuator inputs. While it did improve the vehicle's stability, it also decreased the vehicle's speed a little bit.

#####N & dt values
Initially I set `N` to 50 and `dt` to 0.05. My goal was to find an accurate predicted trajectory. However, this slowed down the model quite a bit as this translated to 390 variables that the solver needed calculate (6 * N + 2 * (N-1) total variables minus the 8 initial values). Since in essense, the MPC only grabs the first accuator input from the predicted trajectory, it probably makes sense to make `N` smaller, so I decided to experiment with lower `N` values and higher `dt` values, and with enough experiments, I found that setting `N` to 10 and `dt` to 0.1 yielded reasonable results.

####Latency
My first working model did not account for latency but the vehicle still drove fine even with the 100ms delay. To account for latency, I first decided to increase the latency to 150ms to see what the effect might be. Sure enough, the vehicle couldn't even get to the first turn before going off the track. I then set the latency to 0 to see what the vehicle would do in a "perfect scenario", and that was my objective when building additional metrics to account for latency.

Using the hint given in lecture, I decided to use the vehicle model to predict the state variables of the vehicle after 100ms, and then use these new state variables as the initial values going into the MPC. I only had to calculate `px`, `py`, `v` and `psi`, since the calculation of `cte` and `epsi` are the same as before. I also had to grab the current `steering_value` and `throttle` from the simulator to calculate the new `v` and `psi`, but that was simple.

Once I built in the logic to account for the 100ms latency, the vehicle ran almost as good as if there were no latency altogether!

I also tested what the model might do if the latency was 200ms, and it turned out that the vehicle drove in a very unsafe manner with constant steering left and right even though it managed to stay on the track. So it's clear that the vehicle model isn't very useful for predicting too long into the future!