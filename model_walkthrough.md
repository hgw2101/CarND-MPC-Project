###Model training notes
I built a model predictive control system by following closely to quiz in the lectures. Basically, a model that minimizes the difference between the reference trajectory and the constantly predicted trajectory of the vehicle path. Once the model is implemented correctly, I only had to tune the cost metrics before I had a vehicle that was able to complete the entire track. I found that the key to metrics tuning was to give the highest weight to `cte` and `epsi` to ensure the vehicle stays on the track, then the accuators for smooth controls, while worrying about reference velocity last. I did not, however, include any other cost metrics besides those mentioned in the lecture.

####Latency
My first working model did not account for latency but the vehicle still drove fine even with the 100ms delay. To account for latency, I first decided to increase the latency to 150ms to see what the effect might be. Sure enough, the vehicle couldn't even get to the first turn before going off the track. I then set the latency to 0 to see what the vehicle would do in a "perfect scenario", and that was my objective when building additional metrics to account for latency.

Using the hint given in lecture, I decided to use the vehicle model to predict the state variables of the vehicle after 100ms, and then use these new state variables as the initial values going into the MPC. I only had to calculate `px`, `py`, `v` and `psi`, since the calculation of `cte` and `epsi` are the same as before. I also had to grab the current `steering_value` and `throttle` from the simulator to calculate the new `v` and `psi`, but that was simple.

Once I built in the logic to account for the 100ms latency, the vehicle ran almost as good as if there were no latency altogether!
