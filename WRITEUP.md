## Model

I used the kinematic bicycle model for this project, which ignores dynamic effects such as 
friction and torque. The model state consists of `x` and `y` for the vehicle position, `psi` 
for the vehicle orientation, `v` for velocity, `cte` for cross-track error, and `epsi` for 
orientation error. 

The model consists of the following equations. (`Lf` is the distance from the center of gravity 
to the front wheels). 
```c++
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

The vehicle is controlled via actuations for steering angle and throttle/brake.

## N and dt

I ended up using 10 for `N` and 0.1 for `dt`. This game me a time duration of 1 second, which 
seemed reasonable for the reference speed of 40mph I had set. I felt it struck a good balance
between accuracy and computational cost. I originally tried `N=5` and `dt=0.05` and the result
were less good.

## Preprocessing

I first converted the waypoint positions from the global coordinates to those relative to
the vehicle. Then the polynomial was fitted to the transformed points using the formula:
```c++
dtx = waypointsx[i] - px;
dty = waypointsy[i] - py;
transformedx[i] = dtx * cos(psi) + dty * sin(psi);
transformedy[i] = dty * cos(psi) - dtx * sin(psi);
```
Vehicle velocity was also converted from mph to m/s since the waypoints are in meters.

## Latency

I compensated for latency prior to passing the state to the solver using the equations below.
```c++
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = epsi[t] + v[t] * delta[t] / Lf * dt
```
Where x[t], y[t], and psi[t] are all 0 due to the transformation to the vehicle's perspective.
