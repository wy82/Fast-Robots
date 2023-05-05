---
permalink: /lab-10/
title: "Lab 10: Grid Localization Using Bayes Filter"
sidebar:
  nav: "lab-10"
---

This lab was primarily dedicated to implementing a Bayes Filter on a provided simulator.

## Bayes Filter

For reference, we note that the discrete version of the Bayes Filter can be described by the following pseudocode:

$$\begin{align}
\textbf{Algorithm} & \textbf{ Bayes_Filter}(bel(x_{t-1}),u_t,z_t) \\
    \text{for }& \text{all } x_t \text{ do} \\
    &\overline{bel}(x_t) = \sum_{x_t} p(x_t|u_t,x_{t-1})\;bel(x_{t-1}) \\
    &bel(x_t) = \eta\;p(z_t|x_t)\;\overline{bel}(x_t) \\
    \text{end}&\text{for} \\
    \text{return } &bel(x_t) 
\end{align}$$

## Compute Control

To begin, we first write a helper function to compute the net distance and rotation traveled between two points in time. 

Whereas the distance traveled could simply be calculated by taking a Euclidean norm, the rotation angles required a bit more care to normalize the angle to the range $[-180,\;180]$ after performing basic trigonometry.

```python
def compute_control(cur_pose, prev_pose):

    total_rot =  (180/np.pi)*np.arctan2((cur_pose[1] - prev_pose[1]), (cur_pose[0]-prev_pose[0]))
    delta_rot_1 = mapper.normalize_angle(total_rot - prev_pose[2])
    delta_trans = np.sqrt((cur_pose[0]-prev_pose[0])**2 + (cur_pose[1] - prev_pose[1])**2)
    delta_rot_2 = mapper.normalize_angle(cur_pose[2] - total_rot)

    return delta_rot_1, delta_trans, delta_rot_2
  
```

## Odometry Motion Model

Next, we then implement odom_motion_model() which essentially calculates the likelihood of transitioning to a pose given the control inputs u after a single timestep. 

This was implemented by taking conditional probabilities on a Normal distribution using the gaussian() function, where the mean and covariance were set to the control input u and process disturbances odom_rot_sigma, odom_trans_sigma, and odom_rot_sigma.

```python
def odom_motion_model(cur_pose, prev_pose, u):

    delta_rot_1, delta_trans, delta_rot_2 = compute_control(cur_pose,prev_pose)
    rot_1_prob = loc.gaussian(mapper.normalize_angle(delta_rot_1-u[0]),0,loc.odom_rot_sigma)
    trans_prob = loc.gaussian(delta_trans-u[1],0,loc.odom_trans_sigma)
    rot_2_prob = loc.gaussian(mapper.normalize_angle(delta_rot_2-u[2]),0,loc.odom_rot_sigma)
    prob = rot_1_prob * trans_prob * rot_2_prob

    return prob
```

## Prediction Step

We then implement the first half of the Bayes Filter by first calculating the odometry u using compute_control(). We then construct 6 nested loops, 3 for each of the poses at the current time step andanother 3 for each of the poses at the previous time step. This is obviously incredibly computationally intensive, and might benefit from vectorizing the computations later on.

Using odom_motion_model(), we then estimate the likelihood of the robot transitioning from between any two discrete poses captured by the x-coordinate, y-coordinate, and angular orientation. We then get the total conditional probability of transitioning from any pose in the previous timestep to a particular pose at the current timestep.

```python
def prediction_step(cur_odom, prev_odom):

    u = compute_control(cur_odom, prev_odom)
    for prev_x in range(mapper.MAX_CELLS_X):
        for prev_y in range(mapper.MAX_CELLS_Y):
            for prev_a in range(mapper.MAX_CELLS_A):
                for cur_x in range(mapper.MAX_CELLS_X):
                    for cur_y in range(mapper.MAX_CELLS_Y):
                        for cur_a in range(mapper.MAX_CELLS_A):
                            cur_pose = mapper.from_map(cur_x,cur_y,cur_a)
                            prev_pose = mapper.from_map(prev_x,prev_y,prev_a)
                            prob = odom_motion_model(cur_pos,prev_pose,u)
                            loc.bel_bar[cur_x,cur_y,cur_a] += prob * loc.bel[prev_x,prev_y,prev_a]
```

## Sensor Model

Before implementing the second half of the Bayes Filter, we write a function to calculate the likelihood of a ToF measurement given the current belief of the pose. This was fairly simple and only required calling gaussian() on 18 precached sensor measurements.

```python
def sensor_model(obs)

    prob_array = np.zeros((1,mapper.OBS_PER_CELL))
    for i in range(mapper.OBS_PER_CELL):
        prob_array[i] = loc.gaussian(obs[i],precached[i],loc.sensor_sigma)          

    return prob_array
```

## Update Step

Finally, we implement the update step of the filter. This mainly involved multiplying the measurement likelihoods obtained from sensor_model() with the beliefs obtained from the prediction step.

The key part of this calculation, however, is in the calculation of the normalizing constant to ensure that the belief remains a valid probability distribution and does not shrink to zero.

```python
def update_step():

    for x in range(mapper.MAX_CELLS_X):
        for y in range(mapper.MAX_CELLS_Y):
            for a in range(mapper.MAX_CELLS_A):
                prob_array = sensor_model(loc.obs_range_data)
                loc.bel[x,y,a] = np.prod(prob_array) * loc.bel_bar[x,y,a]
    eta = 1/(np.sum(loc.bel)) 
    loc.bel = loc.bel * eta 
```

## Simulation Results

To show that the filter actually works, we test the predictions on the simulator:

YOUTUBE

From here we observe that although the filter manages to somewhat accurately estimate the trajectory of the robot, it is still very noisy and prone to errors because of this. 

Furthermore, we also note that the filter has nonzero belief in regions that are impossible to reach, such as inside the boxes or outside the walls.

We also observe that the filter is incredibly slow during the prediction step, and has areas that could be improved upon by vectorizing operations over calling loops.

To address these issues, the discretization of the grid could be refined, and the probabilities in forbidden regions could be manually set to zero during the prediction step. Other solutions to improve accuracy might include adding a second distance sensor, or increasing the number of distance measurements per observation loop.
