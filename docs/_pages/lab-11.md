---
permalink: /lab-11/
title: "Lab 11: Localization on the Real Robot"
sidebar:
  nav: "lab-11"
---

In this lab, the grid localization filter developed in Lab 10 was carried over from the simulator to the real robot.

## Simulation

To begin, we include a brief test of the filter under the simulation, Note that in order to improve the performance of the filter, we use the following vectorizations of the filter in the localization_extras.py file:

```python
def compute_control(self, cur_pose, prev_pose):
        total_rot =  (180/np.pi)*np.arctan2((cur_pose[:,1] - prev_pose[:,1]), 
                                              (cur_pose[:,0]-prev_pose[:,0]))
        delta_rot_1 = self.mapper.normalize_angle(total_rot - prev_pose[:,2])
        delta_trans = np.linalg.norm(cur_pose[:,0:2]-prev_pose[:,0:2],axis = 1)
        delta_rot_2 = self.mapper.normalize_angle(cur_pose[:,2] - total_rot)
        return delta_rot_1, delta_trans, delta_rot_2

    def odom_motion_model(self, cur_pose, prev_pose, u):
        delta_rot_1, delta_trans, delta_rot_2 = self.compute_control(cur_pose,prev_pose)
        rot_1_prob = self.gaussian(delta_rot_1,u[0],self.odom_rot_sigma)
        trans_prob = self.gaussian(delta_trans,u[1],self.odom_trans_sigma)
        rot_2_prob = self.gaussian(delta_rot_2,u[2],self.odom_rot_sigma)
        prob = rot_1_prob * trans_prob * rot_2_prob
        return prob

    def prediction_step(self, cur_odom, prev_odom):
        u = self.compute_control(np.array([cur_odom,]), np.array([prev_odom,]))
        bel_array = np.ravel(self.bel,order = 'C')
        idx = np.nonzero(bel_array > 0.000001)
        prob = self.odom_motion_model(self.mapper.get_cur_poses(idx),self.mapper.get_prev_poses(idx),u)
        prob_array = np.reshape(prob,(self.mapper.total,len(idx[0])),order = 'C')
        predictions = np.dot(prob_array,bel_array[idx])
        self.bel_bar = np.reshape(predictions,self.bel.shape,order = 'C')
        self.bel_bar[self.mapper.get_forbidden()] = 0
        eta = 1/(np.sum(self.bel_bar))
        self.bel_bar = self.bel_bar*eta
        
    def sensor_model(self,obs):
        obs_range_data = np.tile(np.squeeze(self.obs_range_data),(self.mapper.total,1))
        prob_array = self.gaussian(obs_range_data,obs,self.sensor_sigma)    
        return prob_array
        
    def update_step(self):
        self.bel = deepcopy(self.bel_bar)
        obs_array = np.reshape(self.mapper.get_full_views(),(self.mapper.total,self.mapper.OBS_PER_CELL),order = 'C')
        prob_array = self.sensor_model(obs_array)
        updates = np.prod(prob_array,axis = 1)
        self.bel = np.reshape(updates,self.bel_bar.shape,order = 'C')*self.bel
        eta = 1/(np.sum(self.bel))
        self.bel = self.bel * eta
```

The key improvements in the efficiency of the code mainly included broadcasting the likelihood computations in odom_motion_model and compute_control over arrays of multiple poses. This then allowed for parallel processing of every pose combination, which completely removed the need for nested for loops.

Although the sensor_model() and update_step() functions were relatively easy to vectorize and test, the prediction_step was particularly challenging and required several reshaping commands to make processing easier. 

Another feature of the rewritten functions is the inclusion of checks for when the previous timestep's belief is negligible, which then prompts the function to discard these entries in the final calculation of the likelihoods. This was implementing by slicing out the irrelevant entries:

```python
def get_cur_poses(self,idx):
    mask = np.zeros((self.total,), dtype=bool)
    mask[idx] = 1
    return self.cur_poses[np.repeat(mask,self.total)]
    
def get_prev_poses(self,idx):
    mask = np.zeros((self.total,), dtype=bool)
    mask[idx] = 1
    return self.prev_poses[np.repeat(mask,self.total)]
```

Finally, we also note that the prediction step also accounts for forbidden regions of the map, and sets the corresponding beliefs to zero for those locations:

```python
def get_forbidden(self):
    val_x = np.reshape(self.poses[:,0], self.cells.shape,order = 'C')
    val_y = np.reshape(self.poses[:,1], self.cells.shape,order = 'C')
    idx = np.zeros(self.cells.shape, dtype = bool)
    # Box 1
    idx[(val_x>=-0.152) & (val_x<=0.152) & (val_y >= -1.372) & (val_y <= -0.762)] = 1
    # Box 2
    idx[(val_x>=0.762) & (val_x<=1.372) & (val_y >= -0.153) & (val_y <= 0.458)] = 1
    # Walls
    idx[(val_x<=-0.762) & (val_y >= -0.152)] = 1
    return idx
```

Granted, these improvements only provide very marginal improvements to the filter's accuracy, since the disturbances to the robot's motion are quite large. Instead, the update step seems to provide the most accurate estimates of the pose, but it certainly wouldn't hurt to gather more information from the odometry of the robot.

## Observation Loop

