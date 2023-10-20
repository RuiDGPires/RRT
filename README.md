# RRT

## Goal_bias

Every sampled random point is interpolated with a percentage of the goal position following:
$$sample = random\_point*(1 - goal\_bias) + goal * goal\_bias$$

