# RRT

## goal_bias - double [0, 1]

Every sampled random point is interpolated with a percentage of the goal position following:
$$sample = random\\_point*(1 - goal\\_bias) + goal * goal\\_bias$$

## goal_bias_adapt - bool

Adapt *goal_bias* for every collision detected

### goal_bias_adapt_rate - double [0, 1]

Ammount to decrease to *goal_bias* whenever a collision is detected.

### goal_bias_adapt_reset - bool

If **true**, when a node is successfully placed, *goal_bias* is set to the initial value. If **false**, it is incremented by *goal_bias_adapt_rate*.

