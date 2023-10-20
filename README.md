# RRT

## goal_bias

Every sampled random point is interpolated with a percentage of the goal position following:
$$sample = random\\_point*(1 - goal\\_bias) + goal * goal\\_bias$$

## goal_bias_adapt

Adapt *goal_bias* for every collision detected

### goal_bias_adapt_rate

Ammount to decrease to *goal_bias* whenever a collision is detected.

### goal_bias_adapt_reset

If **true**, when a node is successfully placed, *goal_bias* is set to the initial value. If **false**, it is incremented by *goal_bias_adapt_rate*.

