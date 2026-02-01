# Homework 4 - Tyler Scotti
### Date: 01/30/2026


### State Space

**Definition:**
```
X = {(x, y, theta) | x in [0, 3], y in [0, 3], theta in [0, 2pi)}
```

**Explanation:**
A state **x** = (x, y, theta) in X represents the robot's configuration at a given time, where:
- **x** in [0, 3]: the robot's position along the horizontal axis in meters.
- **y** in [0, 3]: the robot's position along the vertical axis in meters. 
- **theta** in [0, 2pi): the robot's direction angle in radians (assuming the robot can pivot in a 360 degree range).

The state describes where the robot is at in the 3m × 3m workspace as well as which direction it is facing.

---

### Action Space

**Definition:**
```
U = {(v, omega) | v in R, omega in R}
```

**Explanation:**
An action **u** = (v, omega) in U represents the control inputs to the robot, where:
- **v**: linear velocity in m/s 
- **omega**: angular velocity in rad/s 

The action (v, omega) is applied over a time interval delta_t to produce motion.

---

### Transition Function

**Definition:**

For a state **x** = (x, y, theta) and action **u** = (v, omega) applied over time interval delta_t, the next state **x'** = f(**x**, **u**, delta_t) is:
```
x' = x + v*cos(theta)*delta_t
y' = y + v*sin(theta)*delta_t
theta' = theta + omega*delta_t (mod 2pi)
```

**Explanation:**

This is the model for Pedro's Robot: 

1. The robot moves in the direction it's facing (theta), so the velocity components are:
   - Horizontal: v*cos(theta)
   - Vertical: v*sin(theta)

2. The orientation changes at rate omega

3. The (mod 2pi) operation on theta' keeps the angle within [0, 2pi)

Note: I made the assumption the robot can only move in the direction it is facing. As well as no noise affecting the robot.
