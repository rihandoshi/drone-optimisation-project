# Minimum-Energy Trajectory Planning for a Planar Drone

## Overview
This project computes a smooth, dynamically-feasible, minimum-energy trajectory for a planar drone using an Augmented Lagrangian optimization method. The system avoids circular obstacles, respects discrete-time dynamics, and minimizes total control effort.

## File Structure
### `dynamic.py`
Contains:
- Drone dynamics matrices (A, B)
- Obstacle penalty computation

### `lagrangian.py`
Contains:
- Augmented Lagrangian computation
- Variable flattening/unflattening functions
- Numerical gradient (finite difference)

### `optimizer.py`
Contains:
- Inner-loop gradient descent
- Outer-loop Augmented Lagrangian updates
- Trajectory solver and visualization

## Running the Code
Install requirements:
```
pip install numpy matplotlib
```
Run:
```
python optimizer.py
```

## Outputs
- Optimal state trajectory `X`
- Optimal control inputs `U`
- Plot of drone trajectory with obstacle visualization

## Changing start and end goals
- The format is [position_x, position_y, velocity_x, velocity_y]
- Change the "start" variable according to the starting point that you want.
- Change the "goal" variable according to the ending point that you want.

## Adding Obstacles
Modify the list in `optimizer.py`:
```python
obstacles = [
    {"center": [2.5, 2.5], "radius": 1.0, "margin": 0.1}
]
```

## Tuning Parameters
- `kappa`: obstacle penalty strength  
- `rho_dyn`, `rho_start`, `rho_goal`: constraint enforcement  
- `inner_iters`: gradient descent iterations  
- `step_size`: learning rate  

## Description
The optimizer uses an Augmented Lagrangian to enforce:
- State transition constraints  
- Start/goal constraints  
- Obstacle avoidance (soft penalty)

Numerical gradients are computed via finite differences.
