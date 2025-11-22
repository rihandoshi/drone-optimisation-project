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

There are two convenient ways to run the project:

- Interactive (recommended): run the user-facing script which prompts for start/goal and obstacles:

```powershell
python main.py
```

When running `main.py` you will be prompted to enter the start and goal states as four space-separated numbers each (px py vx vy). Example responses when prompted:

```
Enter START state (px py vx vy):
0 0 0 0
Enter GOAL  state (px py vx vy):
5 5 0 0
```

You will then be asked for the number of obstacles. For each obstacle you will be prompted to enter the center coordinates and the radius. Example:

```
Number of obstacles: 2

Obstacle 1:
    Center (cx cy): 2.5 2.5
    Radius: 1.0

Obstacle 2:
    Center (cx cy): 3 4
    Radius: 0.25
```

`main.py` builds the obstacles list using the entered `center` and `radius`. A small fixed `margin` of `0.1` is applied by the program.

- Non-interactive / quick test: run the solver with the hard-coded example in `optimizer.py` (this opens a plot):

```powershell
python optimizer.py
```

## Outputs
- Optimal state trajectory `X`
- Optimal control inputs `U`
- Plot of drone trajectory with obstacle visualization

## Changing start and end goals
- Interactive: use `main.py` prompts to enter `[px py vx vy]` values at runtime.
- Programmatic: if you prefer to hard-code values, edit the `start` and `goal` variables inside `optimizer.py` (or call `solve_trajectory(...)` from your own script) using the format `[position_x, position_y, velocity_x, velocity_y]`.

## Adding Obstacles
- Interactive: provide obstacles when prompted by `main.py` (enter center coordinates and radius for each obstacle).

- Programmatic: modify the list in `optimizer.py`:

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

## Notes
- The interactive runner `main.py` prints the final state and (if `matplotlib` is available) shows a plot of the resulting trajectory. If `matplotlib` is not installed the script will skip plotting and still print results.
- The obstacle margin used by `main.py` is fixed to `0.1` for interactive entries; you can change this behavior by modifying `read_obstacles()` in `main.py`.

## Team Members
Team Name: Drone
- Rihan Doshi (BT2024122)
- Saanvi Choudhary (BT2024223)
- Veda Chandavolu (BT2024257)
