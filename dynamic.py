import numpy as np

def get_system_matrices(dt):
    """
    This function is to construct discrete-time A and B matrices for a 2D point-mass drone.
    drone's dynamics:
        1)p_{k+1} = p_k + v_k*dt + 0.5*a_k*dt^2
        2)v_{k+1} = v_k + a_k*dt
    """
    if dt <= 0:
        raise ValueError("Time-step dt must be positive.")

    A = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1,  0],
        [0, 0, 0,  1]
    ], dtype=float)

    B = np.array([
        [0.5 * dt * dt, 0],
        [0, 0.5 * dt * dt],
        [dt, 0],
        [0, dt]
    ], dtype=float)

    return A, B

def obstacle_penalty(x_k, obstacles):
    """
    This Function computes squared penalty for violating any circular obstacle and 
    
    returns the total penalty accumulated from all obstacles.

    x_k: [px, py, vx, vy]
    obstacles: list of dicts:{ "center": [cx, cy], "radius": r, "margin": m }
    
    """
    if not obstacles:
        return 0.0

    px, py = x_k[0], x_k[1]
    p = np.array([px, py])

    total = 0.0
    for obs in obstacles:
        c = np.asarray(obs["center"], float)
        R = float(obs["radius"])
        m = float(obs.get("margin", 0.0))

        dist = np.linalg.norm(p - c)
        h = dist - (R + m)

        if h < 0:
            total += (-h) ** 2

    return total


