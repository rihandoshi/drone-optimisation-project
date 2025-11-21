import numpy as np
import matplotlib.pyplot as plt

from dynamic import get_system_matrices
from lagrangian import (
    flatten_vars, unflatten_vars,
    lagrangian_value, numerical_grad
)

# Inner gradient-descent loop
def minimize_inner(z, lambdas, params, N, nx, nu, steps=40, step_size=5e-2):

    def f_z(vec):
        X, U = unflatten_vars(vec, N, nx, nu)
        return lagrangian_value(X, U, lambdas, params)

    z_curr = z.copy()

    for _ in range(steps):
        g = numerical_grad(f_z, z_curr)
        gnorm = np.linalg.norm(g)
        if gnorm < 1e-7:
            break
        z_curr = z_curr - step_size * (g / gnorm)

    return z_curr


# Outer augmented Lagrangian loop
def solve_trajectory(x_start, x_goal, N=30, dt=0.1, obstacles=None, kappa=2000.0, rho_vals=(10,10,10), outer_iters=10, inner_iters=200):

    A, B = get_system_matrices(dt)
    nx = 4
    nu = 2

    rho_dyn, rho_start, rho_goal = rho_vals

    # Initialise straight-line guess
    X = np.zeros((N+1, nx))
    for k in range(N+1):
        a = k / N
        X[k, :2] = (1-a)*x_start[:2] + a*x_goal[:2]

    U = np.zeros((N, nu))

    # Initialise multipliers
    lam_dyn = np.zeros((N, nx))
    lam_start = np.zeros(nx)
    lam_goal = np.zeros(nx)

    lambdas = {"dyn": lam_dyn, "start": lam_start, "goal": lam_goal}

    params = {
        "A": A, "B": B,
        "x_start": x_start,
        "x_goal": x_goal,
        "rho_dyn": rho_dyn,
        "rho_start": rho_start,
        "rho_goal": rho_goal,
        "obstacles": obstacles,
        "kappa": kappa
    }

    z = flatten_vars(X, U)

    #Outer Loop
    for it in range(outer_iters):

        # Minimise AL for fixed lambdas
        z = minimize_inner(z, lambdas, params, N, nx, nu, steps=inner_iters)

        X, U = unflatten_vars(z, N, nx, nu)

        dyn_res = np.zeros_like(lam_dyn)
        for k in range(N):
            dyn_res[k] = X[k+1] - (A @ X[k] + B @ U[k])

        rs = X[0] - x_start
        rg = X[-1] - x_goal

        # Update multipliers
        lam_dyn += 2 * params["rho_dyn"] * dyn_res
        lam_start += 2 * params["rho_start"] * rs
        lam_goal += 2 * params["rho_goal"] * rg

        # Tighten penalties slightly
        params["rho_dyn"] *= 1.1
        params["rho_start"] *= 1.1
        params["rho_goal"] *= 1.1

        print(f"Outer {it} | dyn err = {np.max(np.linalg.norm(dyn_res,axis=1)):.4e}")

    return X, U

if __name__ == "__main__":
    start = np.array([0,0,0,0], float)
    goal  = np.array([5,5,0,0], float)

    obs = [ {"center":[2.5,2.5],"radius":1.0,"margin":0.1}]
    X, U = solve_trajectory(start, goal, N=30, dt=0.1, obstacles=obs)

    plt.plot(X[:,0], X[:,1], marker='o')
    for o in obs:
        c = o["center"]
        r = o["radius"]
        circ = plt.Circle(c, r, fill=False)
        plt.gca().add_patch(circ)

    plt.axis('equal')
    plt.grid(True)
    plt.show()
