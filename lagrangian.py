import numpy as np
from dynamic import obstacle_penalty

def flatten_vars(X, U):
    return np.concatenate([X.reshape(-1), U.reshape(-1)])

def unflatten_vars(z, N, nx, nu):
    size_x = (N+1) * nx
    X = z[:size_x].reshape(N+1, nx)
    U = z[size_x:].reshape(N, nu)
    return X, U

def lagrangian_value(X, U, lambdas, params):
    A = params["A"]
    B = params["B"]
    start = params["x_start"]
    goal = params["x_goal"]
    rho_dyn = params["rho_dyn"]
    rho_start = params["rho_start"]
    rho_goal = params["rho_goal"]
    kappa = params["kappa"]
    obstacles = params.get("obstacles", None)
    N = U.shape[0]
    nx = X.shape[1]
    L = 0.0

    for k in range(N):
        L += U[k].dot(U[k])

    # Penalty for obstacles
    for k in range(N+1):
        L += kappa * obstacle_penalty(X[k], obstacles)

    # Constraints for laws of motion
    lam_dyn = lambdas["dyn"]
    for k in range(N):
        res = X[k+1] - (A @ X[k] + B @ U[k])
        L += lam_dyn[k].dot(res) + rho_dyn * res.dot(res)

    # Starting constraint (x0 should be the start point)
    res_s = X[0] - start
    L += lambdas["start"].dot(res_s) + rho_start * res_s.dot(res_s)

    # Goal constraint (xN should be the end point)
    res_g = X[-1] - goal
    L += lambdas["goal"].dot(res_g) + rho_goal * res_g.dot(res_g)

    return L


# We compute the gradient here
def numerical_grad(f, z, eps=1e-5):
    grad = np.zeros_like(z)
    f0 = f(z)
    for i in range(z.size):
        zz = z.copy()
        zz[i] += eps
        grad[i] = (f(zz) - f0) / eps
    return grad