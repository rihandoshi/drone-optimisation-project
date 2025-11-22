import numpy as np
from optimizer import solve_trajectory


def read_vector(prompt, length):
    print(prompt)
    values = input().strip().split()
    while len(values) != length:
        print(f"Please enter exactly {length} numbers.")
        values = input().strip().split()
    return np.array([float(v) for v in values], float)


def read_obstacles():
    try:
        count = int(input("Number of obstacles: ").strip())
    except:
        print("Invalid input, assuming 0 obstacles.")
        return []

    obs_list = []
    for i in range(count):
        print(f"\nObstacle {i+1}:")
        cx, cy = input("  Center (cx cy): ").split()
        r = float(input("  Radius: ").strip())
        obs_list.append({
            "center": [float(cx), float(cy)],
            "radius": r,
            "margin": 0.1  # small fixed margin
        })
    return obs_list


if __name__ == "__main__":

    print("\n=== DRONE TRAJECTORY INPUT ===\n")

    x_start = read_vector("Enter START state (px py vx vy):", 4)
    x_goal  = read_vector("Enter GOAL  state (px py vx vy):", 4)

    dt = 0.1

    obstacles = read_obstacles()

    print("\nRunning optimiser...\n")

    X_opt, U_opt = solve_trajectory(
        x_start,
        x_goal,
        N=30,                 
        dt=dt,
        obstacles=obstacles,
        kappa=2000.0,
        rho_vals=(10.0, 10.0, 10.0),
        outer_iters=10,
        inner_iters=200,
    )

    print("\nOptimisation completed.")
    print("Final state:", X_opt[-1])
    print("Goal state:", x_goal)
    
    # Plot the result
    try:
        import matplotlib.pyplot as plt

        px = X_opt[:, 0]
        py = X_opt[:, 1]

        plt.plot(px, py, marker='o', label="trajectory")
        plt.plot(x_start[0], x_start[1], 'gs', label="start")
        plt.plot(x_goal[0], x_goal[1], 'rs', label="goal")

        for obs in obstacles:
            c = obs["center"]
            r = obs["radius"]
            circle = plt.Circle(c, r, fill=False)
            plt.gca().add_patch(circle)

        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title("Drone Trajectory (User Input)")
        plt.show()

        # Plot velocity components over time
        # plt.figure()
        # plt.plot(X_opt[:, 2], label='vx')
        # plt.plot(X_opt[:, 3], label='vy')
        # plt.title("Velocity Profile")
        # plt.xlabel("Time step")
        # plt.ylabel("Velocity")
        # plt.legend()
        # plt.grid(True)
        # plt.show()


    except ImportError:
        print("matplotlib not installed — skipping plot.")
