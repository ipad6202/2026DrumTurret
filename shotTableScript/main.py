"""
FRC 2026 shooter trajectory optimization.

This program uses the Sleipnir NLP solver to find the initial pitch and yaw for a game
piece to hit the 2026 FRC game's target given an initial velocity.

This optimization problem formulation uses direct transcription of the flight dynamics, including
air resistance.

Based on the 2022 trajectory optimization example code by Tyler Veness.
https://github.com/SleipnirGroup/Sleipnir/blob/main/examples/frc_2022_shooter/main.py

Required packages:
- numpy
- sleipnirgroup-jormungandr
"""

import json
import math

import numpy as np
from numpy.linalg import norm
from sleipnir.autodiff import VariableMatrix, atan2, block, cos, hypot, sin, sqrt
from sleipnir.optimization import ExitStatus, Problem

# Physical characteristics
min_pitch = np.deg2rad(90 - 45)  # rad
max_pitch = np.deg2rad(90 - 5)  # rad
g = np.array([[0], [0], [9.81]])  # m/s²
max_shooter_velocity = 14.5  # m/s
ball_mass = 0.5 / 2.205  # kg
ball_diameter = 5.91 * 0.0254  # m
ball_moment_of_inertia = 0.4 * ball_mass * (ball_diameter / 2) ** 2  # kg m²


# Solve settings
printResults = False


def lerp(a, b, t):
    return a + t * (b - a)


def cross(u, v):
    return VariableMatrix(
        [
            [u[1, 0] * v[2, 0] - u[2, 0] * v[1, 0]],
            [-u[0, 0] * v[2, 0] + u[2, 0] * v[0, 0]],
            [u[0, 0] * v[1, 0] - u[1, 0] * v[0, 0]],
        ]
    )


def f(x, omega):
    # x' = x'
    # y' = y'
    # z' = z'
    # x" = −F_D(v)/m v̂_x
    # y" = −F_D(v)/m v̂_y
    # z" = −g − F_D(v)/m v̂_z
    #
    # Per https://en.wikipedia.org/wiki/Drag_(physics)#The_drag_equation:
    #   F_D(v) = ½ρv²C_D A
    #   ρ is the fluid density in kg/m³
    #   v is the velocity magnitude in m/s
    #   C_D is the drag coefficient (dimensionless)
    #   A is the cross-sectional area of a circle in m²
    #   m is the mass in kg
    #   v̂ is the velocity direction unit vector
    rho = 1.221  # kg/m³
    v = x[3:6, :]  # m/s
    v2 = (v.T @ v)[0, 0]
    v_mag = sqrt(v2)
    r = ball_diameter / 2
    C_D = 0.47
    A = math.pi * r**2  # m²
    m = ball_mass
    F_D = 0.5 * rho * v2 * C_D * A

    # Magnus force formula
    # Kinda sus since the internet can't seem to agree on a formula but this seems to be a good one
    #   F_M(v) = ½ρvAC_L (v̂ × ω)
    #   C_L is the lift coefficient (dimensionless)
    C_L = 0.00025
    v_hat = v / v_mag
    F_M = 0.5 * rho * C_L * A * v_mag * cross(v, omega)

    return block([[v], [-g - F_D / m * v_hat + F_M / m]])


N = 40


def add(
    problem,
    distance,
    shooter_height,
    target_height,
    last_solve=None,
):
    """
    Solve for minimum velocity.
    :returns: A tuple of [True, velocity, pitch, yaw, X] if it succeeds at a solve, and a tuple of[False, 0] if it fails.
    """
    # Robot initial state
    shooter_wrt_field = np.array(
        [
            [0],
            [0],
            [shooter_height],
            [0],
            [0],
            [0],
        ]
    )

    target_wrt_field = np.array(
        [
            [distance],
            [0],
            [target_height],
            [0.0],
            [0.0],
            [0.0],
        ]
    )

    # Set up duration decision variables
    T = problem.decision_variable()
    problem.subject_to(T >= 0)
    if last_solve is None:
        T.set_value(
            math.hypot(distance, target_height - shooter_height) / max_shooter_velocity
        )
    else:
        T.set_value(last_solve[2].value())
    dt = T / N

    # Ball state in field frame
    #
    #     [x position]
    #     [y position]
    #     [z position]
    # x = [x velocity]
    #     [y velocity]
    #     [z velocity]
    X = problem.decision_variable(6, N)
    if last_solve is not None:
        X.set_value(last_solve[3].value())

    p = X[:3, :]

    v_x = X[3, :]
    v_y = X[4, :]
    v_z = X[5, :]

    v0_wrt_shooter = X[3:, :1] - shooter_wrt_field[3:, :]

    # Shooter initial position
    problem.subject_to(p[:, :1] == shooter_wrt_field[:3, :])

    omega_magnitude = sqrt((v0_wrt_shooter.T @ v0_wrt_shooter)[0, 0]) / ball_diameter

    omega = problem.decision_variable(3, 1)
    problem.subject_to(
        omega[0, 0]
        == omega_magnitude
        * cos(atan2(v0_wrt_shooter[1, 0], v0_wrt_shooter[0, 0]) - math.pi / 2)
    )
    problem.subject_to(
        omega[1, 0]
        == omega_magnitude
        * sin(atan2(v0_wrt_shooter[1, 0], v0_wrt_shooter[0, 0]) - math.pi / 2)
    )
    problem.subject_to(omega[2, 0] == 0)
    if last_solve is None:
        omega[0, 0].set_value(-max_shooter_velocity / ball_diameter)
    else:
        omega[0, 0].set_value(-math.sqrt(last_solve[0].value()[0][0]) / ball_diameter)

    # Dynamics constraints - RK4 integration
    h = dt
    for k in range(N - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]

        k1 = f(x_k, omega)
        k2 = f(x_k + h / 2 * k1, omega)
        k3 = f(x_k + h / 2 * k2, omega)
        k4 = f(x_k + h * k3, omega)
        problem.subject_to(x_k1 == x_k + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

    # Require final position is in center of target circle
    problem.subject_to(p[:, -1] == target_wrt_field[:3, :])

    # Require the final velocity is at least somewhat downwards by limiting horizontal velocity
    # and requiring negative vertical velocity
    problem.solve(tolerance=1e-4)
    problem.subject_to(v_z[-1] < 0)

    p_x = X[0, :]
    p_y = X[1, :]
    p_z = X[2, :]

    v = X[3:, :]

    # if last_solve is None:
    # Position initial guess is linear interpolation between start and end position
    for k in range(N):
        p_x[k].set_value(lerp(shooter_wrt_field[0, 0], target_wrt_field[0, 0], k / N))
        p_y[k].set_value(lerp(shooter_wrt_field[1, 0], target_wrt_field[1, 0], k / N))
        p_z[k].set_value(lerp(shooter_wrt_field[2, 0], target_wrt_field[2, 0], k / N))

    # Velocity initial guess is max initial velocity toward target
    uvec_shooter_to_target = target_wrt_field[:3, :] - shooter_wrt_field[:3, :]
    uvec_shooter_to_target /= norm(uvec_shooter_to_target)
    for k in range(N):
        v[:, k].set_value(
            shooter_wrt_field[3:, :] + max_shooter_velocity * uvec_shooter_to_target
        )
    # else:
    #     X.set_value(last_solve[4])

    #   √(v_x² + v_y² + v_z²) ≤ v
    #   v_x² + v_y² + v_z² ≤ v²
    #   vᵀv ≤ v²
    initial_velocity_squared = v0_wrt_shooter.T @ v0_wrt_shooter

    pitch = atan2(
        v0_wrt_shooter[2, 0], hypot(v0_wrt_shooter[0, 0], v0_wrt_shooter[1, 0])
    )
    if last_solve is None:
        problem.subject_to(pitch <= max_pitch)
    else:
        problem.subject_to(pitch <= last_solve[1].value())
    problem.subject_to(pitch >= min_pitch)

    # Require initial velocity is less than max shooter velocity
    problem.subject_to(initial_velocity_squared <= max_shooter_velocity**2)

    return initial_velocity_squared, pitch, T, X


def calculate_height(shooter_height, target_height, min_distance, max_distance, samples):
    print(f"Solving shooter height {shooter_height:.2f} m, target height {target_height:.2f} m")
    problem = Problem()

    solutions = []

    T = problem.decision_variable()
    problem.subject_to(T >= 0)
    T.set_value(1)
    problem.minimize(T)

    min_distance_solve = add(problem, min_distance, shooter_height, target_height, None)
    max_distance_solve = add(problem, max_distance, shooter_height, target_height, None)
    solutions.append(min_distance_solve + (min_distance,))
    solutions.append(max_distance_solve + (max_distance,))
    status = problem.solve(tolerance=1e-4)
    if status != ExitStatus.SUCCESS:
        print("Failed to presolve ToF at shooter height")
    problem.subject_to(T == min_distance_solve[2])
    problem.subject_to(T == max_distance_solve[2])
    status = problem.solve(tolerance=1e-8)
    if status != ExitStatus.SUCCESS:
        print("Failed to solve ToF")
        return None

    tof = T.value()
    print(f"ToF: {tof} s")
    print(
        f"Min distance solve: speed={math.sqrt(min_distance_solve[0].value()[0][0]):.3f} m/s, "
        f"pitch={np.rad2deg(min_distance_solve[1].value()):.2f} deg"
    )
    print(
        f"Max distance solve: speed={math.sqrt(max_distance_solve[0].value()[0][0]):.3f} m/s, "
        f"pitch={np.rad2deg(max_distance_solve[1].value()):.2f} deg"
    )

    last_solve = min_distance_solve
    failed_solves = 0
    for i in range(1, samples):
        distance = lerp(min_distance, max_distance, i / samples)
        problem = Problem()
        solve = add(problem, distance, shooter_height, target_height, last_solve)
        status = problem.solve(tolerance=1e-4)
        if status != ExitStatus.SUCCESS:
            print(f"Warning: Failed to presolve at distance {distance}")
        problem.subject_to(tof == solve[2])
        status = problem.solve(tolerance=1e-8)
        if status != ExitStatus.SUCCESS:
            print(f"Failed to solve at distance {distance}")
            failed_solves += 1
            continue
        solutions.insert(i - failed_solves, solve + (distance,))
        last_solve = solve

    # Print formatted table of results
    print("\n" + "=" * 60)
    print(f"{'Distance (m)':>15} | {'Speed (m/s)':>15} | {'Pitch (deg)':>15}")
    print("=" * 60)

    for solution in solutions:
        speed = math.sqrt(solution[0].value()[0][0])
        pitch = np.rad2deg(solution[1].value())
        distance = solution[4]
        print(f"{distance:>15.2f} | {speed:>15.3f} | {pitch:>15.2f}")

    print("=" * 60)

    print("Writing to file")

    output_solutions = {}
    for solution in solutions:
        output_solutions[solution[4]] = {
            "speed": math.sqrt(solution[0].value()[0][0]),
            "pitch": solution[1].value(),
        }

    return {"tof": tof, "solutions": output_solutions}

min_height = 15 * 0.0254
max_height = 25 * 0.0254
height_samples = 11

def write(name, target_height, min_distance, max_distance):
    results = {}
    for i in range(height_samples):
        shooter_height = lerp(min_height, max_height, i / (height_samples - 1))
        result = calculate_height(shooter_height, target_height, min_distance,
                                                   max_distance, 45)
        if result is None:
            continue
        results[shooter_height] = result
    with open(f"../src/main/deploy/{name}.json", "w") as file:
        json.dump({"map" : results}, file, indent=2)


if __name__ == "__main__":
    write(
        "HubShotMap",
        72 * 0.0254,
        1.275,
        (math.hypot(317.7 / 2, 158.6 + 47 / 2) * 0.0254 + 14.4 / 3.281 * 1.694)
    )
    write("GroundShotMap", 0, 2, 16)
