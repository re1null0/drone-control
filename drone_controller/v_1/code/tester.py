import numpy as np
import itertools
import copy

from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

from flightsim.animate import animate
from flightsim.simulate import Quadrotor, simulate
from flightsim.world import World
from flightsim.axes3ds import Axes3Ds
from flightsim.drone_params import quad_params
from flightsim import hover_traj

import waypoint_traj
import se3_control

def test_gains_grid():
    """
    Systematically test different scaling combinations for the controller gains,
    measure a combined performance metric (RMS position error + weighted final time),
    and print the results.
    """

    # Base effective gains from your working configuration
    base_Kp = np.array([1, 1, 0.6])   # Outer-loop (position) gains
    base_Kd = np.array([1, 1, 0.6])

    # Inner-loop (attitude) gains
    base_KR = np.diag([5, 5, 0.05])   # For K_R
    # We'll scale K_omega as diag([5, 5, z]) * komega_scale

    # Candidate values around your known good solution:
    kp_candidates = [20, 25, 30]
    kd_candidates = [7, 10, 12]
    kr_candidates = [550, 575, 600]
    komega_z_candidates = [1, 1.5, 2.5]
    komega_candidates = [28, 30, 32]

    results = []

    for (kp_scale, kd_scale, kr_scale, komega_z_scale, komega_scale) in itertools.product(
            kp_candidates, kd_candidates, kr_candidates, komega_z_candidates, komega_candidates):
        
        print(f"Testing: K_p={kp_scale}, K_d={kd_scale}, K_R={kr_scale}, "
              f"K_omega={komega_scale}, K_omega_z={komega_z_scale}")
        
        # Create fresh instances of the quadrotor and controller
        test_quadrotor = Quadrotor(copy.deepcopy(quad_params))
        test_controller = se3_control.SE3Control(copy.deepcopy(quad_params))
        
        # Outer-loop gains
        test_controller.K_p = base_Kp * kp_scale
        test_controller.K_d = base_Kd * kd_scale
        
        # Inner-loop gains
        test_controller.K_R = base_KR * kr_scale
        # For K_omega, we only scale the third diagonal element by komega_z_scale
        # while the first two by komega_scale:
        # e.g. diag([5,5,z]) -> diag([5*komega_scale, 5*komega_scale, z*komega_z_scale])
        test_controller.K_omega = np.diag([
            5 * komega_scale,
            5 * komega_scale,
            komega_z_scale
        ])

        # Initial state
        init_state = {
            'x': np.array([-0.5, 0.5, 0.25]),
            'v': np.zeros(3),
            'q': np.array([0, 0, 0, 1]),
            'w': np.zeros(3)
        }

        # Waypoint trajectory
        points = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [1, 1, 1]
        ])
        test_traj = waypoint_traj.WaypointTraj(points)

        # Run the simulation
        T_final = 10.0
        (time_sim, state_sim, control_sim, flat_sim, exit_sim) = simulate(
            init_state,
            test_quadrotor,
            test_controller,
            test_traj,
            T_final
        )

        # Compute RMS error
        pos_errs = np.linalg.norm(state_sim['x'] - flat_sim['x'], axis=1)
        rms_err = np.sqrt(np.mean(pos_errs**2))

        # The final simulation time (could be < T_final if it finishes early)
        final_time = time_sim[-1]

        # Example combined score: smaller RMS + shorter time => better
        # Adjust 0.1 weight as needed
        score = rms_err + 0.5 * final_time

        # Store results
        result = {
            'K_p': kp_scale,
            'K_d': kd_scale,
            'K_R': kr_scale,
            'K_omega_xy': komega_scale,
            'K_omega_z': komega_z_scale,
            'rms_err': rms_err,
            'final_time': final_time,
            'score': score,
            'exit': exit_sim.value
        }
        results.append(result)

        print(f" -> RMS error={rms_err:.3f}, final_time={final_time:.2f}, "
              f"score={score:.3f}, exit={exit_sim.value}")

    # Sort by combined score
    results_sorted = sorted(results, key=lambda r: r['score'])
    best = results_sorted[0]
    print("\nBest combination by combined score (RMS + 0.5*Time):")
    print(best)
    return results_sorted

# Then call it
all_results = test_gains_grid()
