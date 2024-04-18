#!/usr/bin/env python3

import numpy as np
import numba
from fsd_path_planning import PathPlanner, MissionTypes
from fsd_path_planning.utils.math_utils import unit_2d_vector_from_angle, rotate
from fsd_path_planning.utils.cone_types import ConeTypes

def precompile_numba_functions():
    # Example function that might need precompilation
    def example_function(x):
        return x * x

    # Compile the example function
    numba.jit(nopython=True)(example_function)(10)

    # Assuming PathPlanner has Numba functions that need precompilation
    # Instantiate the planner with a dummy mission type
    dummy = PathPlanner(MissionTypes.trackdrive)

    phi_inner = np.arange(0, np.pi / 2, np.pi / 15)
    phi_outer = np.arange(0, np.pi / 2, np.pi / 20)

    points_inner = unit_2d_vector_from_angle(phi_inner) * 9
    points_outer = unit_2d_vector_from_angle(phi_outer) * 12

    center = np.mean((points_inner[:2] + points_outer[:2]) / 2, axis=0)
    points_inner -= center
    points_outer -= center

    rotated_points_inner = rotate(points_inner, -np.pi / 2)
    rotated_points_outer = rotate(points_outer, -np.pi / 2)
    cones_left_raw = rotated_points_inner
    cones_right_raw = rotated_points_outer


    rng = np.random.default_rng(0)
    rng.shuffle(cones_left_raw)
    rng.shuffle(cones_right_raw)


    car_position = np.array([0.0, 0.0])
    car_direction = np.array([1.0, 0.0])

    mask_is_left = np.ones(len(cones_left_raw), dtype=bool)
    mask_is_right = np.ones(len(cones_right_raw), dtype=bool)

    # for demonstration purposes, we will only keep the color of the first 4 cones
    # on each side
    mask_is_left[np.argsort(np.linalg.norm(cones_left_raw, axis=1))[4:]] = False
    mask_is_right[np.argsort(np.linalg.norm(cones_right_raw, axis=1))[4:]] = False

    cones_left = cones_left_raw[mask_is_left]
    cones_right = cones_right_raw[mask_is_right]
    cones_unknown = np.row_stack(
        [cones_left_raw[~mask_is_left], cones_right_raw[~mask_is_right]]
    )

    cones_by_type = [np.zeros((0, 2)) for _ in range(5)]
    cones_by_type[ConeTypes.LEFT] = cones_left
    cones_by_type[ConeTypes.RIGHT] = cones_right
    cones_by_type[ConeTypes.UNKNOWN] = cones_unknown

    # Call the path calculation function to trigger compilation
    # This assumes `calculate_path_in_global_frame` uses Numba
    dummy.calculate_path_in_global_frame(cones_by_type, car_position, car_direction)

if __name__ == '__main__':
    precompile_numba_functions()
    print("Compiling Numba functions...")
