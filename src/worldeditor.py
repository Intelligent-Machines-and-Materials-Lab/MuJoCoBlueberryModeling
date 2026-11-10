"""
Functions to make edits to the environment, not the plant
"""

import mujoco
import numpy as np
import logging

logger = logging.getLogger(__name__)

def add_wire_at_height_and_depth(spec, height, depth, friction=0.5):
    """
    Add a wire at the specified height and depth in the environment.

    Parameters:
    - spec: The MuJoCo model specification to modify.
    - height: The height at which to place the wire.
    - depth: The depth at which to place the wire.
    - friction: sliding friction coefficient of the wire. Must be > 0: the
      cane geoms have zero friction, and a zero-friction contact under the
      default pyramidal cone gives near-zero regularization (R ~ mu^2), which
      makes the Newton solver produce NaN accelerations.

    Assumptions:
    - Wire is a cylinder that doesn't have any bends or curves.
    - Wire is oriented parallel to y axis.
    """
    # Implementation for adding the wire goes here
    logger.info(f"Adding wire at height {height} and depth {depth}")
    world_body = None
    for body in spec.bodies:
        if body.name == "world":
            world_body = body
            break
    if world_body is None:
        raise ValueError("World body not found in the specification.")

    wire_body = world_body.add_body(
        name=f"wire_{height}_{depth}",
        pos=[depth, 0, height]
    )
    wire_body.add_geom(
        name=f"wire_geom_{height}_{depth}",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[0.01, 0.5, 0],  # radius, half-length
        pos=[0, 0, 0],
        quat=[np.sqrt(0.5), np.sqrt(0.5), 0, 0],  # rotate 90 deg about x: cylinder z-axis -> world y-axis
        # contact friction = max of the two geoms, so this overrides the canes' zero friction
        friction=[friction, 0.005, 0.0001],
    )
    return spec