"""Static reach only; collisions depend on timed robot occupancy."""
from . import common as c


def safe_robots(points, config):
    """Use Validator reach tolerance, without hypothetical parked-home obstacles."""
    return [robot.id for robot in config.robots
            if all(c.distance_xy(robot.base_xyz_mm, p)
                   <= robot.xy_reach_radius_mm + config.collision.geometry_epsilon_mm
                   for p in points)]
