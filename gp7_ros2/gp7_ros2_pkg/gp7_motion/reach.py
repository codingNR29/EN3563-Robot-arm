"""Find the longest upright ground grasp in the current URDF model."""
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from gp7_motion.kinematics import DOWN


def farthest_ground_pickup(kin, margin=.005):
    """Return a near-boundary pickup and the optimized maximum radius.

    The exact boundary is a straight-elbow singularity. Keep 5 mm radial
    clearance for tracking and continuous IK. This is a kinematic envelope;
    the complete scene-specific route must also be preflighted and simulated.
    """
    if not np.isfinite(margin) or not .002 <= margin <= .10:
        raise ValueError('Reach margin must be between 2 and 100 mm')

    def constraints(q):
        pose = kin.fk(q)
        return np.r_[pose[2, 3]-.025,
                     Rotation.from_matrix(DOWN.T@pose[:3, :3]).as_rotvec()]

    candidates = []
    for angle in [-1.5, -.35, .5, 1.5]:
        seed = kin.ik([.6*np.cos(angle), .6*np.sin(angle), .025],
                      np.zeros(6), restart=True)
        result = minimize(lambda q: -np.linalg.norm(kin.fk(q)[:2, 3]), seed,
                          method='SLSQP', bounds=list(zip(kin.lower, kin.upper)),
                          constraints={'type': 'eq', 'fun': constraints},
                          options={'ftol': 1e-11, 'maxiter': 200})
        if result.success and np.linalg.norm(constraints(result.x)) < 1e-6:
            candidates.append(result)
    if not candidates:
        raise ValueError('Could not determine an upright ground-grasp reach')
    best = min(candidates, key=lambda candidate: candidate.fun)
    radius = float(-best.fun)
    pickup = kin.fk(best.x)[:3, 3].copy()
    pickup[:2] *= (radius-margin)/radius
    pickup[2] = .025
    return pickup, radius


def carry_waypoints(pickup, destination):
    """Stay within reach at full extension, then gain height while moving inward."""
    pickup = np.asarray(pickup)
    above = pickup.copy()
    far = np.linalg.norm(pickup[:2]) > .80
    above[2] = .10 if far else .40
    over_destination = np.asarray(destination).copy(); over_destination[2] = .40
    waypoints = [(pickup, 5., 'Descend to Box B'),
                 (above, 5., 'Lift Box B vertically')]
    if far:
        # Route around the base instead of cutting diagonally through it.
        waypoints.append((np.array([.45, -.45, .40]), 7., 'Raise Box B while moving inward'))
    waypoints.extend([(over_destination, 6., 'Transfer Box B upright'),
                      (np.asarray(destination)+[0., 0., .001], 4., 'Lower Box B onto Box A'),
                      (over_destination, 4., 'Retract adaptive gripper')])
    return above, waypoints
