"""Numerical FK/IK from the actual URDF; dimensions are never duplicated."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
DOWN = np.diag([1., -1., -1.])


class Kinematics:
    def __init__(self, urdf):
        root = ET.fromstring(urdf)
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], 'grasp_frame'
        while link != 'base_link':
            joint = by_child[link]
            chain.insert(0, joint)
            link = joint.find('parent').get('link')
        self.chain = []
        self.lower, self.upper, self.speed = [], [], []
        for joint in chain:
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            transform[:3, 3] = xyz
            axis = None
            if joint.get('type') == 'revolute':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
                limit = joint.find('limit')
                self.lower.append(float(limit.get('lower')))
                self.upper.append(float(limit.get('upper')))
                self.speed.append(float(limit.get('velocity')))
            self.chain.append((transform, axis))
        self.lower, self.upper, self.speed = map(np.array, (self.lower, self.upper, self.speed))

    def fk(self, q):
        result, index = np.eye(4), 0
        for origin, axis in self.chain:
            result = result @ origin
            if axis is not None:
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                result = result @ rotation
                index += 1
        return result

    def ik(self, position, seed, rotation=DOWN, restart=False):
        def residual(q):
            pose = self.fk(q)
            return np.r_[pose[:3, 3] - position,
                         Rotation.from_matrix(rotation.T @ pose[:3, :3]).as_rotvec()]
        seeds = [seed]
        if restart:
            rng = np.random.default_rng(7)
            seeds += [rng.uniform(self.lower, self.upper) for _ in range(24)]
        for guess in seeds:
            result = least_squares(residual, np.clip(guess, self.lower+1e-8, self.upper-1e-8),
                                   bounds=(self.lower, self.upper), max_nfev=150,
                                   ftol=1e-10, xtol=1e-10, gtol=1e-10)
            if np.linalg.norm(residual(result.x)) < 1e-6:
                return result.x
        raise ValueError(f'No IK solution for TCP {position}')

    def segment(self, start, target, duration=4., dt=.05):
        """Cartesian straight line, fixed orientation, quintic time scaling."""
        start = np.asarray(start)
        pose = self.fk(start)
        if Rotation.from_matrix(DOWN.T @ pose[:3, :3]).magnitude() > 1e-4:
            raise ValueError('Cartesian segment must start with the tool pointing down')
        count = max(2, int(np.ceil(duration / dt)))
        t = np.linspace(0., duration, count + 1)
        u = t / duration
        blend = 10*u**3 - 15*u**4 + 6*u**5
        qs, seed = [], start
        for s in blend:
            seed = self.ik(pose[:3, 3] + s * (np.asarray(target)-pose[:3, 3]), seed)
            qs.append(seed)
        qs = np.array(qs)
        if np.max(np.abs(np.diff(qs, axis=0))) > .15:
            raise ValueError('Discontinuous IK branch; task aborted')
        velocities = np.gradient(qs, t, axis=0)
        # Conservative 0.5 rad/s limit in addition to the URDF speed limits.
        stretch = max(1., float(np.max(np.abs(velocities) / np.minimum(self.speed, .5))))
        t *= stretch
        velocities /= stretch
        velocities[[0, -1]] = 0.
        return t, qs, velocities

    def joint_segment(self, start, target, duration=8.):
        t = np.linspace(0., duration, int(duration/.05)+1)
        u = t/duration
        delta = np.asarray(target)-start
        q = np.asarray(start) + (10*u**3-15*u**4+6*u**5)[:, None]*delta
        v = (30*u**2-60*u**3+30*u**4)[:, None]*delta/duration
        stretch = max(1., float(np.max(np.abs(v)/np.minimum(self.speed, .5))))
        return t*stretch, q, v/stretch
