"""Read-only verification of the DH table documented in kinematics.md."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'gp7_ros2/gp7_ros2_pkg'))
from gp7_motion.kinematics import Kinematics

source = (ROOT / 'gp7_ros2/gp7_description/urdf/gp7.urdf').read_text()
root = ET.fromstring(source)
kin = Kinematics(source)
expected = {
    'joint_1_s': ([0, 0, .330], [0, 0, 1]),
    'joint_2_l': ([.040, 0, 0], [0, 1, 0]),
    'joint_3_u': ([0, 0, .445], [0, -1, 0]),
    'joint_4_r': ([.440, 0, .040], [-1, 0, 0]),
    'joint_5_b': ([0, 0, 0], [0, -1, 0]),
    'joint_6_t': ([0, 0, 0], [-1, 0, 0]),
}
for name, (xyz, axis) in expected.items():
    joint = root.find(f"joint[@name='{name}']")
    np.testing.assert_allclose(np.fromstring(joint.find('origin').get('xyz'), sep=' '), xyz, atol=1e-12)
    np.testing.assert_allclose(np.fromstring(joint.find('origin').get('rpy'), sep=' '), 0, atol=1e-12)
    np.testing.assert_allclose(np.fromstring(joint.find('axis').get('xyz'), sep=' '), axis, atol=1e-12)


def fixed(name):
    origin = root.find(f"joint[@name='{name}']/origin")
    t = np.eye(4)
    t[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
    t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
    return t


def dh_matrix(theta, d, a, alpha):
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[c, -s*ca, s*sa, a*c], [s, c*ca, -c*sa, a*s],
                     [0, sa, ca, d], [0, 0, 0, 1]])


c = np.eye(4)
c[:3, :3] = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]
tool = np.linalg.inv(c) @ fixed('joint_6_t-flange') @ fixed('flange-tool0') @ fixed('tool0_to_grasp_frame')
table = [(.040, -np.pi/2, .330, 0), (.445, np.pi, 0, -np.pi/2),
         (.040, -np.pi/2, 0, 0), (0, np.pi/2, -.440, 0),
         (0, -np.pi/2, 0, 0), (0, 0, 0, 0)]


def dh_fk(q):
    t = np.eye(4)
    for angle, (a, alpha, d, offset) in zip(q, table):
        t = t @ dh_matrix(angle + offset, d, a, alpha)
    return t @ tool


examples = [[0]*6, [.4, .6, 1.2, -.7, .8, 1.1],
            [-1, -.5, .2, 1.3, -1.1, -2], [2, 1.8, 2.7, -2.2, 1.5, 3]]
random_q = np.random.default_rng(42).uniform(kin.lower, kin.upper, (100, 6))
errors = []
for index, q in enumerate([*examples, *random_q]):
    dh, urdf = dh_fk(q), kin.fk(q)
    np.testing.assert_allclose(dh, urdf, atol=1e-12, rtol=0)
    error = [np.linalg.norm(dh[:3, 3] - urdf[:3, 3]),
             Rotation.from_matrix(dh[:3, :3].T @ urdf[:3, :3]).magnitude()]
    if index < len(examples):
        print('q:', q, 'position/rotation errors:', error)
    errors.append(error)
print('104 configurations passed; maximum position/rotation errors:', np.max(errors, axis=0))
