"""Offline regression checks for the assignment path, including interpolation."""
import random
import xml.etree.ElementTree as ET
import unittest
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from gp7_motion.kinematics import Kinematics, DOWN
from gp7_motion.model import task_description

URDF = Path(__file__).resolve().parents[2] / 'gp7_description/urdf/gp7.urdf'


class MotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = Kinematics(URDF.read_text())

    def test_home_fk(self):
        # Independently summed URDF origins and the flange/tool TCP offsets.
        tcp = np.fromstring(ET.fromstring(URDF.read_text()).find("joint[@name='tool0_to_grasp_frame']/origin").get('xyz'), sep=' ')
        expected = np.array([.56, 0., .815]) + np.array([tcp[2], -tcp[1], tcp[0]])
        np.testing.assert_allclose(self.kin.fk(np.zeros(6))[:3, 3], expected, atol=1e-7)

    def test_pick_place_paths_and_controller_interpolation(self):
        rng = random.Random(7)
        x, y = rng.uniform(.50, .60), rng.uniform(-.23, -.15)
        q = self.kin.ik(np.array([x, y, .40]), np.zeros(6), restart=True)
        for target in [[x, y, .025], [x, y, .40], [.55, .35, .40], [.55, .35, .226], [.55, .35, .40]]:
            t, qs, v = self.kin.segment(q, target)
            self.assertTrue(np.all(qs >= self.kin.lower))
            self.assertTrue(np.all(qs <= self.kin.upper))
            self.assertLessEqual(np.max(np.abs(v)), .500001)
            np.testing.assert_allclose(v[[0, -1]], 0.)
            interpolated = CubicHermiteSpline(t, qs, v)(np.linspace(0., t[-1], 2*len(t)))
            for sample in interpolated:
                pose = self.kin.fk(sample)
                self.assertLess(Rotation.from_matrix(DOWN.T@pose[:3, :3]).magnitude(), 1e-4)
            np.testing.assert_allclose(self.kin.fk(qs[-1])[:3, 3], target, atol=1e-6)
            q = qs[-1]

    def test_arm_and_box_dimensions(self):
        root = ET.fromstring(URDF.read_text())
        expected = {'joint_1_s': [0., 0., .330], 'joint_2_l': [.040, 0., 0.],
                    'joint_3_u': [0., 0., .445], 'joint_4_r': [.440, 0., .040],
                    'joint_6_t-flange': [.080, 0., 0.]}
        for name, xyz in expected.items():
            actual = np.fromstring(root.find(f"joint[@name='{name}']/origin").get('xyz'), sep=' ')
            np.testing.assert_allclose(actual, xyz)
        for mesh in root.findall('.//mesh'):
            if '/gp7/' in mesh.get('filename'):
                self.assertEqual(mesh.get('scale'), '1 1 1')
        world = ET.parse(URDF.parents[2] / 'gp7_gazebo/worlds/pick_place.sdf')
        for name, size in [('box_a', [.30, .30, .20]), ('box_b', [.05, .05, .05])]:
            for item in ['collision', 'visual']:
                actual = world.find(f".//model[@name='{name}']/link/{item}/geometry/box/size").text
                np.testing.assert_allclose(np.fromstring(actual, sep=' '), size)

    def test_task_preserves_adaptive_gripper(self):
        source = ET.fromstring(URDF.read_text())
        task = ET.fromstring(task_description(URDF.read_text(), '/description', '/controllers.yaml'))
        self.assertEqual([link.get('name') for link in source.findall('link')],
                         [link.get('name') for link in task.findall('link')])
        links = [link for link in task.findall('link') if link.get('name').startswith('gripper_')]
        self.assertEqual(len(links), 7)
        for link in links:
            mesh = link.find('visual/geometry/mesh')
            self.assertIn('/adaptive_gripper/visual/', mesh.get('filename'))
            self.assertEqual(mesh.get('scale'), '1.5 1.5 1.5')
        self.assertIsNotNone(task.find("ros2_control/joint[@name='gripper_driver']/command_interface"))
        self.assertIsNone(task.find("link[@name='suction_tool']"))

    def test_unreachable_target_fails(self):
        with self.assertRaises(ValueError):
            self.kin.ik(np.array([5., 5., 5.]), np.zeros(6))


if __name__ == '__main__':
    unittest.main()
