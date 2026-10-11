# Standard DH model of the implemented GP7

This derivation models the [simulation URDF](../gp7_ros2/gp7_description/urdf/gp7.urdf).
It does not establish equivalence to the original CAD or introduce new dimensions.
The runtime planner still uses [URDF FK and numerical IK](../gp7_ros2/gp7_ros2_pkg/gp7_motion/kinematics.py).

## 1. Read the joint transforms

All six arm joint origins have zero RPY. At zero joint angles:

| Joint | Local axis | Translation from preceding URDF link, metres |
| --- | --- | --- |
| S, q1 | +Z | (0, 0, 0.330) |
| L, q2 | +Y | (0.040, 0, 0) |
| U, q3 | -Y | (0, 0, 0.445) |
| R, q4 | -X | (0.440, 0, 0.040) |
| B, q5 | -Y | (0, 0, 0) |
| T, q6 | -X | (0, 0, 0) |

The flange adds (0.080, 0, 0) in link_6_t. The gripper's grasp frame has a
separate fixed offset. Use that grasp frame when comparing with the runtime FK.

## 2. Define the convention

Use **standard DH**, with joint i rotating around z_(i-1):

```text
A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
```

Distances are signed: a_i is along x_i and d_i is along z_(i-1). DH frames need
not be the URDF link frames. Each frame is right-handed, with y = z cross x.

The homogeneous matrix is:

```text
       [ cosθ  -sinθ cosα   sinθ sinα   a cosθ ]
A_i =  [ sinθ   cosθ cosα  -cosθ sinα   a sinθ ]
       [  0        sinα        cosα        d   ]
       [  0         0            0         1   ]
```

## 3. Assign frames

These origins and directions are expressed in base_link at URDF zero angles.
The frames rotate with their links away from zero.

| Frame | Origin, metres | x direction | z direction |
| --- | --- | --- | --- |
| 0 | (0, 0, 0) | +X | +Z, joint 1 |
| 1 | (0.040, 0, 0.330) | +X | +Y, joint 2 |
| 2 | (0.040, 0, 0.775) | +Z | -Y, joint 3 |
| 3 | (0.040, 0, 0.815) | +Z | -X, joint 4 |
| 4 | (0.480, 0, 0.815) | +Z | -Y, joint 5 |
| 5 | (0.480, 0, 0.815) | +Z | -X, joint 6 |
| 6 | (0.480, 0, 0.815) | +Z | -X, terminal frame |

Frame 3 is at the intersection of the extended elbow and wrist-roll axes;
it is not at the URDF wrist-link origin. The three wrist axes intersect at
(0.480, 0, 0.815). Frame 6 remains there, with the flange/tool included separately.
A labeled diagram showing these frames is still needed for the report.

## 4. Derive the table

| i | a_i (m) | alpha_i (rad) | d_i (m) | theta_i (rad) |
| --- | --- | --- | --- | --- |
| 1 | 0.040 | -pi/2 | 0.330 | q1 |
| 2 | 0.445 | pi | 0 | q2 - pi/2 |
| 3 | 0.040 | -pi/2 | 0 | q3 |
| 4 | 0 | pi/2 | -0.440 | q4 |
| 5 | 0 | -pi/2 | 0 | q5 |
| 6 | 0 | 0 | 0 | q6 |

1. From frame 0, rise 0.330 m and move 0.040 m along the common normal. A
   -pi/2 twist maps +Z to the shoulder's +Y axis.
2. The upper arm is 0.445 m along home +Z. Rotating the DH angle by -pi/2 makes
   x2 point in that direction. The pi twist accounts for the opposite shoulder
   and elbow axis directions.
3. The 0.040 m elbow offset becomes a3. The -pi/2 twist maps -Y to -X.
4. Moving to the wrist center is +0.440X, whereas z3 is -X, so d4 is -0.440.
   This negative number is a frame convention, not a negative physical length.
5. The remaining wrist axes intersect, so their DH translations are zero.

The only joint-angle offset is theta2 = q2 - pi/2. All six q values keep the
URDF joint signs and order. There are no hidden joint sign reversals.

## 5. Base and tool transforms

The world-to-base and base-to-DH-frame-0 transforms are identity. Let C denote
link_6_t to DH frame 6:

```text
C = [ 0  0 -1  0 ]
    [ 0  1  0  0 ]
    [ 1  0  0  0 ]
    [ 0  0  0  1 ]
```

Then the fixed transform from DH frame 6 to grasp_frame is:

```text
T6_grasp = inverse(C) T_link6_flange T_flange_tool0 T_tool0_grasp
```

Read the last three transforms directly from the URDF:

- link6 to flange: translation (0.080, 0, 0), zero RPY.
- flange to tool0: zero translation, RPY (3.14159265, -1.57079632, 0).
- tool0 to grasp: translation (-0.0009111, 0.0123, 0.14598), zero RPY.

Approximately:

```text
T6_grasp = [ 1  0  0  -0.0009111 ]
           [ 0 -1  0  -0.0123    ]
           [ 0  0 -1  -0.22598   ]
           [ 0  0  0   1         ]
```

The verification retains the exact decimal RPY values in the URDF; replacing
them with exact pi values introduces a small, avoidable difference.

```text
T_base_grasp(q) = A1 A2 A3 A4 A5 A6 T6_grasp
```

For report calculations, substitute each row into the general matrix, show
A1 through A6, and multiply them with this tool transform. Comparing a wrist
frame with a grasp frame would give the wrong verification result.

## 6. Verify independently

Run the [documentation verification script](verify_dh.py) from the repository
root using system Python:

```bash
/usr/bin/python3 docs/verify_dh.py
```

It checks the source geometry, evaluates the table independently, then compares
it with URDF FK at four stated configurations and 100 deterministic random
configurations inside the URDF joint limits. The script only reads the model;
it is not installed into the motion package.

Recorded agreement on 11 October 2026:

| q, radians | Position difference (m) | Rotation difference (rad) |
| --- | --- | --- |
| (0, 0, 0, 0, 0, 0) | 1.32e-16 | 1.06e-16 |
| (0.4, 0.6, 1.2, -0.7, 0.8, 1.1) | 2.72e-16 | 1.46e-16 |
| (-1, -0.5, 0.2, 1.3, -1.1, -2) | 3.41e-16 | 2.01e-16 |
| (2, 1.8, 2.7, -2.2, 1.5, 3) | 2.72e-16 | 2.18e-16 |

Across the 100 samples, maximum differences were 5.50e-16 m and 4.92e-16 rad.
Home grasp position is approximately (0.705980, -0.012300, 0.814089) m. These
results validate agreement with the implemented URDF, not manufacturer CAD.

## 7. Explain the implemented numerical IK

The solver minimizes the six-component residual:

```text
r(q) = [ p_FK(q) - p_target,
         rotation_vector(R_target^T R_FK(q)) ]
```

SciPy bounded least squares respects the URDF joint limits. A previous solution
seeds each Cartesian sample to encourage a continuous branch. Optional initial
restarts use a deterministic random generator. Acceptance requires residual
norm below 1e-6; that norm combines translation and rotation components.
This is numerical IK, not a derived analytic solution. It can fail to converge
and it does not enforce collision constraints. The report should show these
equations, explain the method, and include the relevant code snippet.
