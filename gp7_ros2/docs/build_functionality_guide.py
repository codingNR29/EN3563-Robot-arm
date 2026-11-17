"""Build the PDF and Markdown guide. Requires reportlab, matplotlib and numpy."""
from pathlib import Path
import subprocess
from xml.sax.saxutils import escape
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle, Image, Preformatted

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REV = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
BASE = f'https://github.com/codingNR29/EN3563-Robot-arm/blob/{REV}/gp7_ros2/'
KIN = 'gp7_ros2_pkg/gp7_motion/kinematics.py'
REACH = 'gp7_ros2_pkg/gp7_motion/reach.py'
NODE = 'gp7_ros2_pkg/scripts/pick_place_node.py'
LAUNCH = 'gp7_ros2_pkg/launch/pick_place.launch.py'
CFG = 'gp7_gazebo/config/ros2_controllers.yaml'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
fig, ax = plt.subplots(figsize=(7.4, 3.1), layout='constrained')
ax.add_patch(Circle((0, 0), .06, color='#18354a'))
ax.text(-.06, .11, 'Robot base\n(0, 0)', ha='center')
ax.add_patch(Rectangle((.475, .475), .05, .05, color='#e67c38'))
ax.text(.57, .5, 'Small box B\n(0.5, 0.5)', va='center')
ax.add_patch(Rectangle((.35, -.65), .3, .3, color='#498ab5', alpha=.8))
ax.text(.7, -.5, 'Big box A\n(0.5, -0.5)', va='center')
ax.annotate('', xy=(.5, -.33), xytext=(.5, .44), arrowprops={'arrowstyle':'->', 'lw':2, 'color':'#238b85'})
ax.text(.46, 0, 'Transfer at z = 0.40 m', rotation=90, ha='right', va='center', color='#176963')
ax.set(xlim=(-.16, 1.12), ylim=(-.74, .70), xlabel='World x (m)', ylabel='World y (m)', title='Top view: current default scene')
ax.set_aspect('equal'); ax.grid(alpha=.15)
fig.savefig(OUT/'scene_layout.png', dpi=180); plt.close(fig)
u = np.linspace(0, 1, 201)
s = 10*u**3-15*u**4+6*u**5
v = 30*u**2-60*u**3+30*u**4
fig, axes = plt.subplots(1, 2, figsize=(7.4, 2.35), layout='constrained')
axes[0].plot(u, s, color='#238b85', lw=2)
axes[0].set(title='Smooth progress along a segment', xlabel='Normalized time u = t / T', ylabel='Progress s(u)')
axes[1].plot(u, v, color='#e67c38', lw=2)
axes[1].set(title='Normalized speed profile', xlabel='Normalized time u = t / T', ylabel='ds / du (dimensionless)')
for a in axes: a.grid(alpha=.2)
fig.savefig(OUT/'time_scaling.png', dpi=180); plt.close(fig)

styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='TitleCustom', fontName='Helvetica-Bold', fontSize=25, leading=29, textColor=colors.HexColor('#18354a'), spaceAfter=14))
styles.add(ParagraphStyle(name='HeadingCustom', fontName='Helvetica-Bold', fontSize=13, leading=17, textColor=colors.HexColor('#176963'), spaceBefore=12, spaceAfter=6))
styles.add(ParagraphStyle(name='BodyCustom', fontName='Helvetica', fontSize=10, leading=14, spaceAfter=8))
styles.add(ParagraphStyle(name='SmallCustom', fontName='Helvetica', fontSize=8, leading=11, spaceAfter=5))
styles.add(ParagraphStyle(name='CellCustom', fontName='Helvetica', fontSize=8.6, leading=11))
styles.add(ParagraphStyle(name='CodeCustom', fontName='Courier', fontSize=8, leading=11, backColor=colors.HexColor('#f1f5f7'), borderPadding=8, spaceBefore=5, spaceAfter=10))
story, md = [], []
def title(t):
    story.append(Paragraph(escape(t), styles['TitleCustom'])); md.append('# '+t+'\n')
def heading(t):
    story.append(Paragraph(escape(t), styles['HeadingCustom'])); md.append('## '+t+'\n')
def p(t, small=False):
    story.append(Paragraph(escape(t), styles['SmallCustom' if small else 'BodyCustom'])); md.append(t+'\n')
def code(t):
    story.append(Preformatted(t, styles['CodeCustom'])); md.append('```text\n'+t+'\n```\n')
def source(path, line, label):
    url=BASE+path+f'#L{line}'
    text=f'{label} | {path}:{line}'
    story.append(Paragraph(f'<a href="{url}" color="#176963">{escape(text)}</a>', styles['SmallCustom']))
    md.append(f'[{text}]({url})\n')
def table(headers, rows, widths):
    data=[[Paragraph(escape(str(c)), styles['CellCustom']) for c in r] for r in [headers]+rows]
    t=Table(data, colWidths=widths, hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0), colors.HexColor('#dcebef')),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f4f7f9')]),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),8),('RIGHTPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7),('LINEBELOW',(0,0),(-1,0),.6,colors.HexColor('#9ab9c7'))]))
    story.extend([t,Spacer(1,8)])
    md.extend(['| '+' | '.join(headers)+' |', '| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows]+[''])
def img(name, h):
    story.append(Image(str(OUT/name), width=499, height=h)); md.append(f'![Diagram]({name})\n')
def page(): story.append(PageBreak())

title('GP7 simulation\nFunctionality & code guide')
p('A practical guide to inverse kinematics, trajectory planning and speed handling in this repository.')
p(f'Source snapshot: {REV[:7]}. All paths are relative to gp7_ros2/. Source references are clickable and pinned to this commit.', True)
heading('What the simulation does')
p('A six-axis GP7 arm opens its adaptive gripper, approaches the small box, lowers to grasp it, lifts and transfers it, places it on the big box, releases it and retracts. Joint and box feedback are checked throughout the sequence.')
img('scene_layout.png',209)
table(['Object / target', 'World coordinates (metres)'], [
    ['Robot base', '(0, 0, 0)'],
    ['Small box B: bottom / centre', '(0.5, 0.5, 0) / (0.5, 0.5, 0.025)'],
    ['Big box A: bottom / centre', '(0.5, -0.5, 0) / (0.5, -0.5, 0.10)'],
    ['Small box after placement: centre', '(0.5, -0.5, 0.225)']], [245,254])
p('The big box is 0.30 x 0.30 x 0.20 m; the small box is 0.05 m on each side. Ground position refers to the bottom of a box. Gazebo model poses use its centre.', True)
source('gp7_gazebo/worlds/pick_place.sdf',14,'Scene geometry and box positions')

page(); title('1. Find the functionality')
p('Start with pick_place_node.py: its run() method connects the planner, IK solver, gripper and controller.')
table(['Functionality', 'File / entry point'], [
    ['Forward kinematics: angles to tool pose', 'gp7_motion/kinematics.py : Kinematics.fk() : line 40'],
    ['Inverse kinematics: tool pose to angles', 'gp7_motion/kinematics.py : Kinematics.ik() : line 51'],
    ['Choose pickup and placement waypoints', 'gp7_motion/reach.py : carry_waypoints() : line 43'],
    ['Cartesian path + joint velocity samples', 'gp7_motion/kinematics.py : segment() : line 68'],
    ['Initial approach in joint space', 'gp7_motion/kinematics.py : joint_segment() : line 93'],
    ['Execute and check motion', 'scripts/pick_place_node.py : move() : line 127; run() : line 162'],
    ['Controller interfaces and update rate', 'gp7_gazebo/config/ros2_controllers.yaml : line 1']], [205,294])
p('The gp7_motion/ and scripts/ paths in the table are inside gp7_ros2_pkg/.')
heading('How data moves through the system')
code('Launch: world + URDF + controllers\n                    |\nPickPlace.run(): read measured joints and box pose\n                    |\ncarry_waypoints(): Cartesian targets in metres\n                    |\nsegment() -> ik() -> joint positions q(t)\n                    |\nTiming + velocity samples -> FollowJointTrajectory\n                    |\narm_controller -> gz_ros2_control -> Gazebo joints\n                    |\nJoint / box feedback -> arrival and placement checks')
source(LAUNCH,21,'Launch and startup')
source(NODE,162,'Full task sequence')
source('gp7_description/urdf/gp7.urdf',404,'Gazebo control integration')
p('The assignment task uses pick_place_node.py. arm_controller_node.py and demo_sequence_node.py provide separate preset/demo motions; they are not the numerical IK pipeline described here.', True)

page(); title('2. Inverse kinematics')
source(KIN,51,'Open Kinematics.ik()')
heading('Question answered by IK')
p('Given a desired position and orientation of the grasp frame, which six joint angles put the gripper there? Positions use metres; angles use radians. The grasp frame is the tool centre point (TCP) between the fingers.')
code('Input:  desired TCP position p, orientation R, seed q0\nOutput: q = [q1, q2, q3, q4, q5, q6]\nDefault orientation: DOWN = diag(1, -1, -1)')
heading('How this implementation solves it')
p('1. The constructor reads the chain from base_link to grasp_frame in the URDF. It collects each joint origin, rotation axis, position limits and velocity limit.')
p('2. fk(q) multiplies the joint transforms to calculate a 4 x 4 tool transform. Its translation is the predicted TCP position; its 3 x 3 rotation is the predicted orientation.')
p('3. ik() forms a six-component residual: three position errors and three orientation errors represented as a rotation vector.')
code('position_error = FK(q).translation - desired_position\nrotation_error = rotvec(R_desired.T @ R_from_FK)\nresidual       = [position_error, rotation_error]')
p('4. SciPy least_squares minimizes that residual while keeping each joint within its URDF bounds. A solution is accepted when the residual norm is below 1e-6; otherwise the function raises ValueError.')
p('5. With restart=True, the solver can try the supplied seed plus 24 reproducible random seeds. During a Cartesian segment, the previous solution seeds the next sample to encourage a continuous joint path.')
heading('Where IK is called')
source(NODE,175,'Solve the pose above pickup and preflight all Cartesian segments')
source(KIN,79,'Solve each Cartesian sample using the previous joint solution')
source(KIN,12,'Read the URDF chain and joint limits')
source(KIN,40,'Forward kinematics used inside IK')
p('This is numerical IK derived from URDF transforms. There is no separate closed-form DH inverse solution or MoveIt planning call here. The residual mixes metres and radians without explicit weighting, so its threshold is a solver acceptance test, not a physical accuracy claim.', True)

page(); title('3. Trajectory planning')
source(REACH,43,'Route selection: carry_waypoints()')
source(KIN,68,'Cartesian interpolation: segment()')
p('The route is a sequence of selected waypoints. Each Cartesian segment moves the TCP along a straight line while keeping the downward orientation fixed.')
table(['Stage', 'TCP target (m)', 'Nominal time'], [
    ['Approach above pickup', '(0.5, 0.5, 0.40)', '8 s; joint space'],
    ['Descend, then close and attach', '(0.5, 0.5, 0.025)', '5 s'],
    ['Lift', '(0.5, 0.5, 0.40)', '5 s'],
    ['Transfer above big box', '(0.5, -0.5, 0.40)', '6 s'],
    ['Lower, then detach and open', '(0.5, -0.5, 0.226)', '4 s'],
    ['Retract', '(0.5, -0.5, 0.40)', '4 s']], [197,188,114])
p('The lowering target is 1 mm above the expected resting centre (z=0.225 m); the released box settles under gravity. Segment times can grow to meet joint speed limits. Gripper and settling time are additional.', True)
heading('Smooth timing along each segment')
code('u = t / T\ns(u) = 10*u^3 - 15*u^4 + 6*u^5\np(t) = p_start + s(u) * (p_end - p_start)')
img('time_scaling.png',158)
p('The quintic blend starts and ends with zero ideal path speed and acceleration. segment() samples at a nominal 0.05 s interval, solves IK at each point and rejects adjacent joint jumps greater than 0.15 rad. Time stretching increases the final sample interval.')
p('The initial approach uses joint_segment(): it blends joint angles directly, so the TCP need not follow a straight line. The whole route is checked kinematically before the first arm move. These checks do not provide obstacle or self-collision detection.', True)

page(); title('4. Velocity & execution')
heading('Where the arm speed is controlled')
source(KIN,85,'Cartesian segment: estimate velocities and stretch time')
source(KIN,93,'Joint approach: analytic velocities and time stretching')
code('limit[j] = min(URDF_velocity_limit[j], 0.5)  # rad/s\nv = gradient(q_samples, sample_times)\nstretch = max(1, max(abs(v) / limit))\nsample_times *= stretch\nv /= stretch')
p('The Cartesian routine estimates joint speeds from the sampled joint positions. If any exceeds its limit, the entire segment slows down. Positions and the geometric path stay the same. Endpoint velocity samples are set to zero.')
p('For the initial joint-space approach, joint_segment() computes velocities analytically from the quintic derivative, then applies the same speed scaling. Example: a sampled peak of 0.8 rad/s against a 0.5 rad/s limit gives a 1.6x longer duration.')
heading('What is actually sent to the controller')
source(NODE,127,'Build and send the FollowJointTrajectory goal')
code('JointTrajectoryPoint(\n    positions=q.tolist(),\n    velocities=v.tolist(),\n    time_from_start=...)\n\nAction: /arm_controller/follow_joint_trajectory')
p('Each point contains six desired joint positions, six velocity samples and a timestamp. The YAML selects JointTrajectoryController and a position command interface. Position and velocity are available as state interfaces. The controller manager is configured at 100 Hz.')
source(CFG,15,'Arm command and state interfaces')
heading('How to describe this accurately')
p('The implementation performs velocity-limited trajectory generation and sends position trajectories with velocity samples. It does not configure a dedicated velocity-command controller or implement a custom velocity PID loop. The explicit speed cap applies to calculated samples; it is not a proof that every continuous or measured speed stays below the cap.')
heading('Gripper motion')
source(NODE,100,'Smooth gripper position commands')
p('fingers() publishes a quintic position ramp to /gripper_controller/commands, nominally every 0.02 s. Its duration uses abs(angle_change)/0.25 with a one-second minimum. The value 0.25 sets duration; it is not a hard peak velocity cap, because the quintic derivative peaks at 1.875.', True)

page(); title('5. Run, verify & modify')
heading('Start the task')
code('cd /home/banula/ros2_ws/src\n./gp7_ros2/run_pick_place.sh\n\n# Scene inspection without robot motion:\n./gp7_ros2/run_pick_place.sh run_task:=false')
p('Close the old launch with Ctrl+C before restarting. The helper selects this checkout\'s ROS environment. A successful completed task logs PICK AND PLACE VERIFIED.')
heading('Change the right settings together')
table(['Change', 'Where to edit'], [
    ['Small-box pickup coordinates', 'pick_place.launch.py:27 and node defaults:29; keep the SDF fallback pose aligned.'],
    ['Big-box location / placement target', 'pick_place.sdf:15, launch parameters:62, node defaults:32.'],
    ['Lift height and waypoint durations', 'reach.py:43 (carry_waypoints). Initial approach duration: kinematics.py:93.'],
    ['Arm speed cap', 'kinematics.py:87 and :99. Both use min(URDF limit, 0.5).'],
    ['Joint geometry / physical limits', 'gp7_description/urdf/gp7.urdf; constructor reads these into the solver.']], [180,319])
heading('What is verified')
p('Six existing offline tests passed during preparation of this guide: dimensions, home FK, default-route interpolation, far-route reach and interpolation, gripper preservation, and rejection of an unreachable target.')
code('PYTHONPATH=gp7_ros2/gp7_ros2_pkg python3 -m unittest discover \\\n  -s gp7_ros2/gp7_ros2_pkg/test -v')
source('gp7_ros2_pkg/test/test_kinematics.py',1,'Open the offline checks')
p('At runtime, move() checks action success and measured joint arrival (0.015 rad tolerance). run() checks grasp proximity (4 mm), final box position (8 mm) and orientation (1 degree), with feedback freshness checks.')
source(NODE,149,'Motion completion checks')
source(NODE,188,'Grasp and placement checks')
heading('Limits of this simulation')
p('No new Gazebo run was performed for this guide. Offline IK success does not verify collision clearance at the current box locations. The object is retained by an idealized detachable fixed joint after finger closure; this does not establish a friction-based physical grasp. Earlier recorded simulation results in PICK_PLACE.md used previous scene positions.', True)
source('gp7_ros2_pkg/gp7_motion/model.py',5,'Idealized grasp constraint')

def footer(canvas, doc):
    canvas.setStrokeColor(colors.HexColor('#dce5ea')); canvas.line(48,43,547,43)
    canvas.setFont('Helvetica',8); canvas.setFillColor(colors.HexColor('#526a79'))
    canvas.drawString(48,30,f'GP7 functionality guide | source {REV[:7]}')
    canvas.drawRightString(547,30,str(doc.page))

doc=SimpleDocTemplate(str(OUT/'GP7_Functionality_Guide.pdf'), pagesize=(595.28,841.89), rightMargin=48,leftMargin=48,topMargin=43,bottomMargin=56, title='GP7 Simulation: Functionality and Code Guide', author='GP7 project documentation')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
(OUT/'GP7_Functionality_Guide.md').write_text('\n'.join(md)+'\n')
print('Created PDF and Markdown guide in',OUT)
