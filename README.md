<!--
UR5e Forward & Inverse Kinematics GitHub README
This HTML is compatible with README.md on GitHub.
-->

<h1 align="center">UR5e Forward &amp; Inverse Kinematics in Python</h1>

<p align="center">
  <em>Python package for simulating and controlling a UR5e robot manipulator using forward and inverse kinematics in Webots.</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Robot-UR5e-2ea44f" alt="UR5e"/>
  <img src="https://img.shields.io/badge/Python-3.9%2B-blue" alt="Python"/>
  
</p>

<hr/>

<h2 id="-overview">🔎 Overview</h2>
<p>
This repository provides a <strong>forward and inverse kinematics simulation environment</strong> for the <strong>UR5e robot manipulator</strong> using Python and Webots. It allows control via:
<ul>
  <li>Joint-space (direct joint angles)</li>
  <li>Cartesian-space (desired end-effector pose)</li>
</ul>
The project includes tools for <strong>FK, IK, Jacobian computation, and Cartesian velocity control</strong>, with support for tool offsets and joint limits. The code is written in a complete and modularized way so that it can be used in more general cases for other types of industrial arms. Only some parameters inside the code need to be modified. For the visualization of the code and better ability to check the results, a UR5e robot is simulated inside webots.
</p>

<h2 id="-features">✨ Features</h2>
<ul>
  <li>Forward kinematics (FK) computation from joint angles.</li>
  <li>Inverse kinematics (IK) control in Cartesian space with pseudoinverse.</li>
  <li>Jacobian calculation for differential kinematics.</li>
  <li>Velocity-based Cartesian control with angle-axis error computation.</li>
  <li>Webots integration for simulation of joint motors and sensors.</li>
  <li>Joint limits, max velocity constraints, and convergence thresholds.</li>
  <li>Planning for a simplified pick and place task using pddl domain and problem.</li>
  
</ul>

<h2 id="-repo-structure">📁 Repository Structure</h2>
<pre>
UR5e_FK_IK/
├── config.py            # UR5e parameters, joint limits, control gains, and basically all the parameters that can be changed to change the type of the robot and the simulation conditions
├── geometric_model.py   # FK transformations for revolute/prismatic joints
├── kinematic_model.py   # Jacobian computation
├── cartesian_control.py # Cartesian velocity controller
├── robot_base.py        # Webots robot interface setup (sensors & actuators)
├── utils.py             # Helper functions: YPRToRot, KinematicSimulation, parse_expr
├── main.py              # Main simulation script to get the user input and implement the FK/IK control
├── My_Domain.pddl       # A pddl domain file for a pick and place task
├── My_Problem.pddl      # An example of a pick and place problem task to test the domain
└── README.md            # This README (HTML-compatible)
</pre>

<h2 id="-installation">💻 Installation</h2>
<ol>
  <li>Install Python 3.9+ and set up a virtual environment.</li>
  <li>Install required packages:
  <pre><code>pip install numpy</code></pre></li>
  <li> Install Webots and setup the environement: Webots Python API is included in the Webots installation; ensure <code>WEBOTS_HOME</code> environment variable points to your Webots folder. The folder containing the environment is also uploaded on this repository. Make sure to sset the robot controller on extern mode so that it could be controlled using the python code.</li>
  <li>Clone the repository and ensure all modules are in the same folder.</li>
</ol>

<h2 id="-usage">🧑‍💻 Usage</h2>
<ol>
  <li>Start Webots with a UR5e world. press the play button.</li>
  <li>Run the main script:
  <pre><code>python main.py</code></pre>
  </li>
  <li>Choose input mode:
    <ul>
      <li>1 → Joint angles (radians)</li>
      <li>2 → Cartesian pose (x, y, z, roll, pitch, yaw)</li>
    </ul>
  </li>
  <li>Follow on-screen prompts to enter goal configurations.
    <ul>
      <li>1 → For the first case where you enter the goal joint angles (forward kinematics), the input should be as followed: q1 q2 q3 q4 q5 q6
      It should be noted that the values should be space seperated without any brackets and commas. You can also write pi instead of writing the actual value.</li>
      <li>2 → For the Cartesian pose (inverse kinematics) the input should be as followed: x y z roll pitch yaw</li>
    </ul>
   </li>
  <li>The control loop will iteratively move the robot until the goal is reached or max iterations are exceeded. If the goal is reached the simulation stops and it shows the number of iterations it took to reach the goal. If the robot does not reach the fianl goal before the Maximum iteration reaches, it shows an error saying that the goal could not be reached. The robot reaches the goal considering the minimum and maximum joint values and by keeping a smooth motion in the middle.</li>
</ol>

<h2 id="-modules">📦 Modules</h2>
<ul>
  <li><strong>config.py:</strong> UR5e parameters, tool transform, gains, joint limits, convergence thresholds.</li>
  <li><strong>geometric_model.py:</strong> Computes FK transformations, tool frame, supports revolute and prismatic joints.</li>
  <li><strong>kinematic_model.py:</strong> Computes Jacobian, supports tool offsets, provides differential kinematics.</li>
  <li><strong>cartesian_control.py:</strong> Computes desired end-effector velocity based on Cartesian error using angle-axis representation.</li>
  <li><strong>robot_base.py:</strong> Connects to Webots sensors/motors, provides joint read/write, stepping, and tool pose extraction.</li>
  <li><strong>utils.py:</strong> Helper functions: YPR to rotation matrix, kinematic simulation with velocity/position limits, parse expressions for symbolic input.</li>
  <li><strong>main.py:</strong> Integrates all modules, runs FK/IK control loops, handles user input, performs convergence checking.</li>
</ul>

<h2 id="-math-notes">🧮 Mathematical Notes</h2>
<ul>
  <li>Forward kinematics: Computed via homogeneous transformation matrices from base to end-effector .</li>
  <li>Inverse kinematics: Solved iteratively using pseudoinverse of Jacobian, optionally using damped least squares for singularity handling.</li>
  <li>Jacobian: 6xN matrix combining linear and angular velocities, includes tool offsets if used.</li>
  <li>Cartesian control: Error in position and orientation mapped to desired end-effector velocity, converted to joint velocities via Jacobian.</li>
</ul>

<h2 id="-examples">📌 Examples</h2>
Keep in mind that after the input is given to the robot, you should wait until it reaches the goal or gives an error regardin the reachability of the robot. You can not change the goal in the middle of the movement in this version of the code. After the goal was reached and the program stops, you do not need to pause the webot simulation. You can run the python code again and insert new inputs and send the robot to new locations from the last position.
<pre><code># Example joint input mode
# 0 -pi/2 0 -pi/2 0 0
# 0 -pi/2 0 0 0 0
# pi/3 -pi/2 pi/4 0 pi/5 0
# pi/3 -pi/2 pi/4 0 pi/5 pi/3
# pi/3 0 0 0 0 0
python main.py
# Example Cartesian input mode
# 0.4 0.2 0.8 0 0 pi/3
# 0.3 0.4 0.3 0 pi/2 pi/3
python main.py
</code></pre>

<hr>

<h2>Task Planning with PDDL</h2>
<p>
For the pick and place task, it is assumed that the robot is moving towards the already known locations (pick and place locations) using the inverse kinematics algorithm. the robot in this task should also have a gripper to grab the object and drop it in the final location. At this stage the whole simulation for this part is not done yet and is still ongoing. However, in such tasks, an important part of the algorithm is the motion and task planning of the robot. Thus, a careful planning is required. A planning domain is defined over here for the robot to reach each object, grab the object, bring it to the new location and drop the object at the target location. 
</p>

<h3>Domain</h3>
<p>
The domain defines the abstract capabilities of the robot in a symbolic way. It introduces:
</p>
<ul>
  <li><b>Predicates:</b> These describe the world state, such as:
    <ul>
      <li><code>(robot_at ?r ?w)</code> – the robot’s location.</li>
      <li><code>(gripper_empty ?g)</code> / <code>(gripper_holding ?g ?o)</code> – whether the gripper is free or holding an object.</li>
      <li><code>(object_at ?o ?w)</code> – the position of objects.</li>
      <li><code>(location_clear ?w)</code> / <code>(location_accessible ?w)</code> – whether a waypoint can receive objects or be visited.</li>
      <li><code>(adjacent ?w1 ?w2)</code> – connectivity between waypoints.</li>
    </ul>
  </li>
  <li><b>Actions:</b> These define what the robot can do:
    <ul>
      <li><b>go_to:</b> Move the robot between two connected waypoints.</li>
      <li><b>pick_up:</b> Grasp an object if the gripper is empty and the robot is at the correct waypoint.</li>
      <li><b>place:</b> Place a held object at a waypoint if it is clear.</li>
    </ul>
  </li>
</ul>

<h3>Problem</h3>
<p>
The problem file specifies a concrete task instance:
</p>
<ul>
  <li><b>Objects:</b> One robot, one gripper, two manipulable objects (<code>can1</code>, <code>cube1</code>), and several waypoints (<code>home</code>, <code>table1</code>, <code>table2</code>, <code>shelf1</code>).</li>
  <li><b>Initial State:</b> The robot starts at <code>home</code> with an empty gripper, and both objects are located on <code>table1</code>. Accessibility and adjacency relations between waypoints are defined.</li>
  <li><b>Goal State:</b> The can must end up on <code>table2</code>, the cube must be placed on <code>shelf1</code>, and the robot should return to <code>home</code> with an empty gripper.</li>
</ul>

<h3>How It Works</h3>
<p>
The domain and problem together allow a planner to generate a <b>sequence of actions</b> (a plan) that achieves the goal. 
The predicates capture the logical conditions of the environment, and the actions describe how those conditions change when executed. 
If the current problem is execute to give us the proper plan, using a planner like OPTICS (Optimization Preferences and Time-dependent Costs) gives the following task sequence:
</p>

<pre>
(go_to robot1 home table1)
(pick_up robot1 gripper1 cube1 table1) 
(go_to robot1 table1 home)
(go_to robot1 home shelf1) 
(place robot1 gripper1 cube1 shelf1) 
(go_to robot1 shelf1 table2) 
(go_to robot1 table2 table1) 
(pick_up robot1 gripper1 can1 table1) 
(go_to robot1 table1 table2) 
(place robot1 gripper1 can1 table2) 
(go_to robot1 table2 shelf1)
(go_to robot1 shelf1 home)

</pre>

<p>
This high-level plan can then be mapped to motion commands via the inverse kinematics and control modules in the repository, 
bridging <b>symbolic task planning</b> with <b>robot motion execution</b>.
</p>



<h2 id="-notes">📝 Notes</h2>
<ul>
  <li>Supports up to 6-DoF manipulator; UR5e specific DH parameters included in config.</li>
  <li>Tool frame usage can be enabled/disabled via <code>USE_TOOL</code> in config.</li>
  <li>Joint limits, max velocity, and convergence thresholds can be tuned in <code>config.py</code>.</li>
  <li>Other capabilities of the code which will be added in the future are Obstacle avoidance and the complete execution of the pick and place simulation in webots. </li>
</ul>


