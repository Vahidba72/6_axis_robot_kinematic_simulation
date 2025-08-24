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
The project includes tools for <strong>FK, IK, Jacobian computation, and Cartesian velocity control</strong>, with support for tool offsets and joint limits.
</p>

<h2 id="-features">✨ Features</h2>
<ul>
  <li>Forward kinematics (FK) computation from joint angles.</li>
  <li>Inverse kinematics (IK) control in Cartesian space with pseudoinverse or damped least squares.</li>
  <li>Jacobian calculation for differential kinematics.</li>
  <li>Velocity-based Cartesian control with angle-axis error computation.</li>
  <li>Webots integration for simulation of joint motors and sensors.</li>
  <li>Joint limits, max velocity constraints, and convergence thresholds.</li>
</ul>

<h2 id="-repo-structure">📁 Repository Structure</h2>
<pre>
UR5e_FK_IK/
├── config.py            # UR5e parameters, joint limits, control gains
├── geometric_model.py   # FK transformations for revolute/prismatic joints
├── kinematic_model.py   # Jacobian computation
├── cartesian_control.py # Cartesian velocity controller
├── robot_base.py        # Webots robot interface (sensors & actuators)
├── utils.py             # Helper functions: YPRToRot, KinematicSimulation, parse_expr
├── main.py              # Main simulation script for FK/IK control
└── README.md            # This README (HTML-compatible)
</pre>

<h2 id="-installation">💻 Installation</h2>
<ol>
  <li>Install Python 3.9+ and set up a virtual environment.</li>
  <li>Install required packages:
  <pre><code>pip install numpy</code></pre>
  Webots Python API is included in the Webots installation; ensure <code>WEBOTS_HOME</code> environment variable points to your Webots folder.</li>
  <li>Clone the repository and ensure all modules are in the same folder.</li>
</ol>

<h2 id="-usage">🧑‍💻 Usage</h2>
<ol>
  <li>Start Webots with a UR5e world.</li>
  <li>Run the main script:
  <pre><code>python main.py</code></pre>
  </li>
  <li>Choose input mode:
    <ul>
      <li>1 → Joint angles (radians)</li>
      <li>2 → Cartesian pose (x, y, z, roll, pitch, yaw)</li>
    </ul>
  </li>
  <li>Follow on-screen prompts to enter goal configurations.</li>
  <li>The control loop will iteratively move the robot until the goal is reached or max iterations are exceeded.</li>
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
<pre><code># Example joint input mode
# 1  -pi/2  0  -pi/2 0 0
python main.py
# Example Cartesian input mode
# x y z roll pitch yaw
python main.py
</code></pre>

<h2 id="-notes">📝 Notes</h2>
<ul>
  <li>Supports up to 6-DoF manipulator; UR5e specific DH parameters included in config.</li>
  <li>Tool frame usage can be enabled/disabled via <code>USE_TOOL</code> in config.</li>
  <li>Joint limits, max velocity, and convergence thresholds can be tuned in <code>config.py</code>.</li>
</ul>


