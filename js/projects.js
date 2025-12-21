// js/projects.js

document.addEventListener("DOMContentLoaded", () => {
  const projectItems = document.querySelectorAll(".project-item");

  const projectDetails = {
    haptic: `
    <h3>Haptic Robot Control – MPEG Haptics Teleoperation</h3>

    <div class="popup-image-gallery">
      <img src="images/projects/hapticaffichage.png" alt="Unity teleoperation interface" class="popup-image">
    </div>

    <p>
      This project was developed during an internship at <strong>InterDigital</strong>.
      Its goal was to explore the use of the <strong>MPEG Haptics standard</strong> in a robotic <strong>teleoperation</strong> context.
      The demonstrator enables real-time remote control of a <strong>Universal Robots UR3e</strong> arm using a
      <strong>Geomagic Touch haptic device</strong>, allowing the operator to both control the robot and feel forces from its environment.
    </p>

    <span class="popup-section-title">System Architecture</span>
    <p>
      The <strong>UR3e</strong> served as a reference platform to design and implement a <strong>standards-based haptic streaming pipeline</strong>.
      Bidirectional <em>position</em> and <em>force</em> data are encoded using the MPEG Haptics codec (MIHS stream)
      and transmitted over <strong>TCP</strong> and the <strong>RTDE interface</strong> of the robot.
    </p>

    <div class="popup-image-gallery">
      <img src="images/projects/hapticcom.png" alt="Network architecture" class="popup-image">
      <p class="img-caption">fig.1) Overview of the teleoperation pipeline based on the MPEG Haptics standard.</p>
    </div>

    <p>
      The system is composed of two machines (fig.1): the first (Unity/C#) captures the Geomagic Touch positions and orientations,
      which are <strong>encoded in MPEG Haptics format</strong> through a C++ DLL. The second (C++/Python) decodes the stream,
      sends Cartesian commands to the robot via RTDE, and sends force feedback back to the Touch device (fig.4).
    </p>

    <span class="popup-section-title">Force & Motion Mapping</span>
    <div class="popup-image-gallery">
      <img src="images/projects/hapticmapping.png" alt="Mapping " class="popup-image">
      <p class="img-caption">fig.2) Scale and apply the rotation between the Geomagic Touch and the UR3e.</p>
    </div>

    <p>
      Mapping between the robot’s end-effector and the haptic device ensures intuitive control.
      The UR3e’s workspace is scaled to match the Geomagic Touch’s limited range of motion (fig.2),
      allowing precise manipulation within the robot’s reachable volume.
    </p>

    <span class="popup-section-title">Communication Loop</span>
    <div class="popup-image-gallery">
      <img src="images/projects/hapticrtde.png" alt="Force mapping diagram" class="popup-image">
      <p class="img-caption">fig.3) RTDE communication loop for sending commands and receiving force feedback.</p>
    </div>

    <p>
      Force feedback from the UR3e is retrieved via RTDE (fig.3) and mapped to the Geomagic Touch.
      This allows the operator to feel contact forces and constraints, enhancing situational awareness during teleoperation.
      Position and rotation (as quaternions) are sent to the robot, while force feedback (Fx, Fy, Fz) is received in real time.
    </p>

    <span class="popup-section-title">Safety and Constraints</span>
    <div class="popup-image-gallery">
      <img src="images/projects/hapticsafety.png" alt="Safety zone" class="popup-image">
      <p class="img-caption">fig.4) Adjustable safety zone implemented in Unity to limit the working volume.</p>
    </div>

    <p>
      To ensure operator safety, a <strong>virtual safety zone</strong> was implemented in Unity.
      This prevents the user from exceeding the robot’s safe workspace and helps avoid collisions
      by dynamically constraining movements through force feedback.
    </p>

    <span class="popup-section-title">Operation Modes</span>
    <p>
      Two operating modes were developed:
    </p>
    <ul>
      <li><strong>Real-time streaming:</strong> live bidirectional control with force feedback and haptic interaction.</li>
      <li><strong>Playback mode:</strong> replay of pre-recorded trajectories (.hjif / .hmpg) for reproducible demonstrations.</li>
    </ul>

    <div class="popup-image-gallery">
      <img src="images/projects/hapticschema.png" alt="Communication pipeline" class="popup-image">
      <p class="img-caption">fig.5) End-to-end communication pipeline between Unity, C++, Python, and UR3e robot.</p>
    </div>

    <span class="popup-section-title">Technical Challenges</span>
    <p>
      The project combined <strong>Unity (C#)</strong> for visualization, <strong>C++</strong> for MPEG encoding/decoding,
      and <strong>Python/RTDE</strong> for robot control. Quaternion-based pose representation ensured frame consistency.
      Main challenges included <strong>latency management</strong>, <strong>frame synchronization</strong>,
      and <strong>control stability</strong> in real-time haptic feedback.
    </p>

    <span class="popup-section-title">Conclusion</span>
    <p>
      This demonstrator validated the <strong>feasibility of integrating MPEG Haptics into a robotic loop</strong>,
      highlighting its potential for applications in <em>remote manipulation</em> and <em>surgical teleoperation</em>.
    </p>
  `,


    autonomous: `
    <h3>Autonomous Mobile Robot – STM32 & ROS2 Integration</h3>

    <div class="popup-image-gallery">
      <img src="images/projects/robotros2.png" alt="Autonomous Robot" class="popup-image">
    </div>

    <p>
      This project involved the development of an <strong>autonomous mobile robot</strong> capable of operating in three distinct modes:
      <em>manual control</em>, <em>obstacle avoidance</em>, and <em>color tracking</em>.  
      The robot integrates both <strong>embedded control (STM32)</strong> and <strong>high-level vision processing (Raspberry Pi + ROS2)</strong>,
      communicating wirelessly with a <strong>PC-based HMI</strong>.
    </p>

    <span class="popup-section-title">Objectives</span>
    <p>
      The goal of this project was to design, program, and integrate a robot capable of:
    </p>
    <ul>
      <li><strong>Manual Mode:</strong> Remote control of speed and direction with obstacle detection.</li>
      <li><strong>Autonomous Mode:</strong> Random navigation with continuous obstacle avoidance.</li>
      <li><strong>Tracking Mode:</strong> Color-based object following using computer vision.</li>
    </ul>

    <span class="popup-section-title">System Architecture</span>
    <p>
      The system architecture is based on two main processing units — an <strong>STM32 Nucleo-F411</strong> microcontroller and a 
      <strong>Raspberry Pi</strong> — communicating through a serial link (<strong>UART</strong>) and integrated in a <strong>ROS2</strong> network.
    </p>

    <div class="popup-image-gallery">
      <img src="images/projects/robotros2node.svg" alt="ROS2 nodes and topics diagram" class="popup-image">
      <p class="img-caption">fig.1) ROS2 nodes and topics structure between STM32 and Raspberry Pi.</p>
    </div>

    <p>
      The <strong>STM32 Nucleo-F411</strong> is responsible for:
    </p>
    <ul>
      <li>Motor control using PWM and closed-loop regulation.</li>
      <li>Acquisition of ultrasonic and infrared distance sensors.</li>
      <li>LCD display management for real-time information.</li>
      <li>Serial communication with the Raspberry Pi through UART.</li>
    </ul>

    <p>
      The <strong>Raspberry Pi</strong> handles:
    </p>
    <ul>
      <li>Camera acquisition and <strong>image processing</strong> using <strong>OpenCV</strong>.</li>
      <li>ROS2 node management and inter-process communication.</li>
      <li>Wi-Fi communication with the <strong>PC Host</strong> HMI.</li>
    </ul>

    <span class="popup-section-title">Communication Architecture</span>
    <p>
      The robot operates under a <strong>ROS2</strong> communication framework.  
      Two primary <strong>topics</strong> are used between nodes:
    </p>
    <ul>
      <li><code>/commande/move</code> – Sends motion commands (speed, direction) from the PC Host to the STM32.</li>
      <li><code>/sensor/dist_back</code> – Publishes sensor data from the STM32 to the Raspberry Pi and PC for display and decision-making.</li>
    </ul>

    <div class="popup-image-gallery">
      <img src="images/projects/robotros2schema.png" alt="ROS2 global communication schema" class="popup-image">
      <p class="img-caption">fig.2) Global communication schema between PC Host, Raspberry Pi, and STM32 (ROS2 topics).</p>
    </div>

    <span class="popup-section-title">Human-Machine Interface</span>
    <p>
      A <strong>custom HMI</strong> running on a PC host allows users to:
    </p>
    <ul>
      <li>Select the robot's operation mode (Manual / Auto / Tracking).</li>
      <li>Send velocity and direction commands via Wi-Fi.</li>
      <li>Monitor sensor feedback and camera video stream.</li>
    </ul>

    <span class="popup-section-title">Technical Highlights</span>
    <p>
      The system leverages the following technologies:
    </p>
    <ul>
      <li><strong>STM32CubeIDE</strong> for low-level embedded programming (RTOS-based).</li>
      <li><strong>ROS2</strong> for modular communication and node orchestration.</li>
      <li><strong>OpenCV</strong> for real-time color detection and tracking.</li>
      <li><strong>Wi-Fi / UART</strong> communication for data exchange.</li>
      <li><strong>HMI (Python / Qt)</strong> for visualization and user control.</li>
    </ul>

    <span class="popup-section-title">Conclusion</span>
    <p>
      This project demonstrates the integration of <strong>real-time embedded systems</strong> and <strong>ROS2-based robotic communication</strong>,
      combining low-level motor control with high-level vision processing.
      It provided valuable experience in <strong>distributed robotics software</strong>, <strong>real-time communication</strong>,
      and <strong>multi-processor system design</strong>.
    </p>
  `,

    sensor: `
    <h3>Sensor Network – Multi-sensor CAN Bus System</h3>

    <div class="popup-image-gallery">
      <img src="images/projects/reseaucapteur.png" alt="CAN Bus Sensor Network Overview" class="popup-image">
    </div>

    <h4 class="popup-section-title"> Objective</h4>
    <p>
      The goal of this project was to design and implement a <strong>distributed sensor network</strong> where multiple
      <strong>STM32 Nucleo boards</strong> communicate over a <strong>CAN bus</strong> to send measurements to a <strong>PC Host</strong>.
      Each microcontroller is associated with a specific type of sensor and formats its data into standardized CAN frames.
    </p>

    <h4 class="popup-section-title"> System Overview</h4>
    <p>
      Physical measurements are collected by several STM32-based sensor nodes, which transmit their data via
      the <strong>Controller Area Network (CAN)</strong> protocol.  
      The host PC receives and processes these frames to visualize and record sensor information in real time.
    </p>

    <div class="popup-image-gallery">
      <img src="images/projects/rescapt.png" alt="CAN Network Communication Diagram" class="popup-image">
      <p class="img-caption">fig.1) Communication architecture of the multi-node CAN bus system.</p>
    </div>

    <h4 class="popup-section-title"> Hardware Nodes</h4>
    <p>
      The network is composed of several STM32 Nucleo boards, each assigned to a different sensing function:
    </p>
    <ul>
      <li><strong>Board 1 – Motor & Wind Sensor:</strong> Controls a <em>Robotis Dynamixel</em> servo motor and measures wind speed using a <em>Somfy anemometer</em>.</li>
      <li><strong>Board 2 – Environmental Sensors:</strong> Reads <em>luminosity</em> and <em>distance</em> from a <em>VL6180X</em> sensor, and collects <em>pressure</em> and <em>humidity</em> data via <em>LPS22HB</em> and <em>HTS221</em>.</li>
      <li><strong>Board 3 – Motion Sensor:</strong> Acquires acceleration and gyroscope data from the <em>MPU9250</em> via I²C.</li>
    </ul>

    <h4 class="popup-section-title"> Communication & Integration</h4>
    <p>
      Each board encapsulates its sensor readings in <strong>CAN frames</strong> with a unique message ID.  
      The host computer receives and interprets these frames, ensuring synchronized multi-sensor data collection.  
      CAN communication offers high reliability, prioritization, and noise immunity, ideal for embedded sensor systems.
    </p>

    <h4 class="popup-section-title"> Technical Details</h4>
    <ul>
      <li><strong>Microcontrollers:</strong> STM32 Nucleo boards programmed under <em>STM32CubeIDE</em>.</li>
      <li><strong>Communication bus:</strong> CAN.</li>
      <li><strong>Sensor interfaces:</strong> I²C for local acquisition; CAN for inter-node communication.</li>
      <li><strong>Host interface:</strong> USB-to-CAN adapter connected to the PC for data visualization.</li>
    </ul>

    <h4 class="popup-section-title"> Skills & Tools</h4>
    <p>
      <strong>STM32</strong> · <strong>CAN Bus</strong> · <strong>I²C</strong> · <strong>Embedded C</strong> · <strong>STM32CubeIDE</strong>
    </p>
  `,

    reefpulse: `
    <h3>Reef Pulse – AI for Marine Sound Recognition</h3>

    <div class="popup-image-gallery">
      <img src="images/projects/reefpulse.png" alt="Reef Pulse Project Overview" class="popup-image">
    </div>

    <h4 class="popup-section-title"> Objective</h4>
    <p>
      The goal of this project, conducted in partnership with <strong>ENIB (2023)</strong>, was to apply 
      <strong>Artificial Intelligence</strong> and <strong>Machine Learning</strong> techniques to the 
      <strong>detection and classification of marine species</strong> based on underwater acoustic signals.  
      This project is part of a global initiative to develop <em>eco-responsible monitoring tools</em> for 
      <strong>Marine Protected Areas (MPAs)</strong>.
    </p>

    <h4 class="popup-section-title"> Context</h4>
    <p>
      Coral reefs host nearly <strong>30% of marine biodiversity</strong> and support over a billion people worldwide.  
      However, due to human impact, up to <strong>90% of reefs may disappear by 2050</strong>.  
      Current visual survey methods are <em>slow, costly, and limited in time and space</em>.  
      The <strong>Reef Pulse</strong> initiative aims to overcome these limitations by using continuous underwater sound
      recordings to assess the ecological state of coral reefs in real time.
    </p>

    <p>
      By automatically detecting key biological and anthropogenic sound events, Reef Pulse provides 
      valuable indicators to guide <strong>marine conservation and management policies</strong>.
    </p>

    <h4 class="popup-section-title"> Methodology</h4>
    <p>
      The study focused on the analysis of <strong>bioacoustic data</strong> recorded on coral reef sites.  
      Using <strong>Machine Learning</strong> models, we explored how different <strong>acoustic features</strong> 
      (spectral, temporal, cepstral) affect the accuracy of species classification.
    </p>

    <div class="popup-image-gallery">
      <img src="images/projects/whale_spectrogram.png" alt="Spectrogram of underwater sound" class="popup-image">
      <p class="img-caption">fig.1) Example of spectrogram visualization used for species sound analysis.</p>
    </div>

    <p>
      Various feature extraction techniques were tested (e.g., <em>FFT, MFCC, Wavelets</em>), 
      combined with supervised classifiers such as <em>Random Forests</em>, <em>SVM</em>, and <em>XGBoost</em>.  
      The objective was to identify the best compromise between <strong>classification performance</strong> 
      and <strong>computational efficiency</strong>.
    </p>

    <h4 class="popup-section-title"> Energy-Aware AI</h4>
    <p>
      In addition to model accuracy, particular attention was paid to <strong>energy consumption</strong>.  
      We analyzed the computational complexity of each algorithm to assess their environmental footprint 
      and determine the most efficient trade-off between energy use and detection reliability. LightGBM was optimal for this.
    </p>

    <div class="popup-image-gallery">
      <img src="images/projects/reefpulseconso.png" alt="Energy efficiency comparison between ML models" class="popup-image">
      <p class="img-caption">fig.2) Energy–accuracy trade-off across different ML algorithms tested.</p>
    </div>

    <h4 class="popup-section-title"> Research Axes</h4>
    <ul>
      <li><strong>Axe 1 – Literature Review:</strong> State of the art in bioacoustics and AI applications for coral reef monitoring.</li>
      <li><strong>Axe 2 – Traditional ML Methods:</strong> Design of sound classification pipelines using conventional ML techniques.</li>
      <li><strong>Axe 3 – Comparative Study:</strong> Evaluation of performance and sustainability metrics across models.</li>
    </ul>

    <h4 class="popup-section-title"> Key Takeaways</h4>
    <p>
      This project demonstrates the potential of <strong>AI-driven acoustic monitoring</strong> to support 
      sustainable management of marine ecosystems.  
      By optimizing models for both <strong>accuracy</strong> and <strong>energy efficiency</strong>, 
      we contribute to the development of responsible and scalable technologies for marine conservation.
    </p>

    <p><em>Note: Due to confidentiality, specific datasets, algorithms, and quantitative results are not disclosed.</em></p>
  `,
    pickplace: `
    <h3>Pick-and-Place System – ABB Robot</h3>

    <div class="popup-image-gallery">
      <img src="images/projects/abbpresentation.png" alt="ABB Robot Pick-and-Place Setup" class="popup-image">
    </div>

    <h4 class="popup-section-title">Objective</h4>
    <p>
      The goal of this project was to design and program a <strong>pick-and-place robotic system</strong> using an 
      <strong>ABB industrial robot</strong>. The system was simulated and programmed in <strong>RobotStudio</strong>, 
      with real-time visual feedback provided by a camera mounted on the robot.
    </p>

    <div class="popup-image-gallery">
      <img src="images/projects/abb.png" alt="Robot Scene" class="popup-image">
      <p class="img-caption">fig.1) Scene dispositionof the ABB robot.</p>
    </div>

    <h4 class="popup-section-title">Concept</h4>
    <p>
      The robot had to identify several <strong>pucks</strong> scattered on a table, each marked with specific patterns.
      Using a <strong>Python + OpenCV</strong> vision module, the system analyzed the scene, detected the puck positions, 
      and determined their stacking order, through QrCode placed on the pucks.
    </p>

    <div class="popup-image-gallery">
      <img src="images/projects/abbqrcode.png" alt="Scene analysis with OpenCV" class="popup-image">
      <p class="img-caption">fig.2) Scene detection and object localization using OpenCV via QrCode.</p>
    </div>

    <h4 class="popup-section-title">Implementation</h4>
    <p>
      The control logic combined:
    </p>
    <ul>
      <li><strong>Python + OpenCV</strong> for image acquisition and object detection.</li>
      <li><strong>RobotStudio RAPID scripts</strong> for motion control and path planning.</li>
      <li>TCP communication between the detection script and the robot controller.</li>
    </ul>

    <h4 class="popup-section-title">Task Description</h4>
    <p>
      Once the pucks were detected, the robot performed the sequence:
    </p>
    <ol>
      <li>Move above detected puck.</li>
      <li>Pick it up using the gripper.</li>
      <li>Stack it precisely on a target location.</li>
    </ol>

    <p>
      The trajectory was optimized to minimize movement time and avoid collisions.
    </p>

    <h4 class="popup-section-title">Tools & Skills</h4>
    <ul>
      <li>ABB <strong>RobotStudio</strong></li>
      <li><strong>Python</strong> for vision processing</li>
      <li><strong>OpenCV</strong> for object detection</li>
    </ul>
  `,

    mocap: `
    <h3>Motion Capture – Unity</h3>

    <div class="popup-image-gallery">
      <img src="images/projects/mc.png" alt="Unity Motion Capture" class="popup-image">
    </div>

    <h4 class="popup-section-title">Objective</h4>
    <p>
      Development of a <strong>motion capture system</strong> integrated into <strong>Unity</strong> for 
      interactive character animation and control experiments.
    </p>

    <h4 class="popup-section-title">Description</h4>
    <p>
      The project consisted of creating a virtual agent capable of receiving and interpreting 
      input signals and replaying animations previously recorded through motion capture.
    </p>

    <div class="popup-image-gallery">
      <video class="popup-video" autoplay loop muted playsinline>
        <source src="images/projects/mcanimation.mp4" type="video/mp4">
        Your browser does not support the video tag.
      </video>
      <p class="img-caption">fig.1) 3D character tracking through Unity’s motion data pipeline.</p>
    </div>

    <h4 class="popup-section-title">Implementation</h4>
    <p>
      The captured data was processed through <strong>C# scripts</strong> to drive humanoid rig animations 
      in Unity. The animations were refined for smooth transitions and realistic timing.
    </p>

    <h4 class="popup-section-title">Tools & Skills</h4>
    <ul>
      <li><strong>Unity</strong> (C# scripting, rig animation)</li>
      <li><strong>Motion capture</strong> data integration</li>
      <li>Animation blending and timeline control</li>
    </ul>
    `,
    
    ultrasonic: `
    <h3>Ultrasonic LiDAR – ROS2 Scanning System (Arduino + Raspberry Pi)</h3>

    <div class="popup-image-gallery">
      <img src="images/projects/ultrasoniclidar.png" alt="Ultrasonic LiDAR prototype" class="popup-image">
      <p class="img-caption">Ultrasonic LiDAR prototype (HC-SR04 on servo)</p>
    </div>

    <h4 class="popup-section-title">Objective</h4>
    <p>
      Build a lightweight <strong>LIDAR-like scanner</strong> by combining an 
      <strong>Arduino Mega</strong> (servo + HC-SR04 control) and a 
      <strong>Raspberry Pi</strong> running a Python <strong>ROS2 bridge</strong>.
      The Arduino streams servo angle + distance through <code>/dev/ttyACM0</code>;  
      the Raspberry Pi converts these values into <strong>ROS2 topics</strong> and visualizes them in <strong>RViz2</strong>.
    </p>

    <h4 class="popup-section-title">Hardware & Data Flow</h4>
    <ul>
      <li><strong>Arduino Mega</strong>: drives the servo sweep and reads the ultrasonic sensor.</li>
      <li><strong>Serial link</strong> via <code>/dev/ttyACM0</code> sending angle + distance frames.</li>
      <li><strong>Raspberry Pi 4</strong>: Python script parses serial data and publishes ROS2 topics:</li>
      <ul>
        <li><code>/servo_angle</code> — <strong>Float32</strong> (angle in degrees)</li>
        <li><code>/sonar_range</code> — <strong>sensor_msgs/Range</strong> (distance in meters)</li>
        <li><code>/sonar_points</code> — <strong>MarkerArray</strong> used to display detected points</li>
      </ul>
      <li><strong>RViz2</strong>: shows the sweeping direction arrow + mapped points.</li>
    </ul>

    <h4 class="popup-section-title">Demo 1 — Angle & Distance Arrow</h4>
    <p>
      The first demo shows a <strong>real-time arrow</strong> that uses:
      <br>• the <strong>servo angle</strong> to rotate over 180°  
      <br>• the <strong>ultrasonic distance</strong> to scale its length (closer → shorter, farther → longer).
    </p>

    <div class="popup-image-gallery">
      <video class="popup-video" controls loop muted playsinline>
        <source src="images/projects/ultrasonic_angle_demo.mp4" type="video/mp4">
        Your browser does not support the video tag.
      </video>
      <p class="img-caption">Demo 1 — Arrow reacting to distance (length) and servo position (rotation).</p>
    </div>

    <h4 class="popup-section-title">Demo 2 — 2D Point Mapping</h4>
    <p>
      The second demo displays the <strong>accumulated point measurements</strong> captured during the 
      180° sweep. Each ultrasonic reading becomes a plotted point, producing a simple but functional 
      <strong>2D map of the environment</strong> inside RViz2.
    </p>

    <div class="popup-image-gallery">
      <video class="popup-video" controls loop muted playsinline>
        <source src="images/projects/ultrasonic_map_demo.mp4" type="video/mp4">
        Your browser does not support the video tag.
      </video>
      <p class="img-caption">Demo 2 — Points generated from ultrasonic data forming the scanned area.</p>
    </div>

    <h4 class="popup-section-title">Current Status & Next Steps</h4>
    <p>
      The full pipeline (Arduino → Serial → ROS2 → RViz) is operational.  
      Next steps include measurement smoothing, noise filtering, and potential shape detection.
    </p>

    <h4 class="popup-section-title">Repository</h4>
    <p>
      Full project available on GitHub:<br>
      <a href="https://github.com/FlorianWellme/Ultrasoniclidar_ROS" target="_blank">
        Ultrasoniclidar_ROS
      </a>
    </p>
  `,



    guidance: `
    <h3>Guidance & Stabilization System – In progress</h3>

    <div class="popup-image-gallery">
      <img src="images/projects/guidance.png" alt="Guidance prototype" class="popup-image">
    </div>

    <h4 class="popup-section-title">Objective</h4>
    <p>
      Design a guidance and stabilization system (prototype) for a small rocket-like platform using:
      an <strong>accelerometer</strong>, a camera, <strong>4 servomotors</strong> driven by a <strong>PCA9685</strong>,
      an <strong>Arduino</strong> (controller) and a <strong>Raspberry Pi 4</strong> as high-level computer, powered by a 10000 mAh battery.
    </p>

    <h4 class="popup-section-title">Architecture & Components</h4>
    <ul>
      <li><strong>Sensors:</strong> IMU (accelerometer/gyro), camera for visual feedback.</li>
      <li><strong>Actuation:</strong> 4 servos controlled by a PCA9685 PWM driver (I2C) from the Arduino or Pi.</li>
      <li><strong>Comms & compute:</strong> Raspberry Pi runs ROS2 nodes for high-level control; Arduino handles low-level PWM/servo safety.</li>
      <li><strong>Power:</strong> External 10 000 mAh battery for autonomy and peak current supply.</li>
    </ul>

    <h4 class="popup-section-title">Use case</h4>
    <p>
      Stabilize and control attitude of a small demonstrator (mini rocket testbed). System will estimate orientation
      from IMU + vision, compute corrective commands and send them to servos to stabilize the platform.
    </p>

    <h4 class="popup-section-title">Status</h4>
    <p>
      Project currently <strong>in progress</strong>. Core ROS2 topics & nodes are being implemented (sensor drivers, estimator, controller).
      Simulation & visualization planned in RViz and optionally Gazebo for full dynamic tests.
    </p>

    <h4 class="popup-section-title">Skills & Tools</h4>
    <ul>
      <li>ROS2 (rclpy)</li>
      <li>Arduino (C/C++)</li>
      <li>PCA9685 I2C driver</li>
      <li>IMU data fusion, simple PID/LQR controllers</li>
      <li>RViz / Gazebo (simulation)</li>
    </ul>
    `,



  };
  const formationDetails = {
  ros_eth: `
    <h3>Programming for Robotics – ROS</h3>
    <p><strong>Institution:</strong> ETH Zurich</p>

    <h4 class="popup-section-title">Abstract</h4>
    <p>
      This course provides an introduction to the <strong>Robot Operating System (ROS2)</strong>
      and its most commonly used tools in robotics. Through practical examples, it offers
      a solid foundation for working with robotic systems, from simulation to real hardware.
    </p>

    <h4 class="popup-section-title">Objectives</h4>
    <ul>
      <li>ROS2 architecture: nodes, topics, messages, services, parameters, actions</li>
      <li>Console tools for debugging and system analysis</li>
      <li>Creating ROS packages and launch files</li>
      <li>ROS2 C++ client library (<strong>rclcpp</strong>)</li>
      <li>Simulation with <strong>Gazebo</strong> and robot modeling (URDF / SDF)</li>
      <li>Visualization tools: <strong>RViz</strong> and <strong>rqt</strong></li>
      <li>Advanced concepts: TF, time handling, rosbag</li>
    </ul>

    <h4 class="popup-section-title">Content</h4>
    <p>
      The course is based on a guided tutorial with exercises of increasing difficulty.
      It covers the full setup of an autonomous robotic system using ROS2, including
      sensor integration, actuator control, and the implementation of first
      closed-loop control algorithms.
    </p>
  `
  };
 //----------------------------------------------------------------
  // ✅ Crée un seul vrai MODAL (compatible avec ton CSS)
  const modal = document.createElement("div");
  modal.classList.add("modal");
  modal.innerHTML = `
    <div class="modal-content">
      <span class="close">&times;</span>
      <div id="modal-body"></div>
    </div>
  `;
  document.body.appendChild(modal);

  const modalBody = modal.querySelector("#modal-body");
  const closeBtn = modal.querySelector(".close");

  // ✅ Ouvrir un projet (afficher le modal)
  projectItems.forEach(item => {
    item.addEventListener("click", () => {
      const key = item.dataset.project;
      modalBody.innerHTML = `<div class="popup-text">${projectDetails[key]}</div>`;
      modal.classList.add("show");
      document.body.style.overflow = "hidden"; // bloque le scroll du fond
    });
  });

  // ✅ Fermer le pop-up
  const closeModal = () => {
    modal.classList.remove("show");
    document.body.style.overflow = "";
  };

  closeBtn.addEventListener("click", closeModal);

  // Fermer si clic à l’extérieur du contenu
  modal.addEventListener("click", (e) => {
    if (e.target === modal) closeModal();
  });

  // Fermer avec la touche Échap
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") closeModal();
  });
 //----------------------------------------------------------------
  // Gestion du message caché -------------------------------------
  const hiddenMessage = document.getElementById("hiddenMessage");
  const hiddenPopup = document.getElementById("hiddenPopup");
  const closePopup = document.querySelector(".close-popup");

  hiddenMessage.addEventListener("click", () => {
    hiddenPopup.style.display = "flex";
  });

  closePopup.addEventListener("click", () => {
    hiddenPopup.style.display = "none";
  });

  window.addEventListener("click", (e) => {
    if (e.target === hiddenPopup) {
      hiddenPopup.style.display = "none";
    }
  });

  // ----------------------------------------------------------------
  // Formation modal handling
  const formationButtons = document.querySelectorAll(".formation-btn");

  formationButtons.forEach(btn => {
    btn.addEventListener("click", () => {
      const key = btn.dataset.formation;
      modalBody.innerHTML = `<div class="popup-text">${formationDetails[key]}</div>`;
      modal.classList.add("show");
      document.body.style.overflow = "hidden";
    });
  });
 //----------------------------------------------------------------
});
