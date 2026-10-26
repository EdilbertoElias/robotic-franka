# Project Documentation: Project - Aided Building
## Human-Robot Collaborative Manual Soldering & Dynamic Trust Assessment

---

## 1. Executive Summary

**Project Title:** Project - Aided Building  
**Domain:** Collaborative Robotics (Co-bots), Human-Robot Interaction (HRI), Multimodal Emotion Recognition, Safe Motion Planning.  
**Objective:** To design, implement, and evaluate a collaborative Human-Robot Interaction (HRI) framework for a micro-soldering task. The system employs a robotic arm equipped with a soldering tool to operate on a Printed Circuit Board (PCB) held dynamically by a human operator. The primary scientific and engineering goal is to measure, model, and optimize **human trust** and perceived safety by evaluating real-time multimodal behavioral/emotional cues (facial expressions, vocal prosody, and hand retraction behavior) and adapting robot trajectories to maintain comfortable clearance from the operator's hands.

---

## 2. System Architecture & Setup

### 2.1 Hardware Architecture
1. **Robotic Manipulator:** 6-DOF Collaborative Robot (Cobot) arm (e.g., UR3e or Franka Emika Production) featuring low-latency trajectory control and force feedback.
2. **End-Effector:** Custom insulated soldering iron mount with thermal shield, tip-temperature monitoring, and automatic solder feeder.
3. **Human Workstation:** ESD-safe working mat, human seat, and ergonomic PCB hand-holding fixtures.
4. **Perception & Monitoring Hardware:**
   - **RGB-D Camera Array:** Overhead/front-facing depth cameras (e.g., Intel RealSense D435i) for real-time 3D tracking of human hands, fingers, and facial keypoints.
   - **Directional Microphone Array:** High-sensitivity audio input focused on the operator's vocal acoustic features.
   - **Safety Hardware:** Hardware Emergency Stop (E-Stop), light curtains (optional enclosure boundary), and dynamic power-and-force limiting safety controllers compliant with **ISO/TS 15066**.

```
                           +----------------------------------+
                           |    Overhead RGB-D Camera         |
                           +----------------------------------+
                                     |           |
                                     v           v
+------------------+       +-----------------------+       +---------------------+
|  Human Operator  | <---> |   PCB Board (Held)    | <---> | Robot End-Effector  |
|  (Face / Voice)  |       |  (Target Workpiece)   |       | (Soldering Iron)    |
+------------------+       +-----------------------+       +---------------------+
         |                             ^                              ^
         v                             |                              |
+------------------+                   |                     +-------------------+
| Audio Mic Array  |                   +---------------------| Robotic Arm (6DOF)|
+------------------+                                         +-------------------+
```

### 2.2 Software & Control Architecture
- **Middleware:** Robot Operating System (ROS 2 / MoveIt 2).
- **Perception Subsystem:** MediaPipe / OpenPose for hand/finger 3D keypoint tracking; AffectNet/FER CNN for facial emotion recognition; librosa/OpenSMILE for speech emotion recognition (SER).
- **Behavior & Trust Engine:** Fuses real-time emotion and proximity data into a scalar **Trust Index $T(t)$**.
- **Adaptive Path Planner:** Dynamic motion planner utilizing Artificial Potential Fields (APF) / Trajectory Optimization (e.g., CHOMP/STOMP or MPC) to enforce dynamic safety margins based on operator trust.

---

## 3. Multimodal Human Emotion & Behavior Monitoring

To continuously quantify human trust and emotional discomfort, the framework integrates three continuous sensing channels:

### 3.1 Facial Expression Analysis (FEA)
- **Sensor:** High-definition front-facing RGB camera focused on the operator's head.
- **Features Tracked:** Facial Action Coding System (FACS) Action Units (AUs), specifically AUs associated with fear, anxiety, and tension (e.g., AU1 Inner Brow Raiser, AU2 Outer Brow Raiser, AU4 Brow Lowerer, AU20 Lip Stretcher).
- **Metric Output:** Real-time probability vector $P_{face}(\text{fear/distress}) \in [0, 1]$.

### 3.2 Speech & Vocal Emotion Analysis (VEA)
- **Sensor:** Directional microphone array with noise-cancellation.
- **Features Tracked:** Fundamental frequency ($F_0$), pitch variation, jitter, shimmer, Mel-Frequency Cepstral Coefficients (MFCCs), and vocal intensity.
- **Trigger Event:** Natural vocalizations or utterances (e.g., gasps, exclamations, verbal safety cues).
- **Metric Output:** Acoustic stress index $P_{voice}(\text{stress}) \in [0, 1]$.

### 3.3 Hand Tracking & Retraction Detection
- **Sensor:** RGB-D Depth Camera.
- **Features Tracked:** 3D spatial keypoints of operator wrist, palm, and individual fingertips relative to the soldering iron tip position ($p_{tool}$).
- **Key Event - Hand Retraction:** Rapid displacement of hands away from the soldering zone ($\frac{d}{dt} \|p_{hand} - p_{tool}\| > v_{threshold}$) indicates acute breach of trust or fear.

---

## 4. Operational Scenarios (Experimental Matrix)

The experiment is divided into **3 distinct operational scenarios** based on how the operator holds and stabilizes the PCB. The objective across all scenarios is to synthesize optimal robot trajectories that maximize task completion rate while keeping perceived operator risk below a critical threshold.

| Scenario | Board Handling Condition | Operator Ergonomics & Constraint | Hand Mobility / Risk Level | Primary Path Planning Constraint |
| :--- | :--- | :--- | :--- | :--- |
| **Scenario 1: Free Two-Handed Hold** | The operator holds the PCB directly using both hands without mechanical support. | High flexibility; hands are in direct proximity to soldering points on both sides of the board. | **High Risk:** Both hands close to tool; fingers prone to involuntary tremors. | Enforce spherical clearance zones around both hands; high path reactivity. |
| **Scenario 2: Single-Hand Hold with Edge Grip** | The operator holds one edge of the PCB with their dominant hand; non-dominant hand remains free/resting. | Asymmetric posture; one side of the board is unsupported/cantilevered. | **Moderate Risk:** One hand near soldering tool; free hand available for quick withdrawal. | Enforce adaptive safety corridor along the holding axis; asymmetric clearance cone. |
| **Scenario 3: Jig-Assisted Hand Support** | The PCB is placed in an ESD jig, but held down/stabilized continuously by operator fingers. | Highly constrained posture; hands resting on stable surface/jig base. | **Low-to-Moderate Risk:** Stable hand position; predictable retraction trajectories. | Strict boundary constraints along jig perimeter; minimal required buffer distance. |

---

## 5. Trust Modeling & Adaptive Trajectory Planning

### 5.1 Trust Metric Formulation
The instantaneous Trust Score $T(t) \in [0, 1]$ is computed using a weighted sensor fusion model:

$$T(t) = 1.0 - \Big[ w_1 \cdot P_{face}(\text{fear}) + w_2 \cdot P_{voice}(\text{stress}) + w_3 \cdot S_{retract}(t) + w_4 \cdot f(d_{hand})\Big]$$

Where:
- $w_1, w_2, w_3, w_4$ are normalized weight coefficients ($\sum w_i = 1$).
- $S_{retract}(t) \in \{0, 1\}$ is a binary indicator triggered upon sudden hand retraction.
- $f(d_{hand})$ is an exponential proximity penalty function:
  $$f(d_{hand}) = \exp\left(-\frac{\|p_{tool} - p_{hand}\|^2}{2\sigma^2}\right)$$

### 5.2 Dynamic Safety Distance & Path Optimization
The minimum dynamic clearance distance $D_{safe}(t)$ between the hot soldering tip and the operator's hands is adjusted adaptively according to the calculated trust level $T(t)$:

$$D_{safe}(t) = D_{min} + (1 - T(t)) \cdot D_{comfort}$$

- **$D_{min}$:** Absolute minimum mechanical safety distance (e.g., $5\text{ cm}$).
- **$D_{comfort}$:** Additional buffer distance required when trust is low (e.g., $15\text{ cm}$).

### 5.3 Path Generation Objective Function
The trajectory optimizer solves for the optimal tool path $\mathbf{q}(t)$ by minimizing the multi-objective cost function:

$$J(\mathbf{q}) = \int_{0}^{t_f} \left( \lambda_1 \|\ddot{\mathbf{x}}(t)\|^2 + \lambda_2 E_{task}(\mathbf{x}) + \lambda_3 \frac{1}{\|p_{tool}(t) - p_{hand}(t)\| - D_{safe}(t)} \right) dt$$

- **Term 1 ($\|\ddot{\mathbf{x}}\|^2$):** Minimizes movement jerk to ensure smooth, predictable robot motion (reducing human fear).
- **Term 2 ($E_{task}$):** Tool alignment and soldering accuracy error relative to the PCB joint.
- **Term 3:** Repulsive potential field cost preventing breach of the dynamic safety boundary $D_{safe}(t)$.

---

## 6. Safety Protocol & Retraction State Machine

To prevent thermal burns or mechanical collisions during hand retractions, the system implements a finite state machine (FSM):

```
                     +-----------------------+
                     |    NORMAL OPERATION   |
                     |  (Soldering Trajectory|
                     |   with Normal Speed)  |
                     +-----------------------+
                                 |
           Hand Retraction /     |     Trust Score Restored
           High Fear Detected    |     & Hands Repositioned
                                 v
                     +-----------------------+
                     |    SAFETY SLOW / HOVER|
                     | (Retract Tool 50mm,   |
                     |  Halt Solder Feed)    |
                     +-----------------------+
                                 |
           E-Stop Command /      |     Operator Re-engages
           Critical Proximity    |     and Confirms
                                 v
                     +-----------------------+
                     |     PAUSE / E-STOP    |
                     | (Power down heating,  |
                     |  Freeze robot motion) |
                     +-----------------------+
```

1. **State 1 (Normal Execution):** Trust score $T(t) \ge T_{threshold}$. Robot executes planned soldering trajectory at operational speed.
2. **State 2 (Cautious Execution / Hover):** Retraction or mild distress detected ($T_{lower} \le T(t) < T_{threshold}$). Robot halts forward progress, retracts tool vertically by $50\text{ mm}$ along $Z$-axis, and holds position until hands stabilize.
3. **State 3 (Safety Pause):** Acute fear or rapid retraction detected ($T(t) < T_{lower}$). Heating element disabled or set to standby; robot enters safe holding pose.

---

## 7. Experimental Evaluation Metrics

To rigorously assess the HRI system across all 3 scenarios, data is collected across both **objective (quantitative)** and **subjective (qualitative)** metrics:

### 7.1 Objective Metrics
1. **Hand Retraction Frequency ($N_{retract}$):** Total count of voluntary hand withdrawal events per soldering session.
2. **Minimum Separation Distance ($d_{min\_obs}$):** Smallest recorded distance between soldering tip and operator fingers.
3. **Task Completion Time ($T_{task}$):** Total time taken to complete board assembly.
4. **Physiological / Affective Stress Score:** Integrated area under curve for detected facial/vocal fear signals.
5. **Path Smoothness (Jerk Metric):** Mean square jerk of the robot end-effector trajectory.

### 7.2 Subjective Metrics
Post-trial questionnaires administered to human participants using validated psychometric scales:
- **Godspeed II Questionnaire:** Perceived Safety and Animacy subscales.
- **RoSAS (Robotic Social Attributes Scale):** Perceived Discomfort, Warmth, and Competence.
- **NASA-TLX:** Assessment of operator mental workload and stress during the task.

---

## 8. Summary of Expected Outcomes

1. **Quantified Trust Thresholds:** Identification of specific clearance distances ($D_{safe}$) required to maintain human comfort across different PCB holding postures.
2. **Adaptive Trajectory Library:** Optimized path profiles for Scenarios 1, 2, and 3 that minimize operator hand retractions without significantly compromising task efficiency.
3. **Multimodal Trust Dataset:** A multimodal dataset recording synchronized human facial expressions, audio features, hand trajectories, and robot motion states during high-risk micro-assembly tasks.

---

*Document Revision: 1.0*  
*Classification: Technical Documentation / Project Specification*