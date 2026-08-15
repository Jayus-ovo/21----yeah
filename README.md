# The 21st National University Intelligent Vehicle Competition
# Technical Filing Document for "yeah" Team Smart Medical Challenge
**University: Nantong Institute of Technology　　Division: East China Division**
**Group: Undergraduate Group**
**Core Platform: D-Robotics RDK X5**

## I. Team Filing
| Filing Item | Registered Content |
| ---- | ---- |
| Team Name | yeah |
| Affiliated University | Nantong Institute of Technology |
| Team Leader | Liu Jiayi |
| Contact Number | 19851738659 |
| Team Members | Liu Jiayi, Gao Ning, Lin Jiawen, Li Jing, Wang Zihan |
| Supervisors | Sha Lei, Ding Shumei |
| Competition Track | D-Robotics Smart Medical Challenge |
| Competition Group | Undergraduate Group |
| Competition Mode | Fully Autonomous On-board Mode |

After the competition starts, the RDK X5 independently completes image capture, QR code reading, path selection, yellow lane cruising, obstacle avoidance, human figure image-to-text generation, result broadcasting, return journey and parking without receiving external control commands.

## II. Equipment Composition
### 2.1 RDK X5 Main Control Platform
The vehicle adopts D-Robotics RDK X5 as the sole main controller for all tasks. The development board is equipped with an octa-core Arm Cortex-A55 processor and a 10 TOPS INT8 BPU, running the official adapted system of D-Robotics and ROS 2 environment. Yellow lane perception and target detection are executed on the BPU side, while route scheduling, QR code decoding, mileage statistics, voice management and screen display run collaboratively on the CPU side.

The RDK X5 is obtained through official channels, and development is carried out using official system images, model conversion tools, TROS function packages and hardware interface documents. Its USB, HDMI, network, TF card and chassis communication interfaces meet the requirements of on-board perception, computing and human-machine interaction.

### 2.2 On-board Equipment List
| Module | Device | Function |
| ---- | ---- | ---- |
| Visual Input | RDK-compatible Camera | Capture images of tracks, QR codes, obstacles, lane entrances, human figure signs and Zone P |
| Motion Execution | ROS 2 Compatible Vehicle Chassis | Receive unified speed commands to complete line tracking, obstacle avoidance, steering and parking |
| Human-Machine Interaction | HDMI On-board Display, Speaker | Display QR codes and image-to-text results, broadcast directions and character descriptions simultaneously |
| Power Supply & Storage | Regulated Power Supply, Power Battery, High-speed TF Card | Provide continuous power supply and store systems, models, programs and logs |
| Structural Accessories | Heat Dissipation Components, USB Hubs, Fixing Parts and Wiring Harnesses | Ensure heat dissipation, interface expansion and overall vehicle connection reliability |

### 2.3 Connection Logic
| Camera Input | → | RDK Image Codec & Decoding | → | Dual-channel BPU Perception | → | Route Controller |
| ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| Odometer | → | Loop Length Accumulation & Return Positioning | → | Speed Arbitrator | → | Chassis Execution |
| Human Figure Screenshot | → | Image-to-Text Service | → | Screen Display + Voice Broadcast | → | Task Completion Feedback |

The camera is connected to RDK via USB interface, the display via HDMI, and the chassis driver receives speed commands through the on-board communication interface. Power circuits and computing circuits adopt separate voltage stabilization, and all wiring harnesses are fixed inside the vehicle structure.

## III. Technical Scheme
### 3.1 Dual-reference Closed-loop Task Flow
The vehicle judges the completion of full cruising in Zone C based on both "visual recheck at lane entrance" and "odometer accumulation", avoiding misjudgment of a full loop caused by single-frame landmarks or pure timing.

| Task Release Point | → | QR code recognition & direction broadcast | → | Enter Zone C via designated lane |
| ---- | ---- | ---- | ---- | ---- |
| Yellow lane tracking + mileage accumulation | → | Dual output of human figure image-to-text results | → | Exit after entrance recheck |
| Return to starting coordinate / Zone P marker | → | Precise positioning parking | → | Full vehicle static hold |

The QR code recognition module outputs raw numeric data and direction codes. Different direction codes generate opposite angular velocities for branch selection, based on which the route controller selects clockwise or counterclockwise paths. The original QR code content, driving direction and image-to-text results are displayed on the on-board panel, and the voice module broadcasts directions and character descriptions synchronously.

After entering Zone C, the centerline perception module continuously outputs the central line of the yellow lane, and the trajectory follower generates candidate speeds; the scene perception module detects obstacles, lane entrances, human figure signs and Zone P markers, and the obstacle controller generates avoidance speeds. The route controller is the sole publisher of final speed commands, selecting tracking or avoidance instructions according to risk status.

A mileage reference baseline is recorded at the start of cruising. The vehicle will switch to exit state only when it reaches the effective loop mileage, re-detects the lane entrance and obtains human figure description. During the return journey, both the starting mileage coordinate and visual Zone P marker are used to trigger precise parking.

### 3.2 Software Architecture
| Functional Unit | Implementation Content |
| ---- | ---- |
| Image Pipeline | Camera capture, JPEG/NV12 format conversion and ROS 2 image publishing |
| Path Perception | On-board yellow lane centerline model |
| Environment Perception | On-board models for obstacles, lane entrances, human figures and parking markers |
| Task Acquisition | QR code parsing, numeric verification and direction encoding |
| Route Scheduling | Eight-stage route state machine, mileage accumulation, entrance recheck and 180-second timeout limit |
| Motion Execution | Tracking candidate speed, obstacle avoidance candidate speed, command timeout detection and unified arbitration |
| Scene Understanding | Human figure screenshot capture, multimodal image-to-text generation and result feedback |
| Information Output | On-board blue information panel and TTS voice broadcast |

The project only retains three practical model capabilities: centerline detection, scene obstacle protection and human figure recognition. No model directories for N, S, GO or BACK are configured. Direction switching is driven by QR code data without repeated model switching.

### 3.3 Rule Implementation
Subtask 1: The vehicle autonomously drives to the task release point, reads the QR code, displays raw numbers and broadcasts clockwise/counterclockwise directions.
Subtask 2: The vehicle enters Zone C via the designated lane and travels along the yellow circular track in the specified direction. After detecting a human figure, it automatically captures screenshots, generates text descriptions and feeds back results via screen and voice simultaneously. It exits through Zone B lane after meeting both mileage and entrance recheck conditions.
Subtask 3: The vehicle returns to the parking area relying on starting coordinates and Zone P markers and stops fully autonomously.

Global timing starts upon main controller startup. The vehicle enters safe stop state immediately once the 180-second time limit is reached. Zero speed commands are continuously published when candidate control messages timeout, tasks are completed or programs exit.

## IV. Development Basis
1. Hardware specifications, system images and interface usage of RDK X5 are referenced from official D-Robotics documents.
2. Camera, image codec, BPU model deployment and ROS 2 communication adopt official D-Robotics TROS toolchains and sample function packages.
3. Visual models are deployed to RDK X5 following the official D-Robotics model conversion and quantization workflow.
4. Basic chassis driver adopts official ROS 2-compatible interfaces; route control, mileage loop judgment, speed arbitration, image-to-text interaction and information panel are self-developed competition application-layer programs.
5. Credentials for image-to-text access are loaded via vehicle system environment variables and not written into submitted source codes.

## V. Competition Site Guarantee
### 5.1 Startup Guarantee
An integrated launch file initializes chassis, camera, image conversion, perception, QR code processing, task control, image-to-text, screen display and voice nodes. After startup, each node publishes operating status; the master controller keeps outputting zero speed if key data is missing.

### 5.2 Runtime Guarantee
- Speed limitation is applied when the yellow lane is temporarily lost; expired old control commands will be discarded.
- Obstacle avoidance commands take priority upon obstacle detection to prevent collision or pushing obstacles.
- Mileage increment jump filtering suppresses false loop length caused by positioning frame loss.
- QR code values are verified for validity; the driving direction is locked once only at the task acquisition stage.
- Incomplete or failed human figure description requests will not mark image-to-text as completed; the on-board terminal displays processing status in real time.
- Dual triggers (visual Zone P marker and starting coordinate) are adopted for Zone P parking to improve return parking reliability.
- Zero-speed lock is activated upon 180-second timeout, normal task completion or program exit.

### 5.3 Backup Guarantee
System images, models and programs are stored on primary and spare TF cards. The team prepares spare cameras, power cables and fixing wiring harnesses. Full joint debugging covering clockwise travel, counterclockwise travel, obstacle avoidance, human figure image-to-text, entrance recheck, return parking and 180-second timeout is completed before the competition.