## INTRODUCTION — THE CAR THAT THINKS IN ANGLES

This document presents the engineering concept and integration framework of the WRO Future Engineers 2025 project — “The Car That Thinks in Angles.”
Created by Kyivrobomagic, it reflects our practical engineering culture: precise where it must be, experimental where it can be.

The idea was simple — to design a robot that doesn’t just move, but understands its movement.
Each subsystem — from the semi-spherical drivetrain to the sensor fusion core — was built to think in geometry, not in lines.
Mechanical, electrical, and software layers are integrated into a single responsive platform, capable of autonomous navigation and spatial reasoning.

Key design priorities include:

Adaptive geometry: torque transfer through a semi-spherical gear system allowing full-angle articulation.

Sensor fusion: LiDAR, IMU, and camera coordination for stable perception under any conditions.

Power efficiency: UPS-based 18650 system with DC-DC boost for 12 V drive and 5 V control logic.

Real-time control: multi-threaded C++ architecture ensuring predictive, low-latency motion.

Every Kyivrobomagic robot carries a small dose of what we call “engineering magic” — not mystery, but elegant simplicity that makes a complex system work naturally.
This year, that magic is geometry itself.

Table of Contents

Part 1: Team Introduction

1.1 Who We Are
1.2 Our Ideology
1.3 Team Members
1.4 Preparation for WRO 2025
1.5 Project Development Calendar

Part 2: Engineering Solution

2.1 General Ideology — The Car That Thinks in Angles
2.2 Robot Overview: Photo and General Dimensions
2.3 System Integration Specification

Part 3: Robot Construction

3.1 Building the Robot — Required Materials and Tools
3.2 Electrical Components — Core Power and Control Elements
3.3 Core Structural Elements — Mechanical and Functional Modules

Part 4: Performance Evaluation

4.1 Open Challenge Round
4.2 Obstacle Challenge Round

Part 5: Resource Library

Part 1: Team Intro

Who we are:

We are Kyivrobomagic — a robotics team born in 2018 in Kyiv, Ukraine. Our team has changed over time, but the heart of it has always been the same: Petro Moroz and Mark Biryukov. Together, we’ve grown into one of the most awarded robotics teams from Ukraine.

Our journey has been incredible. In 2020, we earned 1st place at WRO Elementary, in 2021 we took silver, then added another silver at WRO Junior in Panama in 2023, and reached the Top 10 in Turkey in 2024 in the Future Engineers category.

But our story isn’t only about WRO. Each contest has taught us something new and made us stronger as a team. They brought fresh ideas, pushed us to think like real engineers, and inspired us to go further. With every challenge, we learned not just how to build better robots, but also how to work together, solve problems creatively, and never give up.

For us, robotics is more than medals — it’s about curiosity, friendship, and proving that Ukrainian kids can compete and win on the world stage.

Our Ideology

From the very beginning, our team has always appeared at competitions dressed as wizards. We are inspired by the famous quote: “Any sufficiently advanced technology is indistinguishable from magic.”

For us, it’s far more than words. At every competition, in every solution, we strive to create something so refined and well-crafted that it amazes everyone and truly feels like magic. We spend long hours working on programming and engineering, pouring not only knowledge but also imagination and creativity into our projects.

This is our philosophy: to transform complex technologies into something that inspires, fascinates, and opens new possibilities. Technology as magic — and it is this magic that drives us forward.

### Team Members

Petro Moroz — Engineering & Software
In the team, Petro is responsible for engineering solutions, their implementation, and programming. For him, a robot is like a best friend, and every competition is a personal challenge. He always strives to come up with something new, yet as simple and elegant as possible, to solve the tasks in the most efficient way.

Petro is also a talented coder — he likes to joke that he knows more programming languages than human ones. He proved his skills by taking 2nd place in Ukraine in programming among 9th graders.

Mark Biryukov — Software & Team Coordination
Mark is responsible for software development and team coordination. He can find common ground with anyone and is a talented mathematician and programmer, a multiple prizewinner of math and programming Olympiads.

His greatest skill is his incredible speed in finding solutions. Mark quickly tests and implements ideas, helping the team move forward fast. Thanks to his speed, energy, and openness, the team easily builds connections, communicates with ease, and always keeps a fun and positive spirit. He is the team’s “social glue.”

Our fanclub: Kyrylo Moroz — Design & Identity

Our story wouldn’t be complete without mentioning our biggest fan and friend — Petro’s younger brother, Kyrylo Moroz. He is only 12 years old, but he is the one responsible for our logo, team identity, and making sure that everything about our team looks stylish and creative. Thanks to Kyrylo, our work is not only about engineering and programming, but also about design and presentation.

### Preparation to WRO 2025:

### Logistic challenge as a main issue

Because of the war in Ukraine, we had to move our work fully online. Mark is in Kyiv, while Petro is in Cherkasy, where his grandparents live. Another important point is that we work without a coach, since in Ukraine there are no trainers in this category. This makes our journey not only about robotics, but also about self-education, self-organization, and discipline — without which none of this would be possible.

The preparation itself is also a challenge because of logistics. We break it into stages:

Research & Information Gathering
We already had experience in the Future Engineers category last year, which gave us a strong foundation. Still, we spent a lot of time studying the latest solutions for autonomous systems. For example, our chassis design is based on a patented engineering idea that had never been manufactured before. Turning it from an abstract idea into a working prototype made us the first to implement it — and that’s where the “magic” of our ideology begins.

Invention & Prototyping
Most of the robot development takes place in Petro’s “home lab” in Cherkasy — actually, his and his brother’s bedroom 🙂 — equipped with a 3D printer and access to parts. We are always connected online, exchanging ideas and directions for technical development.

Programming & Testing
At this stage, we start traveling to meet each other on weekends. This means spending about three hours on the road back and forth just to work together in person, test programs, and fine-tune the design.

Finalization (Mini-Camp)
We plan the final stage as a one-week mini-camp during the autumn school break in October. This is when we meet, live and work together, and finalize the solutions we’ve been developing for months.

### Calendar

Structurally, our preparation calendar looked like this. Since we are now in our final school years, it was especially important for us to stay on schedule and avoid major overlaps with exams and studies — and we managed to do so successfully.

We divided our work into clear stages, planned around school and competitions, which allowed us to keep discipline, balance learning and robotics, and stay fully prepared for every challenge.

Part 2: Engineering solution

General Ideology – The Car That Thinks in Angles

1. From the Problem of Corners to the Geometry of Control

Traditional vehicles are built for one-dimensional motion — they move forward and backward efficiently but lose precision when direction changes. Most drivetrains are optimized for straight-line torque transfer, where every degree of deviation adds mechanical loss or instability.

This project started from a simple question: can geometry itself improve control?

Instead of treating steering angles as an obstacle, the robot uses them as part of its design logic. The structure was built around the idea that stability and control are not opposites — they can be optimized together through geometry. By rethinking how torque and direction interact, the robot achieves smoother steering and higher mechanical accuracy within a compact frame.

2. The Idea – The Car That Thinks in Angles

The robot uses a single drive motor and one servo to control all four wheels. Each wheel can rotate up to 180 degrees, which allows tight turning and stable high-speed movement without adding multiple motors. This layout is uncommon in small-scale robotics and offers excellent control with minimal components. Instead of using sensors on each wheel, the system relies on mechanical synchronization. The servo defines steering geometry, and the DC motor delivers torque. Because all wheels are mechanically linked, motion remains consistent even at high rotation angles. This design combines simplicity and reliability — fewer moving parts mean fewer points of failure, faster assembly, and easier debugging during competitions.

3. From Dimensions to Solutions

Conventional robots operate in two dimensions: forward/backward and left/right. This system introduces a third dimension — adaptive geometry. It can adjust orientation and turning angle dynamically, reacting to how the robot moves across the field. To achieve this, the robot combines three complementary sensors:

LiDAR for distance mapping and obstacle detection,

IMU for stabilizing orientation between LiDAR updates,

Camera for color, boundary, and line recognition.

Each sensor covers the weaknesses of the others. Together, they form a reliable perception system that supports precise navigation under variable lighting and surface conditions. This configuration is rare for WRO-level robots but proved highly effective in testing.

4. Key Technical Highlights

a) Unique Motion System Each wheel can rotate up to 180°, yet the robot operates with just one motor and one servo. This configuration is rare in competition and allows both high speed and precise turning.

b) Consistent Torque Delivery The drivetrain maintains torque during steering — a result of careful alignment between geometry and load distribution.

c) 3D-Printed Precision Components Core parts, including gear interfaces inspired by a patented design, were 3D-printed and tested successfully. This experiment showed that 180° and potentially 360° torque transfer is achievable at this scale.

d) Triple-Sensor Fusion Using LiDAR, IMU, and camera together increases accuracy and makes the robot resistant to lighting or surface issues that affect vision-only systems.

e) Mechanical Reliability and Future Use All mechanisms were validated under real competition conditions and performed consistently. They now serve as a mechanical base for future robot designs, expanding possibilities for stronger and more adaptive frames.

5.Summary

“The Car That Thinks in Angles” is a compact and efficient robotic system that uses minimal components to achieve full control of movement. The result is a stable, fast, and reliable platform that pushes the limits of what a single-motor robotic vehicle can do. Each part of the robot — from drive and steering to sensors and structural design — was developed through testing and iteration. Its architecture combines mechanical precision, geometric efficiency, and sensor integration in a practical way.

# System Integration Specification

1. System Architecture Overview

The robot integrates four independent drive wheel modules, each connected through a semi-spherical bevel gear system inspired by the patent US5129275A. All four wheels are driven by a single JGB37-520 12 V DC motor through two differential units — one on each side — linked by the main transmission shaft.

Steering of all wheel modules is performed simultaneously by a single global servo (JX PDI-4409MG) that provides up to 180° articulation per wheel. Each differential transfers torque to two wheel modules, which are mounted on custom 3D-printed gear housings. This arrangement allows all wheels to rotate freely at variable steering angles while maintaining balanced torque across the chassis. All subsystems are coordinated by the Raspberry Pi, which serves as the central computer, processing sensory data and issuing motion commands through PWM and digital interfaces.

Core components:

Motor: JGB37-520 DC Motor (12 V)

Motor Driver: Cytron MD13S (PWM ≤ 20 kHz)

Servo: JX PDI-4409MG Coreless Digital Servo (160° steering)

Main Controller: Raspberry Pi 4 Model B (8 GB)

Power Source: UPS HAT with 2 × 18650 Li-ion cells (7.4 V nominal)

Voltage Regulation: TPS61088 boost converter 12 V for motor

Sensors: RPLIDAR S3, BNO085 IMU, Sony IMX219 Camera

Chassis: Fully custom 3D-printed modular frame

## 2. Integration Logic

The Raspberry Pi 4B provides the central control logic.
It generates PWM signals for both the Cytron MD13S motor driver and the servo control channel.
The motor driver regulates torque and speed of the global drive motor, while the servo governs the steering geometry.

Electrical power is distributed independently to ensure system stability:

UPS HAT stores and supplies energy to the control electronics (5 V / 3.3 V logic).

TPS61088 boost converter raises the motor rail from 5 V to 12 V for torque-demanding operation.

Li-ion battery pack (2S) provides the unified power input.

Each sensor communicates directly with the Raspberry Pi:

RPLIDAR S3 via USB (mapping & obstacle detection),

BNO085 IMU via I²C (orientation and stabilization),

IMX219 Camera via CSI ribbon cable (visual recognition).

The Pi receives all sensor data, computes navigation decisions in C++, and dispatches real-time control signals back to the drivetrain and steering systems.

## 3. Mechanical Subsystem

The mechanical layout is fully custom and primarily 3D-printed. A semi-spherical bevel gear system distributes torque from the global motor into two side differentials. Each differential outputs power to two wheel modules through compact coupling shafts. This structure allows torque to propagate across the full chassis while preserving correct wheel orientation under any steering angle. This modular mechanical system minimizes friction losses, enhances alignment accuracy, and enables quick replacement of any sub-assembly.

4.Electrical Subsystem

## 5. Control Software

C++ for motor and navigation logic

Python for IMU interface

PWM control for drive and steering

Sensor fusion (LiDAR + IMU + camera)

Feedback loops for heading correction

Summary:

The system combines a single-motor + global-servo drivetrain, adaptive sensing, and efficient power management into a compact four-wheel platform.
It offers stable control, full-angle steering, and reliable power distribution within a lightweight, modular 3D-printed chassis.

Innovative abilities:

Projected Robot Dimensions

PART 3: ROBOT CONSTRUCTION

BUILDING ROBOT – NECESSARY MATERIALS AND INSTRUMENTS

### 1 3D printer Ender V3 Specification:

This is the printer personally owned and used in my workshop for all prototype and final component manufacturing. It was used to fabricate the majority of the robot’s mechanical and structural parts. Its automatic bed leveling system saves significant setup time — a notable advantage, as manual leveling is often one of the most time-consuming and error-prone steps in 3D printing. Using a 0.2 mm nozzle enabled the production of small, detailed parts with superior accuracy and smooth surfaces — critical for aligning bearings and ensuring tight tolerance in rotating assemblies.

### 2. Printing plastic

The color itself is not critical, but it can slightly affect surface texture, print strength, and layer bonding. In my experience, black PETG from CR produced the most stable and consistent results on the Ender-3 V3 printer. The printer operates optimally at 230 °C nozzle temperature, but users should experiment with small test prints to calibrate for their specific filament and environment. Mechanical and aesthetic properties vary among manufacturers, so it is recommended to check public filament test databases or community benchmarks before printing structural components.

ELECTRICAL COMPONENTS – CORE POWER AND CONTROL ELEMENTS

### 1. Raspberry Pi 4 Model B – Central Control Unit

This board serves as the central control and coordination unit of the robot. It manages all communication channels, motor coordination, and sensor data processing. Thanks to its processing capacity, it can simultaneously handle LiDAR data, IMU readings, camera input, and motor control tasks. The use of C++ for main logic ensures high-speed performance, while a small Python script interfaces the IMU (BNO085) library for simplified sensor fusion. An UPS HAT (B) module was added, providing uninterrupted power and reducing both development cost and integration time. Its combination of strong community support, abundant documentation, and available peripherals made it the most efficient platform for this project.  

### 2. Sunon KD0501PFB3-8 Micro Fan – Essential Cooling Element

Although small, this fan plays a critical role in maintaining the thermal stability of the system. Without active cooling, the Raspberry Pi 4 overheats within minutes, reaching temperatures at which the CPU throttles and can no longer operate reliably. This miniature fan dramatically improves system cooling — lowering the processor temperature enough that it remains stable and touch-safe, even under full computational load. To reduce noise and vibration, a thin layer of double-sided adhesive tape was placed between the fan and the 3D-printed mounting surface. This soft layer acts like acoustic foam, absorbing micro-vibrations and preventing the fan from resonating against the chassis. As a result, the fan becomes almost silent in operation while maintaining full airflow efficiency.

### 3. RPLIDAR S3 – High-Precision 2D LiDAR Scanner

This LiDAR sensor became a turning point in the robot’s spatial awareness system. In the previous generation, the robot relied solely on the camera for navigation, which meant it had no memory of its environment — each run started “from zero.” With the introduction of the RPLIDAR S3, the robot can now build a persistent 2D map of its surroundings and perform precise localization and path correction in real time. Unlike the camera, the LiDAR is unaffected by color, lighting, or visual illusions — problems that previously caused the robot to confuse red cubes with orange lines, or gray walls with the floor surface. This model performs exceptionally well even when the color and brightness of the environment vary, making it ideal for the WRO field where lighting can change from test to final runs. By combining LiDAR mapping with IMU-based orientation correction, the robot achieves unprecedented stability and navigational precision, allowing the vision system (camera) to focus only on critical object detection tasks rather than full scene reconstruction.

### 4.Sony IMX219 8 MP Camera Module for Raspberry Pi – High-Speed Vision System

This camera is a direct upgrade from the original Raspberry Pi Camera Module v1 used in the previous robot version. The Sony IMX219 is considered one of the most efficient and reliable sensors for compact robotic systems — even outperforming the newer Camera Module 3 in this application. Its key advantage lies in the faster data acquisition rate: when operated at 640 × 480 resolution, it can reach up to 90–100 FPS, providing real-time object tracking without increased noise levels compared to standard 30 FPS operation. In this project, the camera is responsible for detecting red and green objects and identifying walls during the Open Challenge. While it is theoretically possible to perform full navigation through LiDAR and IMU data only, this approach is computationally heavier and less efficient for tasks that require quick color and boundary detection. The camera instead delivers fast, low-latency visual input, while LiDAR and IMU perform position approximation and correction, allowing the robot to stop precisely at the required position. This combined perception system eliminates the positional drift observed last year, when the robot followed an $-shaped trajectory (“the dollar-sign problem”) and crossed the same line twice. With the high-speed vision of the IMX219, the robot can now accurately recognize lines and estimate its field position, producing a much more stable and predictable motion profile.

### 5.BNO085 9-DOF IMU Fusion Breakout (Adafruit 4754) – Inertial Compensation System

While the RPLIDAR S3 provides spatial mapping, it operates at roughly one measurement every 100 ms—an eternity in control-loop terms, equivalent to one data point per ~10 cm of travel. The BNO085 IMU fills this temporal gap, providing continuous inertial data that maintains motion awareness between LiDAR updates. Although IMU sensors inevitably drift over time, their short-term precision makes them ideal for interpolating between slower mapping samples, ensuring smooth trajectory estimation and orientation stability. Integrating this IMU was one of the most challenging technical stages of the build. Initial communication attempts via SPI resulted in unstable responses and high noise levels. After direct consultation with Bryan Siepert — the sensor’s lead developer and author of the official Adafruit tutorial — it was confirmed that SPI can be unreliable in this configuration, and I²C was recommended instead. However, the I²C link also initially failed to initialize properly. The breakthrough came from using the official Python library, which provided stable communication when launched from a lightweight Python process running in parallel with the main C++ control program. The architecture was therefore adapted: the Python IMU module handles sensor fusion and streams processed orientation data to the C++ controller via an internal data pipe. This hybrid solution successfully merged the reliability of the Python driver with the performance of the C++ control system.

### 6.Waveshare UPS HAT (B) – Uninterruptible Power Supply Module for Raspberry Pi

The Waveshare UPS HAT (B) serves as the primary power-stabilization unit for the robot’s control system, ensuring continuous operation of the Raspberry Pi 4 and peripheral electronics even under dynamic motor loads. During the previous year’s design, frequent brown-out events occurred — moments when high current draw from motors or servos caused the Raspberry Pi to momentarily lose power and reboot. This board completely eliminates that issue: its maximum current output is roughly double that of the previous power solution, allowing stable voltage under heavy load. Although physically large, which creates some mechanical integration challenges, the UPS HAT (B) provides a significant improvement in reliability. It delivers both 5 V and 3.3 V rails, along with a communication channel to report real-time battery status. When connected strategically through the GPIO pins, the power path flows through the UPS HAT rather than through the Pi, preventing voltage drops that previously triggered brown-outs.Another key feature is that this UPS continues to power the Raspberry Pi while charging, meaning the system can remain operational during energy replenishment — a capability not all UPS boards support.

### 7.DC-DC Step-Up (Boost) Module – TI TPS61088

This module plays a critical role in power conversion, raising the 5 V output from the UPS HAT (B) to 12 V required for the robot’s motor drivers (Cytron MD13S) and DC gear motors. The TPS61088 chip is widely recognized for its exceptional current capability — up to 10 A, whereas most comparable hobby-level converters can handle only 2.5 – 5 A. That higher power margin provides stable motor performance even during acceleration or direction changes, preventing voltage sag and motor desynchronization. Despite its compact size, the module allows precise adjustment of output voltage through an onboard potentiometer, and includes optional configuration headers for fine-tuning. It’s a highly versatile unit suitable not only for this robot but also for other power-intensive hobby or research projects. In this robot, the converter forms the power bridge between the logic system (5 V bus) and the high-power actuator rail (12 V). It has been tuned and calibrated for optimal stability at 12 V under dynamic motor load conditions.

### 8. Cytron MD13S – 13A 6–30V DC Motor Driver

The Cytron MD13S serves as the main motor control unit for each DC drive motor. It provides reliable, high-current performance and simplifies integration because it requires no soldering or external MOSFETs — making it a true plug-and-play solution for robotics. Compared to earlier designs using an L239D-based driver, this module proved dramatically more durable. During early testing, the previous driver failed (burned) when the robot collided with a wall and the motor produced a sudden reverse energy surge. The MD13S survived similar stress tests without issue, thanks to its robust overcurrent protection and large power-handling capacity. Although physically larger than lightweight alternatives, this board’s high current limit (13 A continuous) and thermal stability far outweigh the size drawback. Once installed, it operates flawlessly and requires virtually no maintenance — an ideal characteristic for a competition-grade robot.

### 9. JGB37-520 DC Gear Motor (12 V) – High-Power Drive Unit

This motor serves as the main drive unit for the robot’s four-wheel propulsion system, providing both forward and reverse motion. Compared with the previous year’s design, the current robot is significantly larger and heavier, requiring a motor with greater torque and acceleration capacity. The JGB37-520 12 V motor easily meets those demands. Although this model includes a planetary gearbox, the system was configured to operate without gear reduction, maximizing rotational speed and providing “brute-force” linear acceleration. During testing, the robot achieved extremely high drive speeds — delivering both high power and responsiveness. This motor was selected after prior testing confirmed that smaller 6–9 V motors lacked the required strength for dynamic maneuvers.

### 10.JX PDI-4409MG Coreless Metal Gear Digital Low Profile Servo – Steering System Actuator

This servo motor forms the core of the steering system, responsible for turning all four wheels simultaneously. While the main JGB37-520 DC motors provide linear motion and drive power, the JX PDI-4409MG is dedicated to steering and directional control. Its coreless motor architecture and metal gear transmission deliver high torque and fast response — essential for the robot’s precise angle corrections and dynamic maneuvers. The servo was chosen after multiple tests showed it could reliably sustain the mechanical load generated by the steering linkage, which transmits force to all four wheels.Even under vibration or shock (for example, during high-speed turns), the servo maintains stable performance without gear slippage or positional loss.

### MECHANICAL ELEMENTS:

### 1.Deep Groove Ball Bearing 684ZZ – Core Rotational Support Element

These bearings form the core mechanical foundation of the robot’s motion system. Whenever a component rotates, pivots, or transmits torque, it is almost certainly supported by one of these bearings. Their small size and high precision make them ideal for compact, high-speed robotic assemblies. By minimizing friction and vibration, the 684ZZ bearings enable smoother motion and improved power efficiency, allowing the motors to deliver greater torque and speed without mechanical loss. They are especially effective in this robot’s drivetrain and wheel assemblies, where compactness and stability are both critical.

### 2.Radial Deep-Groove Ball Bearing CX 61700 ZZ – Differential Support Bearing

This larger bearing was introduced after the robot’s chassis and drivetrain significantly increased in size compared to last year’s model. Many of the previous mechanical solutions could no longer support the increased torque and structural loads, particularly within the differential assembly. In earlier designs, the differential suffered from instability and lateral play (“wobbling”) because the smaller bearings could not hold the shaft firmly. The CX 61700 ZZ solved this by providing rigid radial alignment and a compact yet robust support for the differential shaft. Because of tight spatial constraints — where both the differential body and central axle had to pass through the same structural axis — the bearing’s thin profile and precision design made it an ideal choice. It allowed all components to fit within the limited geometry while maintaining mechanical rigidity and smooth rotation.

### 3.Metric Mounting Screw Set – M1, M2, M2.5 (Precision Fasteners Kit)

This small component set became an unexpected turning point in the assembly process. Originally purchased to replace missing M2.5 screws for the Raspberry Pi, the kit revealed a new scale of precision fasteners — screws so small they were nearly invisible, yet perfectly suited for compact 3D-printed assemblies. What began as a coincidence quickly became a design breakthrough. The discovery that multiple 3D-printed parts could be joined using tiny M1–M2 screws opened entirely new mechanical possibilities. Mounts that previously required glue or complex interlocking geometry could now be securely and cleanly fastened with miniature metal hardware. Beyond function, these fasteners added a sense of craftsmanship — the robot’s components could now be serviced, replaced, or adjusted like a professional machine rather than a prototype.Despite their engineering value, the screws are inexpensive, widely available, and one of the highest cost-to-performance upgrades in the project.

### 4.Threaded Rod RS PRO 530-292 – Structural Torque Transmission Element

The RS PRO 530-292 threaded rod serves as a mechanical backbone for transmitting force and rotation across the robot’s frame. Early prototypes suffered from chassis flex — even large 3D-printed beams would bend slightly under load, causing distortion in servo feedback and inaccurate motion control. By replacing flexible plastic connectors with a rigid, precision-cut steel rod, the structure gained complete torsional stability. This rod connects the servo mechanism on one side of the robot to the steering assembly on the opposite side, ensuring pure torque transfer with zero angular error. Its thread geometry allows it to screw directly into 3D-printed parts, forming a secure and compact mechanical link without adhesives or extra fittings. Even under high stress, the rod remains perfectly straight and immune to deformation — a key factor that preserves alignment between servo input and wheel actuation. Beyond this specific role, the part revealed new design opportunities: it can function as a miniature chassis rail, opening future concepts for metal-reinforced 3D-printed structures.

### 5. Tire 24 × 12 Low-Profile – LEGO Part 18977

This tire was selected because its small radius directly reduces the physical envelope of the steering assembly. Since each wheel in the robot’s drivetrain is capable of 180° rotation, any increase in tire radius would multiply the mechanical clearance required for turning. The 24 × 12 low-profile design keeps the wheel compact while maintaining a wide footprint and stable contact patch, giving excellent traction without expanding the turning circle. The rubber compound provides an ideal balance between grip and compliance: it adheres well on smooth indoor surfaces yet resists deformation during rapid direction changes. The uniform texture and round shoulders ensure predictable lateral behavior — critical when the steering servo performs fast, precise angular corrections. Even though these are LEGO components, their dimensional accuracy and material consistency are on par with precision-manufactured miniature tires, making them perfectly suited for competitive robotics.

### 6.Cables

CORE STRUCTURE ELEMENTS:

### Introduction

This section provides a detailed overview of every major printed and structural component used in the robot’s construction.
The documentation follows a logical order — from the base frame and core transmission parts to the hinge system, wheel assemblies, and specialized mounting modules.
Each description includes the function of the part, its relation to other components, and notable engineering or assembly details.

#### 1. Frame System — Base and Structural Parts

Frame Top – Mounting platform for sensors and control electronics

Main Frame – Central chassis distributing load and connecting modules

Frame Forward / Frame Back – Front and rear structural supports

Motor Mount – Interface connecting the main motor to drivetrain

Motor Gear / Motor Gear Pin – Transmission from motor to main axle

#### 2. Axle and Bearing Assembly

Main Axle & Supports – Primary torque transmission shaft

Ball Bearing Frames (Standard / with Tube / Large) – Axle stabilization and alignment

Diamond–Diamond Axle Connector – Shaft coupling and modular connection

#### 3. Differential Mechanism

Differential Core – Main torque-splitting unit

Differential Pin / Sun Gears (Long & Short) – Gear interfaces and connection shafts

Planetary Pinion – Torque balancing element

Hypoid Pinion – Angled transmission between motor and differential

#### 4. Steering and Hinge System

Hinge Frames (Robot Side / Wheel Side / Cover) – Rotational mechanism base

Hinge Connectors and Arms (L/R) – Structural links enabling wheel rotation

Hinge Caps (Standard / Gear) – Retainers ensuring stability

Hinge Supports (Standard / Open) – Bearing holders increasing rigidity

#### 5. Turn and Steering Transmission

Servo Arm / Servo Pin – Primary steering actuator and support

Turn Axle Connector – Print-in-place axle interface

Turn Geared Liftarms (Standard / Small) – Motion inversion and linkage

Turn Liftarms (Front / Back / Universal) – Steering transfer to wheel modules

Turn Liftarm Pin – Alignment and spacing component

#### 6. Wheel and Tire Assembly

Wheel Hub – Base element connecting tire and axle

Tire (LEGO Low Profile 24×12) – External traction component

#### 7. Auxiliary and Module Elements

Motor Driver Module – Left-side integration of motor driver and voltage booster

Cable Box 2 – Cable management and protective housing

Lidar Driver Module – Mount for LiDAR USB controller

1. Frame Top — Sensor and Electronics Mounting Plate

This component serves as the upper structural plate of the robot and provides mounting surfaces for all primary electronic and sensory modules. It is a 3D-printed flat frame with integrated mounting bosses and through-holes for the Raspberry Pi 4B, RPLIDAR S3, Sony IMX219 camera, BNO085 IMU, and cooling fan. All features are dimensioned to ensure that the LiDAR and IMU centers are aligned with the geometric center of the robot, which is critical for accurate navigation and mapping. The part is printed from PETG for rigidity and heat resistance and uses M2–M2.5 metric screws for fastening.

2. Main Frame — Structural Base of the Robot

The main frame is the primary structural platform that supports all mechanical and electronic subsystems. It provides the mounting points for the wheel assemblies, differential mechanisms, main drive motor, and hinge connectors for the central axle. The frame has a symmetrical layout, allowing identical wheel modules to be attached at the front and rear through paired mounting holes at both ends. Each side includes cylindrical cutouts and opposing slots for securing the differentials. Along the central length, small connectors are distributed for hinge and axle attachments, while two hexagonal cutouts in the middle serve as mounting interfaces for the rear part of the Raspberry Pi assembly.

The frame is 3D-printed with a variable thickness: 3 mm for the main surface to accommodate internal components such as the motor and battery, and 6 mm structural beams for additional stiffness and rigidity. A minor offset in the wheel axis alignment ensures that wheels can rotate freely up to 180° without interference from the chassis. This part defines the robot’s geometry, providing both load-bearing capacity and precision alignment for all motion subsystems.

3.Diamond–Diamond Axle Connector

This component serves as a universal coupling element for 4 mm axles — the standard diameter used throughout the robot due to its balance between compactness and mechanical strength. The connector has a cylindrical body with a square central socket and two perpendicular fastening holes for set screws, allowing secure attachment of two separate shafts.

It is primarily used to join the main axle with the wheel steering mechanism and the differential assembly, enabling torque transfer across segments while maintaining alignment. Although similar coupling features are integrated into other parts of the robot, this standalone connector provides modular flexibility during assembly and testing.

4. Ball Bearing Frame — Differential Support Mount

This part functions as a structural adapter for mounting a standard ball bearing onto the main frame near the differential assembly. It ensures stable fixation of the main axle, which passes through the bearing, allowing smooth and aligned rotation under load. The component features two lateral mounting holes compatible with standard metric screws used throughout the robot, enabling easy attachment to the frame.

The Ball Bearing Frame is critical for maintaining drivetrain stability. The differential gears generate significant perpendicular force during operation, and this mount provides the required rigid lateral support to prevent misalignment or bending of the main axle.

5.Ball Bearing Frame with Tube — Axle Protection and Support Mount

This modified version of the standard ball bearing frame includes an additional circular tube holder designed to support a lightweight protective tube positioned around the main axle. The tube serves as both a mechanical guard and a stabilizing sleeve, preventing contact between the rotating axle and nearby components such as the battery pack.

The assembly keeps the main axle centered and structurally secured between the differential and bearing mount. Without this part, the axle would lack lateral support and could detach under torque load. It attaches to the main frame through two standard screw holes and aligns precisely with the bearing axis for continuous, low-friction rotation.

### 6.Main Axle and Support Structure

The main axle is the core mechanical element responsible for transmitting rotational torque from the motor to both differentials. It runs through the entire length of the robot, connecting the drive gear on the motor side to the opposite differential. This part integrates several functional elements into a single printed structure, including the embedded bearing frame and protective tube, which are permanently enclosed during the printing process.

The printing process requires a controlled pause to insert a ball bearing near the motor connection point, providing additional stability and reducing radial load on the shaft. Because the part is printed vertically, special support structures are used to prevent deformation and to maintain perfect axial alignment. These supports, visible in the model, stabilize the gear teeth and ensure accurate geometry during printing.

The result is a rigid, lightweight axle capable of precise torque transfer and long-term durability, serving as the mechanical backbone of the drivetrain system.

7.Hinge Frame — Robot Side

This component forms one half of the wheel rotation mechanism and attaches directly to the main frame of the robot. It includes four mounting contact points for structural fixation and a central bearing housing to ensure precise rotational alignment of the hinge assembly. Two cylindrical pins extend outward from the frame to connect with the opposite hinge half, creating a stable pivot joint.

Adjacent to the pins are two integrated gears that synchronize the movement of both hinge sides, ensuring coordinated rotation during steering. The component provides both mechanical support and transmission alignment, allowing the wheel assembly to articulate smoothly while maintaining consistent torque transfer through the hinge system.

8.Hinge Frame — Wheel Side

This component is a compact variant of the hinge frame used on the robot side. It incorporates several design adjustments to minimize the overall length of the steering mechanism and provide space for wheel rotation. The frame includes a partial bearing housing, allowing half of the ball bearing to be seated within this part while the opposite half is enclosed by the robot-side hinge. This configuration reduces the total assembly depth and maintains secure alignment of the rotating joint.

The integrated gears are offset by one tooth relative to the robot-side hinge to prevent interference between the meshing gear teeth and to ensure smooth synchronized motion. This part is essential for connecting the wheel module to the robot’s frame, providing accurate mechanical coupling and consistent steering response.

9.Hinge Frame Cover

This component attaches to the hinge frame (wheel side) with four screws and secures the second half of the ball bearing inside the hinge assembly. Its main purpose is to lock the bearing in place, preventing it from sliding or coming loose during wheel rotation. By closing the bearing from the outer side, it completes the hinge structure and ensures smooth, stable movement of the steering mechanism. The part maintains exact alignment with the bearing axis and provides additional rigidity to the hinge system.

10. Hinge Connector and Hinge Arm (Left & Right)

These components connect the two halves of the hinge assembly from both sides, preventing separation and maintaining the correct spacing between structural elements. The hinge connector provides the main mechanical linkage, while the hinge arms (L & R) are modified versions that include additional mounting points for the steering mechanism.

Each hinge arm attaches to the corresponding wheel-side hinge, allowing the connected wheel to rotate up to 180°. The mirrored left and right versions ensure symmetrical steering behavior and consistent articulation across both sides of the robot. Together, these parts complete the hinge system that transmits motion and supports precise steering geometry.

11.Hinge Cap and Hinge Cap (Gear)

These components secure the hinge connectors in place, preventing them from slipping off the alignment pins. The standard hinge cap attaches directly to the hinge frame with screws, while the gear variation includes an additional tooth profile that provides optional reinforcement and improved resistance to lateral impact — useful when the wheel experiences shocks during operation.

During assembly, these caps must be tightened carefully: excessive force increases friction in the hinge rotation, while insufficient tightening leads to unwanted wheel play. Proper torque balance ensures smooth steering movement and mechanical reliability of the entire hinge system.

12.Sun Gear Long — Wheel Rotation and Torque Transfer Element

This component enables the 180° steering rotation of the wheel according to the patented semi-spherical gear mechanism. The gear features reinforced tooth geometry with a central rounded hub that increases rigidity and prevents deformation under load. Its extended shaft provides deep insertion into the wheel housing, ensuring a strong and stable mechanical coupling.

The design underwent multiple experimental iterations to achieve both smooth gear engagement and self-supporting structural integrity. The final configuration allows precise torque transmission through the steering axis while maintaining alignment within the hinge and differential assemblies. It plays a critical role in connecting the rotational power from the drivetrain to the articulated wheel module.

13.Sun Gear Short — Modular Torque Transmission Node

This component serves the same functional principle as the Sun Gear Long, but with a shortened shaft designed specifically for modular integration within the drivetrain.

Instead of connecting directly to a wheel assembly, the Sun Gear Short interfaces with the diamond–diamond adapter, allowing torque to be transferred between interchangeable modules of the robot.

Its compact geometry reduces spatial footprint without sacrificing alignment or strength, making it ideal for areas where precision and modularity are prioritized over reach. The rounded hub preserves the gear’s rigidity under load, while the shortened axle simplifies installation and enables quick replacement during maintenance or configuration changes.

14. Wheel Assembly — LEGO-Compatible Low-Profile Drive Interface

This wheel is designed as the primary interface between the drivetrain and the LEGO tire, providing a secure and precise connection that preserves the system’s modular compatibility.

The wheel hub includes a separate ring insert positioned in the center — a detail introduced to optimize both printing and mechanical performance. The insert reduces the overall print height, minimizing the need for heavy support structures while improving print precision around the circular perimeter.

Functionally, this design helps fine-tune the spacing between the frame and the tire, maintaining the correct alignment for torque transmission. The wheel’s outer diameter follows the LEGO standard, allowing it to fit the previously described 24 × 12 Low Profile Tire perfectly.

15. Hinge Support — Bearing Retention and Structural Reinforcement

The hinge support components are designed to securely hold the ball bearing inside the hinge frame (robot side). Their main function is to ensure proper bearing alignment and prevent axial displacement during wheel rotation. This straightforward mounting system simplifies assembly — it removes the need for complex integrated printing techniques such as “print-in-place.”

Additionally, the hinge support enhances the overall structural rigidity of the steering mechanism, preventing deformation when torque is transferred through the hinge. The open variation of the part includes a small cutout, which provides screwdriver access to one of the internal screws that connects to the rotation system.

16. Differential — Torque Distribution and Height Optimization

The differential is built around a single hypoid gear, enabling the main drive axle to sit 4 mm lower than the wheel axis — a structural decision that significantly reduces the overall height of the robot. This compact design contributes to a lower center of gravity and improved stability during motion.

The differential casing is laterally offset due to the geometry of the hypoid gear, but this shift does not interfere with performance. Beneath the gear sits a connector mount for a large ball bearing, providing stability under rotational load.

Functionally, the differential plays a crucial role in torque distribution — it ensures both wheels receive appropriate power during turns. This is essential because each wheel follows its own path radius, and without differential control, one wheel would slip or drag. The combination of precision gearing, ball bearing support, and structural alignment allows this differential to deliver smooth, balanced power transfer across the drivetrain.

17. Differential Pin — Secondary Bearing Coupler

The Differential Pin is the companion component to the main differential housing, serving as the rear coupling interface that connects the differential assembly to the large ball bearing. It ensures that the rotational load from the hypoid gear is evenly distributed and transmitted to the support structure, maintaining stability under torque stress.

Its circular flange with three mounting holes allows for secure attachment to the differential body while aligning precisely with the bearing mount. The pin’s centered bore provides a tight fit for the axle shaft, ensuring smooth, low-friction rotation. Though small, this part is essential for maintaining mechanical precision in the robot’s drivetrain — without it, the entire differential system would lose its axial support and alignment.

18. Differential Sun Gear (Long and Short) — Angular Torque Transfer Pair

The differential sun gears form the active core of the hypoid transmission system, responsible for translating motor torque into balanced rotational output for both wheels. Both gears share identical tooth geometry, optimized for angular torque transfer at 45°, but differ in axle configuration to accommodate the offset differential housing.

The long version includes an extended cylindrical shaft that passes through the differential body, ensuring stable alignment and torque continuity. It connects to the wheel hinge through a separate diamond–diamond connector. The short version, positioned on the opposite side, integrates a built-in square diamond connector instead of a shaft. This allows a direct, compact connection to the hinge mechanism without additional adapters.

19.Differential Planetary Pinion — Core Gear of the Torque Balance System

This component is one of the key elements that allows the differential to operate correctly. It features the same bevel gear profile as the previously described sun gears but functions differently — it is directly mounted inside the differential housing. The pinion transfers rotational motion between the two sun gears, balancing torque when the robot turns or one wheel encounters resistance.

Attachment is achieved through a precision-fit screw connection. Because the gear rotates around its own pin under significant pressure, it is critical not to overtighten the screw — excess torque can introduce friction and hinder smooth operation. For this reason, a partially threaded 2.5–2.6 mm screw is used, ensuring only the tip engages while the smooth shaft portion allows free rotation.

This gear is essential for differential performance — without it, torque distribution between the robot’s wheels would be uneven, compromising control and stability during turns or terrain transitions.

20.Big Ball Bearing Frame — Main Bearing Mount and Structural Adapter

This component serves as the primary mount for the robot’s large ball bearing, acting as an adapter between the bearing and the chassis. It ensures precise alignment of the drivetrain and provides a rigid interface for torque transfer. Two identical units are used — one on each side of the main differential assembly — to hold the bearing securely in place and maintain parallel geometry across the structure.

Because of the compact spacing around this section, short M2.4 or M2.6 screws are recommended for fastening. The part’s geometry is optimized for both strength and precision, allowing the bearing to sit flush within the frame and ensuring that rotational loads are evenly distributed across the mount.

21. Hypoid Pinion Drive-to-Differential Connector

This gear connects the main axle to the differential, forming one half of the hypoid gear pair that transfers motor torque at an offset angle. The pinion ensures smooth rotation and precise contact between the two mating gears, maintaining consistent torque delivery across the drivetrain.

The geometry of this part was modeled and optimized manually using parameters generated in an online bevel/hypoid gear calculator. Each gear was then reconstructed and fine-tuned in Blender before being transferred to Fusion 360 for alignment and testing.

Because there is no automated generation process for this kind of geometry, each gear pair took between 20 to 40 minutes to build manually requiring multiple iterations to achieve the correct curvature and contact angle. This made it one of the most time-intensive and technically refined elements of the drivetrain.

22.Frame Forward — Servo and Front Assembly Mount

The forward frame serves as the main front mounting plate of the robot, connecting the servo motor, wheel hinge assemblies, and differential housing. It also provides the front attachment points for the Raspberry Pi, with two dedicated holes positioned near the rear edge of the plate.

A rectangular cutout in the center allows free rotation of the steering mechanism while minimizing weight and material use. The geometry is designed to maintain structural rigidity around the servo mounts while keeping clearance for gear motion.

23. Servo Pin — Compact Servo Retention PlateThe servo pin is a small mechanical clamp used to secure the servo motor in the frame forward assembly. It allows the servo to be held firmly in place using smaller screws than those typically provided in servo kits, improving weight efficiency and reducing strain on the printed mount.

Its dual-slotted design simplifies installation and removal during maintenance without fully disassembling the servo mount. Despite its minimal size, this part is crucial for maintaining alignment and preventing servo movement under dynamic load during wheel articulation.

24. Servo Arm — Dual-Link Steering Transmission Lever

The servo arm connects directly to the output shaft of the servo motor and performs two key steering functions. The short forward link attaches to the front hinge steering mechanism, ensuring precise directional control of the front wheels. The longer, reinforced rear link transfers rotational torque to the opposite side of the robot, synchronizing the left and right wheel articulation.

The arm’s central gear interface securely mates with the servo spline, preventing slippage during high-torque operation. Its asymmetrical geometry provides both mechanical leverage and clearance for other frame components, allowing smooth 180° rotation without interference.

25. Turn Axle Connector — Embedded Joint Interface

The Turn Axle Connector serves as the mechanical interface for attaching the metal rod (RS PRO 530-292) mentioned earlier. It is designed as a print-in-place component — meaning the 3D printer must be paused mid-process to insert a secondary element, such as the servo arm or the turn gear lift arm. Once the insert is positioned, printing resumes, encapsulating the part inside the structure and forming a functional hinge. To ensure proper assembly, the printing height must be carefully calibrated so that the inserted component sits precisely above the base layer without fusing to it. This design significantly increases reliability and alignment precision in the turning mechanism.

26. Turn Geared Liftarm — Direction Inversion and Torque Stabilization Link

The Turn Geared Liftarm functions as the rear counterpart to the servo arm, designed specifically to invert the steering direction between the front and back wheel systems. In the robot’s geometry, the front and rear steering mechanisms must rotate in opposite directions to maintain coordinated motion — this part makes that possible. It features a compact integrated gear section optimized for maximum surface contact, which improves torque transfer and minimizes tooth bending — a common issue in earlier iterations. The gear teeth were reshaped for smoother meshing and higher load tolerance. The arm interfaces directly with the metal rod ((RS PRO 530-292)) connector, translating its linear or rotational input into precise, synchronized motion on the opposite side of the chassis.

27. Turn Geared Liftarm Small — Reverse Gear Link for Steering Mechanism

This smaller version of the Turn Geared Liftarm is used to transfer and redirect rotational motion directly into the steering hinge assembly. Its gear orientation is intentionally reversed compared to the main liftarm, ensuring correct directional behavior when paired with the servo-driven linkage. A small cutout along the beam prevents interference with adjacent parts during motion, allowing the gear to operate freely within the tight mechanical space.

28. Turn Gear Pin — Retention and Stabilization Component

The Turn Gear Pin secures the Turn Geared Liftarm and Turn Geared Liftarm Small in place, preventing them from slipping out of alignment during operation. It also reinforces the axle connection, reducing bending and ensuring smooth gear engagement under torque. This pin acts as both a locking and structural element, helping distribute stress evenly across the turning mechanism. Without it, the servo-driven steering system would gradually lose precision due to gear flex and axial movement.

29. Turn Liftarm Set — Front, Back, and Universal Steering Connectors

The Turn Liftarm components form the mechanical link between the servo-driven rotation (via the Turn Geared Liftarm system) and the Wheel Hinge assemblies, translating servo motion into synchronized wheel steering. Each variant is designed to accommodate specific spatial and structural constraints within the chassis:

Turn Liftarm Front — Connects directly to the front steering hinges. Its longer pin provides extra strength to absorb sharp servo movements without deformation.

Turn Liftarm Back — Designed for the rear steering assembly where rotation is inverted. The pin is repositioned closer to the robot’s center to align with the reversed motion path.

Turn Liftarm Universal — A flexible connector variant that can operate on either side of the chassis. It includes a clearance cutout allowing it to fit below the differential housing and prevent collision during operation.

These 3 liftarms ensure consistent torque transmission and enable both front and rear steering systems to work in perfect synchronization.

30. Turn Liftarm Pin

Originally designed to secure the Turn Liftarm assemblies, this component now primarily serves a compensating and decorative role. After the introduction of the Turn Liftarm Universal, its structural importance was reduced — it now functions as a spacer that offsets screw length, ensuring proper tightening without excessive pressure on the surrounding parts.

31. Frame Back — Rear Structural Base and Motor Mount Interface

The rear frame serves as the primary structural support for the drive system. It houses the main DC motor, provides mounting points for both Turn Geared Liftarm mechanisms (standard and small versions), and includes a cutout for the differential assembly, mirroring the geometry of the main and front frames.Two extended arms at the top reinforce the structure and serve as mounting points for the Raspberry Pi, increasing rigidity and alignment accuracy. A dedicated slot in the center accommodates the motor mount, ensuring stable positioning and torque transfer to the drivetrain.

32. Motor Mount — Structural Interface for Power Transmission

The motor mount serves as a central connector between the main DC motor, the rear frame, and the main axle system. It features three mounting holes arranged in a triangular pattern for precise motor fixation and grooved lower supports that align with the rear frame. A large central opening allows the motor gear to pass through and engage with the drivetrain. Two lower extensions connect to the main frame, enabling the mount to slightly lift the rear assembly and isolate the motor from wheel load. A dedicated slot secures the motor gear and its pin, while a small lateral cutout provides access for a screwdriver during assembly. Beyond its technical role, the part also covers the front of the motor, hiding oil residue and improving the robot’s appearance. Functionally, it acts as the primary interface transmitting rotational power from the motor gear to the main axle, while maintaining alignment and structural rigidity.

33. Motor Gear — Main Power Transfer Gear

The motor gear directly transmits rotation from the main DC motor to the main axle. During printing, the process is paused to insert a ball bearing inside the gear — this print-in-place integration ensures smooth, high-speed operation without vibration or heat buildup. The inner geometry is precisely matched to the motor’s shaft, while the outer teeth are angled to engage with the hypoid gear on the main axle. This profile allows for efficient torque transfer even under high load, ensuring consistent alignment between the motor and drivetrain.

34. Motor Gear Pin

This small structural element secures the motor gear to the motor mount using a single precision screw.
Despite its size, it plays an essential role in maintaining alignment between the gear and motor shaft, preventing slippage during high-torque operation.

35. Motor Driver Module — Custom Expansion Bay Integration

The chassis of this robot includes modular side bays designed for adding custom hardware. On the left side, this space is used efficiently to house a combined control and power unit — the Motor Driver Module, which integrates the Cytron MD13S motor driver and the TPS61088 voltage booster (raising 5 V to 12 V). The module demonstrates how the expansion bays can be used for compact and flexible system integration: the booster is attached with strong double-sided tape, while the driver is fixed with screws and separated by a small internal wall for safety and stability. At the rear, a hexagonal connector pin links the module directly to the chassis structure between the Main Frame and Frame Back, ensuring both mechanical support and clean routing of power lines.

36. Cable Box —Cable Management Module

The right-side expansion bay is used for cable organization. The LiDAR unit came with a 1-meter cable, which was too long for the compact chassis layout. Instead of cutting or replacing it, the cable was carefully coiled and placed inside this cable box, then lightly secured with zip ties to prevent movement without applying excess pressure on the housing.

37. LiDAR Driver Module

This module holds the USB driver board for the RPLiDAR sensor, which comes included with the LiDAR unit. It is mounted between the Raspberry Pi and the top frame, where the mounting peg is slightly larger — making it impossible to attach from below without modifications. For simplicity and reliability, the driver board is secured using high-strength double-sided adhesive tape.

Part 4. PERFORMANCE

4.1 Open Challenge round

Youtube video

Strategy

The overall navigation strategy is based on sequential sensor integration that combines LiDAR-based positioning with camera-based environmental analysis.

At the initial stage, the LiDAR module determines the robot’s orientation and heading direction, providing a significant improvement compared to last year’s version, where the direction estimation relied primarily on probabilistic inference.

Once the approximate heading is established, the robot transitions to vision-driven navigation, as the LiDAR’s effectiveness is partially limited by the irregular and chaotic layout of walls and obstacles on the field.

The camera system therefore becomes the primary source of environmental data for pathfinding and object classification. However, the LiDAR remains essential for distance measurement and movement tracking—allowing the system to calculate displacement, progress, and positional updates with high precision.

This hybrid strategy ensures continuous localization even when visual input is obstructed or degraded, maintaining reliable navigation accuracy throughout the task cycle.

Source code

The key improvement implemented since last year’s model lies in the transition from camera-based trajectory recognition to multi-sensor positional tracking.

In the previous design, the robot relied primarily on visual line detection to estimate its progress along the course. This approach often led to instability — for instance, if the camera detected an unintended line or visual artifact, the navigation logic could misinterpret the path and trigger a failure in trajectory following.

The updated system eliminates this dependency by shifting progress estimation to non-visual sensors, primarily LiDAR and IMU-based odometry.

This allows the robot to continuously monitor its movement, orientation, and displacement without relying on camera feedback for positional awareness. As a result, the navigation remains stable, consistent, and less sensitive to unpredictable visual environments, such as varying lighting, reflections, or overlapping line patterns.

This change marks a significant step toward sensor redundancy and autonomous robustness, ensuring that the robot can sustain accurate navigation even under partial sensor degradation or environmental noise.

4.2 Obstacle challenge round.

Youtube video

Strategy

The robot’s navigation system is based on a multi-sensor integration framework, where each sensor performs a defined and complementary role.

The LiDAR module determines the robot’s orientation and spatial positioning on the map. Together with the camera, it forms the core of the perception system responsible for environmental analysis and obstacle avoidance.

The IMU (Inertial Measurement Unit) plays a supportive function, providing orientation stability and correction data in cases where LiDAR or visual inputs experience short-term uncertainty.

The robot continues to perform object avoidance using the camera, as in previous versions. However, unlike last year’s system, it now uses LiDAR for real-time localization — allowing it to memorize the spatial layout of detected obstacles and avoid collisions with walls or objects even when they are temporarily outside the camera’s field of view.

A crucial part of the strategy is the implementation of 180-degree steering wheels, which significantly enhance overall mobility and maneuverability. Each wheel serves as an independent pivot point, dramatically reducing the turning radius despite the robot’s relatively large chassis dimensions.

This configuration enables the robot to operate at higher speeds while maintaining precision control, allowing for sharper maneuvers and smoother navigation through constrained environments.

Source code

The navigation process begins with LiDAR-based orientation detection, which establishes the robot’s initial heading and spatial reference within the field. Once the primary direction is defined, the camera system takes over as the leading navigation source, guiding obstacle recognition and path correction.

Unlike the previous year’s model, the camera no longer determines position solely through visual memory. Instead, the LiDAR continuously tracks the robot’s exact position in real time, allowing the system to record the coordinates of detected objects and monitor mission progress with far greater accuracy.

This approach compensates for one of the camera’s main weaknesses — limited spatial awareness and susceptibility to visual occlusion.

In addition, the LiDAR maintains a map of surrounding walls, preventing potential collisions even when visual detection is temporarily obstructed.

The IMU (Inertial Measurement Unit) further enhances system precision by providing high-frequency updates on acceleration and angular velocity.

Its sampling rate is approximately five times faster than the camera’s frame rate and about fifty times faster than the LiDAR’s scanning cycle, which enables it to supply continuous motion feedback between sensor updates.

This complementary input allows the robot to interpolate missing data and maintain smooth trajectory estimation even during sensor latency or visual dropout.
