<img src="./readme images/1 logo.jpg" style="width:100%; height:auto;">

# TABLE OF CONTENTS

INTRODUCTION — THE CAR THAT THINKS IN ANGLES (page 4)

## PART 1 — TEAM INTRO (page 6-9)
- 1.1. Who We Are
- 1.2. Our Ideology
- 1.3. Team Members
- 1.4. Preparation to WRO 2025
- 1.5. Calendar

## PART 2 — ENGINEERING SOLUTION (page 10-15)
- 2.1. General Ideology – The Car That Thinks in Angles
- 2.2. Robot Photo and General Dimensions
- 2.3. System Integration Specification
  - 2.3.1. System Architecture Overview
  - 2.3.2. Integration Logic
  - 2.3.3. Mechanical Subsystem
  - 2.3.4. Electrical Subsystem
  - 2.3.5. Control Software
- 2.4. Innovative Abilities

## PART 3 — ROBOT CONSTRUCTION (page 18-53)
- 3.1. Building Robot — Necessary Materials and Instruments
- 3.2. Electrical Components — Core Power and Control Elements
- 3.3. Mechanical Elements
- 3.4. Core Structure Elements
  - 3.4.0. Introduction
  - 3.4.1. Frame System — Base and Structural Parts
  - 3.4.2. Axle and Bearing Assembly
  - 3.4.3. Differential Mechanism
  - 3.4.4. Steering and Hinge System
  - 3.4.5. Turn and Steering Transmission
  - 3.4.6. Wheel and Tire Assembly
  - 3.4.7. Auxiliary and Module Elements

## PART 4 — PERFORMANCE (page 53)
- 4.1. Open Challenge Round
  - 4.1.1. Program Logic Overview
  - 4.1.2. Program Flow Summary
  - 4.1.3. Software Module Reference Table
  - 4.1.4. Detailed Program Functions’ Description
- 4.1.5. Open Challenge Strategy and Tactics
  - 4.1.5.1. System Strategy — Autonomous Field Navigation
  - 4.1.5.2. Tactical Implementation — Coordinated Sensor Logic
- 4.2. Obstacle Challenge Round
  - 4.2.1. Strategy
  - 4.2.2. Program Flow Summary
  - 4.2.3. Software Module Reference Table
  - 4.2.4. Detailed Program Functions’ Description

## PART 5 — RESOURCES LIBRARY

# INTRODUCTION — THE CAR THAT THINKS IN ANGLES

This document presents the engineering concept and integration framework of the WRO Future Engineers 2025 project — **“The Car That Thinks in Angles.”**

Created by **Kyivrobomagic**, it reflects our practical engineering culture:  
precise where it must be, experimental where it can be.

The idea was simple — to design a robot that doesn’t just move,  
but **understands its movement**.

Each subsystem — from the semi-spherical drivetrain to the sensor fusion core —  
was built to *think in geometry*, not in lines.

Mechanical, electrical, and software layers are integrated into a single responsive platform,  
capable of autonomous navigation and spatial reasoning.

---

## **Key Design Priorities**

- **Adaptive geometry:** torque transfer through a semi-spherical gear system allowing full-angle articulation.  
- **Sensor fusion:** LiDAR, IMU, and camera coordination for stable perception under any conditions.  
- **Power efficiency:** UPS-based 18650 system with DC-DC boost for 12 V drive and 5 V control logic.  
- **Real-time control:** multi-threaded C++ architecture ensuring predictive, low-latency motion.  

---

Every Kyivrobomagic robot carries a small dose of what we call **“engineering magic”** —  
not mystery, but **elegant simplicity** that makes a complex system work naturally.

This year, that magic is **geometry itself**.

# PART 1: TEAM INTRO

## **Who We Are**

We are **Kyivrobomagic** — a robotics team born in **2018 in Kyiv, Ukraine**.

Our team has changed over time, but the heart of it has always been the same:  
**Petro Moroz** and **Mark Biryukov**.

Together, we’ve grown into one of the **most awarded robotics teams from Ukraine**.

<p align="center">
<img src="./readme images/2.png" style="width:100%; height:auto;">
</p>


## **Our Journey**

Our journey has been incredible.  
In **2020**, we earned **1st place** at **WRO Elementary**,  
in **2021** we took **silver**,  
then added another **silver at WRO Junior in Panama (2023)**,  
and reached the **Top 10 in Turkey (2024)** in the **Future Engineers** category.

But our story isn’t only about WRO.  
Each contest has taught us something new and made us stronger as a team.  
They brought fresh ideas, pushed us to think like real engineers, and inspired us to go further.

With every challenge, we learned not just how to build better robots,  
but also how to work together, solve problems creatively, and never give up.

For us, robotics is more than medals —  
it’s about **curiosity, friendship, and proving that Ukrainian kids can compete and win** on the world stage.

---

## **Our Ideology**

From the very beginning, our team has always appeared at competitions **dressed as wizards**.  
We are inspired by the famous quote:

> *“Any sufficiently advanced technology is indistinguishable from magic.”*

For us, it’s far more than words.  
At every competition, in every solution, we strive to create something so refined and well‑crafted  
that it amazes everyone and truly **feels like magic**.

We spend long hours working on programming and engineering,  
pouring not only knowledge but also imagination and creativity into our projects.

This is our philosophy:  
to transform complex technologies into something that inspires, fascinates, and opens new possibilities.  
**Technology as magic — and it is this magic that drives us forward.**

---

## **Team Members**

### **Petro Moroz — Engineering & Software**

In the team, Petro is responsible for engineering solutions, their implementation, and programming.  
For him, a robot is like a best friend, and every competition is a personal challenge.

He always strives to come up with something new —  
yet as simple and elegant as possible — to solve the tasks in the most efficient way.

Petro is also a talented coder — he likes to joke that he knows more programming languages than human ones.  
He proved his skills by taking **2nd place in Ukraine in programming among 9th graders**.

<p align="center">
<img src="./readme images/3.png" style="width:100%; height:auto;">
</p>

## **Mark Biryukov — Software & Team Coordination**

Mark is responsible for software development and team coordination.  
He can find common ground with anyone and is a talented **mathematician and programmer**,  
a multiple prizewinner of math and programming Olympiads.

His greatest skill is his **incredible speed in finding solutions**.  
Mark quickly tests and implements ideas, helping the team move forward fast.

Thanks to his speed, energy, and openness, the team easily builds connections,  
communicates with ease, and always keeps a fun and positive spirit.

He is the team’s **“social glue.”**

<p align="center">
<img src="./readme images/4.png" style="width:100%; height:auto;">
</p>

## **Kyrylo Moroz — Design & Identity**

Our story wouldn’t be complete without mentioning our biggest fan and friend —  
**Petro’s younger brother, Kyrylo Moroz**.

He is only **12 years old**, but he is the one responsible for our **logo**,  
team **identity**, and making sure that everything about our team  
looks **stylish**, **consistent**, and **creative**.

Thanks to Kyrylo, our work is not only about engineering and programming,  
but also about **design, presentation, and imagination**.

<p align="center">
<img src="./readme images/5.png" style="width:100%; height:auto;">
</p>

## **Preparation to WRO 2025**

### **Logistic challenge as the main issue**

Because of the war in Ukraine, we had to move our work fully online.  
**Mark is in Kyiv**, while **Petro is in Cherkasy**, where his grandparents live.

Another important point is that we work **without a coach**,  
since in Ukraine there are no trainers in this category.  
This makes our journey not only about robotics, but also about  
**self-education, self-organization, and discipline** —  
without which none of this would be possible.

The preparation itself is also a challenge because of logistics.  
We break it into stages:

---

## **1. Research & Information Gathering**

We already had experience in the **Future Engineers** category last year,  
which gave us a strong foundation.

Still, we spent a lot of time studying the latest solutions for autonomous systems.  
For example, our chassis design is based on a **patented engineering idea**  
that had never been manufactured before.

Turning it from an abstract idea into a working prototype  
made us the first to implement it — and that’s where the **“magic”** begins.

---

## **2. Invention & Prototyping**

Most of the robot development takes place in Petro’s **“home lab” in Cherkasy** —  
actually, his and his brother’s bedroom 🙂 — equipped with a **3D printer**  
and access to parts.

We are always connected online, exchanging ideas  
and guiding the next steps of technical development.

---

## **3. Programming & Testing**

At this stage, we begin traveling to meet each other on weekends.  
This means spending **about three hours on the road** back and forth  
just to work together in person, test programs, and fine‑tune the design.

---

## **4. Finalization (Mini‑Camp)**

We plan the final stage as a **one‑week mini‑camp**  
during the autumn school break in October.

This is when we meet, live, and work together,  
finalizing the solutions we’ve been building for months.

---

# **Calendar**

Structurally, our preparation calendar looked like this.  
Since we are now in our final school years,  
it was especially important to stay on schedule  
and avoid major overlaps with exams and studies — and we managed to do so successfully.

We divided our work into clear stages, planned around school and competitions,  
which allowed us to maintain discipline, balance learning and robotics,  
and stay fully prepared for every challenge.

<p align="center">
<img src="./readme images/6.png" style="width:100%; height:auto;">
</p>

# PART 2: ENGINEERING SOLUTION

## **General Ideology – The Car That Thinks in Angles**

### **1. From the Problem of Corners to the Geometry of Control**

Traditional vehicles are built for one-dimensional motion —  
they move forward and backward efficiently but lose precision when direction changes.

Most drivetrains are optimized for straight-line torque transfer,  
where every degree of deviation adds mechanical loss or instability.

This project started from a simple question:  
**can geometry itself improve control?**

Instead of treating steering angles as an obstacle,  
the robot uses them as part of its design logic.

The structure was built around the idea that **stability and control are not opposites** —  
they can be optimized together through geometry.

By rethinking how torque and direction interact,  
the robot achieves **smoother steering** and **higher mechanical accuracy**  
within a compact frame.

---

### **2. The Idea – The Car That Thinks in Angles**

The robot uses **one drive motor** and **one servo** to control all four wheels.  
Each wheel can rotate **up to 180°**, allowing:

- tight turning,  
- stable high-speed movement,  
- and minimal components without sacrificing precision.

The system relies on **mechanical synchronization**, not sensors on each wheel.  
The servo defines steering geometry,  
and the DC motor delivers torque.

Because all wheels are mechanically linked,  
motion remains consistent even at high rotation angles.

This design embodies **simplicity + reliability**:

- fewer moving parts,  
- fewer points of failure,  
- faster assembly,  
- easier debugging during competitions.

---

### **3. From Dimensions to Solutions**

Conventional robots operate in **two dimensions**: forward/backward and left/right.  
This system introduces a **third dimension — adaptive geometry**.

The robot dynamically adjusts orientation and turning angle  
based on real-time motion across the field.

To achieve this, it uses **three complementary sensors**:

- **LiDAR** — distance mapping + obstacle detection  
- **IMU** — stabilizes orientation between LiDAR updates  
- **Camera** — color, boundary, and line recognition  

Each sensor covers the weaknesses of the others.  
Together, they form a **robust and reliable perception system**  
even under variable lighting and surface conditions.

This setup is rare for WRO-level robots  
but proved **highly effective** in testing.

---

### **4. Key Technical Highlights**

#### **a) Unique Motion System**  
Each wheel rotates up to **180°**, yet the robot uses only **one motor + one servo**.  
This is extremely rare in competition robotics.

#### **b) Consistent Torque Delivery**  
The drivetrain maintains torque during steering through balanced geometry + load distribution.

#### **c) 3D-Printed Precision Components**  
Core parts — including advanced gear interfaces — were 3D-printed and tested.  
They demonstrate that **180° and even 360° torque transfer** is possible at this scale.

#### **d) Triple-Sensor Fusion**  
LiDAR + IMU + Camera → combined for superior navigation and robustness.

#### **e) Mechanical Reliability and Future Use**  
All components were validated under real competition conditions.  
They now form a **mechanical base** for future robot designs.

---

### **5. Summary**

**“The Car That Thinks in Angles”** is a compact and efficient robotic system  
that uses minimal components to achieve full control of movement.

The result is a **stable**, **fast**, and **reliable** platform  
that pushes the limits of what a single-motor robotic vehicle can do.

Every subsystem — drive, steering, sensing, structural design —  
was shaped through testing and iteration.

Its architecture blends:

- mechanical precision  
- geometric efficiency  
- sensor integration  

into one practical and elegant engineering solution.

---

# **Robot Photo and General Dimensions**

<p align="center">
<img src="./readme images/7.jpg" style="width:100%; height:auto;">
</p>

<p align="center">
<img src="./readme images/8.jpg" style="width:100%; height:auto;">
</p>

## **Robot Dimensions**
- **Length:** 24 cm  
- **Width:** 12 cm  
- **Height:** 10 cm  

---

# **SYSTEM INTEGRATION SPECIFICATION**

## **1. System Architecture Overview**

The robot integrates four independent drive wheel modules, each connected through a **semi-spherical bevel gear system** inspired by patent **US5129275A**.

All four wheels are driven by a **single JGB37-520 12 V DC motor** through two differential units — one on each side — linked by the **main transmission shaft**.

Steering of all wheel modules is performed simultaneously by a **single global servo (JX PDI-4409MG)** providing up to **180° articulation** per wheel.

Each differential transfers torque to two wheel modules installed on custom 3D‑printed housings, enabling free rotation under multiple steering angles with balanced torque.

A **Raspberry Pi** coordinates all subsystems, processes sensor data, and controls motor + servo through PWM and digital channels.

### **Core components**
- **Motor:** JGB37-520 DC Motor (12 V)  
- **Motor Driver:** Cytron MD13S (PWM ≤ 20 kHz)  
- **Servo:** JX PDI-4409MG (180° steering)  
- **Main Controller:** Raspberry Pi 4 Model B (8 GB)  
- **Power Source:** UPS HAT with 2 × 18650 Li-ion (7.4 V nominal)  
- **Voltage Regulation:** TPS61088 (5V → 12V boost)  
- **Sensors:** RPLIDAR S3, BNO085 IMU, Waveshare OV5647 Camera  
- **Chassis:** Fully custom 3D‑printed modular frame  

---

## **2. Integration Logic**

The **Raspberry Pi 4B** generates PWM for the Cytron MD13S and the steering servo.  
The motor driver regulates torque and speed; the servo defines steering geometry.

### **Power Distribution**
- UPS HAT powers control electronics (**5 V / 3.3 V**)  
- TPS61088 boosts **5 V → 12 V** for the DC motor  
- 2S Li-ion battery pack is the unified input  

### **Sensor Interfaces**
- **LiDAR:** USB (mapping + obstacle detection)  
- **IMU (BNO085):** I²C (orientation, stabilization)  
- **Camera:** CSI ribbon cable (vision processing)  

The Pi fuses all sensor data, computes navigation in C++, and sends real‑time motion commands.

---

## **3. Mechanical Subsystem**

A fully custom 3D‑printed semi-spherical bevel gear system distributes torque from the motor into two side differentials.

Each differential powers two wheel modules via compact coupling shafts.

This structure:
- preserves wheel orientation at any steering angle  
- minimizes friction  
- simplifies replacement of subassemblies  
- maintains alignment across the chassis  

---

## **4. Electrical Subsystem**

| Component | Function | Specification |
|----------|-----------|--------------|
| **Power Source** | Primary supply | 2 × 18650 Li-ion (7.4 V, 2S) |
| **Voltage Regulation** | Boost 5V → 12V | TPS61088, 30 W |
| **Power Distribution** | Logic rails | UPS HAT B |
| **Motor Driver** | Drive control | Cytron MD13S (13 A, 6–30 V) |
| **Motor** | Main torque | JGB37-520 DC Motor (12 V) |
| **Servo** | Steering | JX PDI-4409MG |
| **Sensors** | Perception | LiDAR S3, BNO085, OV5647 Camera |
| **Controller** | Navigation | Raspberry Pi 4B (8 GB) |

---

## **5. Control Software**

- **C++** → motor control, navigation logic  
- **Python** → IMU interface  
- **PWM** → drive + steering actuation  
- **Sensor fusion** → LiDAR + IMU + Camera  
- **Feedback loops** → heading + stability correction  

---

## **Summary**

The system integrates:
- single‑motor + global‑servo drivetrain  
- adaptive sensing  
- efficient power management  

Resulting in a compact, stable, full‑angle steering platform with reliable sensing and control.

---

## **Innovative Abilities**

| Ability | Description |
|--------|-------------|
| **Full‑Torque Steering** | Maintains torque during 180° wheel rotation |
| **Adaptive Response** | LiDAR + IMU + Camera fusion for stable navigation |
| **Efficient Power Control** | UPS HAT + TPS61088 ensure stable motor + logic power |

# PART 3: ROBOT CONSTRUCTION

## **Building Robot – Necessary Materials and Instruments**

## **1. 3D Printer: Ender V3 — Detailed Usage & Specifications**

<p align="left">
<img src="./readme images/9.jpg" style="width:25%; height:auto;">
</p>

This is the printer personally owned and used in my workshop for all prototype and final component manufacturing.  
It was used to fabricate the majority of the robot’s mechanical and structural parts.

Its **automatic bed leveling system** saves significant setup time —  
a major advantage, as manual leveling is often one of the most time‑consuming and error‑prone steps in 3D printing.

Using a **0.2 mm nozzle** enabled the production of small, detailed parts with:
- superior dimensional accuracy  
- smooth surface finish  
- tight bearing alignment  
- reliable tolerance for rotating assemblies  

These characteristics were crucial for the semi‑spherical gear interfaces, differential units, and steering components.

---

## **Ender V3 Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Type** | FDM (Fused Deposition Modeling) |
| **Build Volume** | 220 × 220 × 250 mm |
| **Recommended Filament Types** | PLA, PETG, TPU |
| **Filament Diameter** | 1.75 mm |
| **Nozzle Diameter** | 0.2 mm (standard, interchangeable) |
| **Nozzle Temperature Range** | up to 260 °C |
| **Bed Temperature Range** | up to 100 °C |
| **Layer Resolution** | 0.1 – 0.35 mm |
| **Print Speed** | up to 180 mm/s (recommended ≤ 120 mm/s for precision) |
| **Automatic Leveling** | CR Touch Sensor + strain gauge Z‑offset calibration |
| **Connectivity** | SD card, USB‑C |
| **Control Interface** | 4.3″ color display with Creality OS |
| **Frame** | All‑metal structure with dual Z‑axis & belt tensioners |
| **Supported File Formats** | STL, OBJ, AMF, G-code |
| **Slicing Software** | Creality Print, Cura, PrusaSlicer |
| **Power Supply** | 100–240 V AC (50/60 Hz) |
| **Noise Level** | < 45 dB (silent stepper drivers) |
| **Printer Dimensions** | 490 × 490 × 510 mm |
| **Weight** | ≈ 7.1 kg |

---

## **2. Printing Plastic**

<p align="center">
<img src="./readme images/10.jpg" style="width:100%; height:auto;">
</p>

The color itself is not critical, but it can slightly affect surface texture, print strength,  
and layer bonding. In my experience, **black PETG from CR** produced the most stable  
and consistent results on the Ender‑3 V3 printer.

The printer operates optimally at **230 °C nozzle temperature**,  
but users should run small calibration prints to tune performance  
for their specific filament brand and ambient environment.

Mechanical and aesthetic properties vary among manufacturers,  
so it's recommended to check **public filament tests** and **community benchmarks**  
before printing structural components such as gears, holders, and load‑bearing frames.

---

## **PETG Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Filament Type** | PETG (Polyethylene Terephthalate Glycol) |
| **Color** | Black / Blue |
| **Filament Diameter** | 1.75 mm |
| **Printing Temperature Range** | 230 – 250 °C |

---

# ELECTRICAL COMPONENTS — CORE POWER AND CONTROL ELEMENTS

## **1. Raspberry Pi 4 Model B – Central Control Unit**

<p align="left">
<img src="./readme images/11.jpg" style="width:75%; height:auto;">
</p>

This board serves as the **central control and coordination unit** of the robot.  
It manages all communication channels, motor coordination, and sensor data processing.

Thanks to its processing capacity, it can **simultaneously handle**:

- LiDAR data  
- IMU readings  
- Camera input  
- Motor control tasks  

The main navigation + control logic is written in **C++**, ensuring high-speed, low-latency execution.  
A lightweight **Python** script interfaces the **BNO085 IMU** library, simplifying data access and fusion.

An **UPS HAT (B)** module provides uninterrupted power, reducing development time, increasing reliability,  
and protecting the Raspberry Pi from power drops during motor load spikes.

The Raspberry Pi 4B was chosen for its:

- strong community support  
- abundant documentation  
- compatible peripherals  
- excellent performance for real-time robotics  

---

## **Raspberry Pi 4 Model B — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model** | Raspberry Pi 4 Model B |
| **RAM** | 8 GB LPDDR4 3200 |
| **Processor** | Broadcom BCM2711 — Quad-core Cortex A72 (ARM v8, 1.5 GHz) |
| **GPU** | VideoCore VI (OpenGL ES 3.x, Vulkan) |
| **Network / Connectivity** | Gigabit Ethernet, WiFi 2.4/5.0 GHz, Bluetooth 5.0, BLE |
| **USB Ports** | 2 × USB 3.0, 2 × USB 2.0 |
| **Display Outputs** | 2 × micro HDMI (dual displays up to 4K) |
| **Video / Audio** | H.265 4Kp60, H.264 1080p60, 4Kp30 encode/decode |
| **Storage Interface** | microSD slot |
| **Power Supply** | USB‑C 5.1 V / 3 A |
| **GPIO Header** | 40‑pin GPIO (Pi‑compatible) |
| **Dimensions** | 88 × 58 × 19.5 mm |
| **Operating Temperature** | 0°C to +50°C |
| **Operating System** | Raspberry Pi OS, Ubuntu, ARM Linux |
| **Typical Applications** | Robotics, IoT, education, desktop |
| **Manufacturer** | Raspberry Pi Ltd (UK) |
| **Official URL** | https://www.raspberrypi.com/products/raspberry-pi-4-model-b/ |

---

## **2. Sunon KD0501PFB3-8 Micro Fan – Essential Cooling Element**

Although small, this fan plays a **critical role in maintaining thermal stability** of the system.  
Without active cooling, the **Raspberry Pi 4 overheats within minutes**, reaching temperatures where the CPU throttles  
and can no longer operate reliably.

This miniature fan dramatically improves cooling performance — lowering the CPU temperature so the board stays:

- stable  
- responsive  
- touch-safe  
- fully operational under maximum load  

To reduce **noise and vibration**, a thin layer of **double‑sided adhesive tape** was placed between the fan and the 3D‑printed frame.  
This acts like soft **acoustic foam**, absorbing micro‑vibrations and preventing resonance with the chassis.

As a result, the fan becomes **almost silent** while maintaining full airflow efficiency.

---

## **Sunon KD0501PFB3-8 — Technical Specifications**

<p align="left">
<img src="./readme images/12.jpg" style="width:60%; height:auto;">
</p>

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Type** | DC Brushless Cooling Fan |
| **Manufacturer** | Sunon |
| **Model** | KD0501PFB3-8 |
| **Dimensions** | 20 × 20 × 10 mm |
| **Rated Voltage** | 5 V DC |
| **Operating Voltage Range** | 3.5 – 5.5 V |
| **Rated Current** | 0.06 A |
| **Rated Power Consumption** | 0.3 W |
| **Rated Speed** | 10,000 ±15% RPM |
| **Air Flow** | 1.0 CFM |
| **Static Pressure** | 3.6 mm H₂O |
| **Noise Level** | 22 dBA |
| **Bearing Type** | Vapo Bearing (MagLev technology) |
| **Connector Type** | 2-wire (Red +, Black –) |
| **Frame Material** | Thermoplastic (UL 94V-0) |
| **Weight** | ~4 g |
| **Operating Temperature Range** | –10 °C to +70 °C |
| **Expected Life** | 50,000 hours @ 40 °C |

---

## **3. RPLIDAR S3 – High-Precision 2D LiDAR Scanner**

<p align="left">
<img src="./readme images/13.jpg" style="width:70%; height:auto;">
</p>

This LiDAR sensor became a **turning point** in the robot’s spatial awareness system.

In the previous generation, the robot relied solely on the **camera for navigation**, meaning it had **no memory of its environment** — every run started “from zero.”  

With the introduction of the **RPLIDAR S3**, the robot can now:

- build a persistent **2D map** of its surroundings  
- perform **precise localization**  
- correct its path in real time  
- operate reliably regardless of lighting or color conditions  

Unlike the camera, the LiDAR is **immune to lighting changes**, shadows, reflections, and color inconsistencies.  
Earlier robots often confused:

- **red cubes** with **orange lines**,  
- **gray walls** with the **floor**,  
- or misread the field under shifting lighting conditions.

The RPLIDAR S3 eliminates these problems completely.

By combining **LiDAR mapping** with **IMU-based orientation correction**, the robot achieves:

- unprecedented stability  
- high navigation accuracy  
- reliable field awareness  

allowing the camera to focus **only on object detection**, not full scene interpretation.

---

## **RPLIDAR S3 — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model** | RPLIDAR S3 |
| **Type** | 2D 360° laser scanner (DTOF / SL TOF) |
| **Distance Range (70% refl.)** | 0.05–40 m |
| **Distance Range (10% refl.)** | 0.05–15 m |
| **Distance Range (2% refl.)** | 0.05–5 m |
| **Blind Range** | 0.05 m |
| **Sampling Rate** | Up to 32 kHz |
| **Scan Frequency** | 10–20 Hz |
| **Angular Resolution** | 0.1125°–0.225° |
| **Ranging Resolution** | 10 mm |
| **Range Accuracy** | ±30 mm |
| **Ambient Light Immunity** | ≥80,000 Lux (sunlight resistant) |
| **Communication Interface** | TTL UART (1 Mbit/s) |
| **Laser Safety** | Class 1 (eye safe) |
| **Power Supply** | 5 V DC, ~500 mA |
| **Dimensions** | 59.8 × 55.6 × 41.3 mm |
| **Weight** | ~115 g |
| **Operating Temperature** | –10 °C to +60 °C |
| **Applications** | Robotics, AGV/AMR, SLAM, UAV avoidance |
| **Manufacturer** | SLAMTEC (China) |
| **Official URL** | https://www.slamtec.com/en/S3 |

---

## **4. Waveshare OV5647 5 MP Wide-Angle Camera Module — Technical Description**

<p align="left">
<img src="./readme images/14.jpg" style="width:60%; height:auto;">
</p>

This camera module is the same model used in last year’s robot design.  
It is built on the **OV5647 5‑megapixel sensor** and operates natively with all **Raspberry Pi boards** via the CSI interface.

Despite being lightweight and simple, it is **highly optimized for Raspberry Pi**, providing stable and predictable behavior in robotics applications.

The camera captures **RGB images at up to 30 FPS (640×480)**.  
Although modest compared to modern high‑speed camera systems, this performance is **fully sufficient for fast robot movement** when paired with efficient image-processing algorithms.

### **Wide-Angle Advantage (160° FoV)**

A major benefit of this module is its **160‑degree wide-angle lens**, dramatically expanding the field of view.  
This allows the robot to:

- detect walls more reliably  
- track boundaries continuously  
- reduce blind spots  
- maintain stable navigation even in narrow or irregular field layouts  

### **Role in the New Architecture**

Unlike last year’s robot, **the camera no longer determines field position**.  
Localization is now handled entirely by the **LiDAR + IMU system**, resulting in:

- far higher positional stability  
- no more double-scanning of lines  
- reduced navigation errors  
- cleaner division of sensing roles  

The camera now focuses solely on:

- wall detection  
- color marker identification  
- boundary tracking  
- visual assistance in obstacle detection  

This separation significantly improves system accuracy and reduces computational load.

---

## **Waveshare OV5647 — Technical Specifications**

| **Parameter** | **Description / Value** |
|--------------|--------------------------|
| **Sensor Model** | OV5647 |
| **Image Sensor Type** | CCD, 1/4" |
| **Resolution** | 5 MP (2592×1944) |
| **Image Format** | RGB |
| **Supported Video Resolution** | Up to 1080p |
| **Max Frame Rate** | 30 FPS |
| **Recommended Mode** | 640×480 @ 30 FPS |
| **Field of View** | 160° wide-angle lens |
| **Standard FoV (base module)** | ~73° |
| **Connection Interface** | CSI |
| **Raspberry Pi Compatibility** | Pi 4B, 3B+, 3B, 2B, 1B, Zero series |
| **Zero Adapter Options** | 150 mm / 300 mm adapters |
| **Lens Characteristics** | Factory-mounted, Pi‑tuned optics |
| **CPU Load** | Minimal (hardware-accelerated pipeline) |
| **Primary Purpose** | Wide-angle real-time vision |
| **Form Factor** | Direct CSI‑port camera module |

---

## **5. BNO085 9‑DOF IMU Fusion Breakout (Adafruit 4754) – Inertial Compensation System**

<p align="left">
<img src="./readme images/15.jpg" style="width:60%; height:auto;">
</p>

While the RPLIDAR S3 provides spatial mapping, it operates at roughly **one measurement every 100 ms**—an eternity in control‑loop terms (about one data point per ~10 cm of travel).  

The **BNO085 IMU** fills this temporal gap by delivering **continuous inertial data**, maintaining motion awareness between LiDAR samples.

Although IMUs experience long‑term drift, their **short‑term accuracy** makes them ideal for interpolating robot motion between LiDAR frames, ensuring:

- smooth trajectory estimation  
- stable orientation  
- consistent turning logic  

### **Integration Challenge**

Integrating this IMU was one of the most challenging stages of the build:

- **SPI communication failed** due to unstable responses and noise.  
- Consultation with **Bryan Siepert** (BNO085 library author) confirmed SPI is unreliable in this configuration.  
- **I²C also failed initially**, not initializing consistently.  

The breakthrough came from using the **official Python driver**, which proved fully stable.  
A small Python service now runs in parallel with the C++ navigation engine:

- Python module → handles IMU fusion & quaternions  
- C++ program → receives results via internal data pipe  

This hybrid architecture merges the **reliability of Python** with the **performance of C++**.

---

## **BNO085 — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model** | BNO085 9-DOF IMU Fusion Breakout (Adafruit 4754) |
| **Function** | 3-axis accelerometer + 3-axis gyroscope + 3-axis magnetometer with fusion |
| **Processor** | ARM Cortex M0 running CEVA SH-2 firmware |
| **Sample Rate** | Up to 1 kHz |
| **Report Types** | Orientation, quaternion, lin. acceleration, gravity, raw/calibrated data, motion detection |
| **Interface** | I²C (default), optional UART/PSI |
| **Logic Voltage** | 3.3 V (5 V tolerant) |
| **Power Supply** | 3.3–5 V |
| **Dimensions** | 25.6 × 22.7 × 4.6 mm |
| **Weight** | ~2.5 g |
| **Mounting** | Breakout board + STEMMA QT/Qwiic |
| **Operating Temperature** | –40 °C to +85 °C |
| **Communication Speed** | I²C up to 400 kHz / UART 115,200 bps |
| **Applications** | Robotics, AR/VR, tracking, IMU fusion |
| **Manufacturer** | Adafruit / Hillcrest Labs |
| **Official URL** | https://www.adafruit.com/product/4754 |

---

## **6. Waveshare UPS HAT (B) – Uninterruptible Power Supply Module for Raspberry Pi**

<p align="left">
<img src="./readme images/16.jpg" style="width:60%; height:auto;">
</p>

The **Waveshare UPS HAT (B)** serves as the **primary power-stabilization unit** for the robot’s control system, ensuring continuous and stable operation of the Raspberry Pi 4 and all electronics even under dynamic motor loads.

In the previous year’s design, frequent **brown-out events** occurred — moments when sudden current spikes from motors or servos caused the Raspberry Pi to reboot.  
This UPS module **eliminates that problem entirely**:

- delivers **up to 5 A** continuous current  
- provides **stable 5 V and 3.3 V rails**  
- includes **real‑time battery monitoring** through I²C  
- **continues powering the Pi even while charging**  
- routes all power **through the UPS**, preventing undervoltage drops  

Although physically large and challenging to integrate mechanically, the reliability improvement is dramatic and essential for uninterrupted LiDAR, IMU, and camera processing.

---

## **Waveshare UPS HAT (B) — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model** | Waveshare UPS HAT (B) |
| **Output Voltage** | 5 V DC regulated |
| **Max Output Current** | Up to 5 A continuous |
| **Charger Input** | 8.4 V / 2 A |
| **Control Bus** | I²C — live monitoring (voltage, current, power, remaining capacity) |
| **Battery Support** | 2 × 18650 Li-ion cells |
| **Battery Protection** | Overcharge, overdischarge, overcurrent, short circuit, reverse polarity, balancing charge |
| **Automatic Switchover** | Yes — seamless transition to battery when power fails |
| **GPIO / Connector** | Pogo-pin connector (minimal GPIO usage) |
| **Compatibility** | Raspberry Pi 3 / 3B+ / 4B (+ likely Pi 5) |
| **Communication Interface** | I²C @ address 0x36 (BQ27441 or equivalent) |
| **Dimensions** | 56 × 85 mm |
| **Mounting Hole Size** | 3.0 mm |
| **Operating Temperature** | 0 °C – 50 °C |
| **Manufacturer** | Waveshare International |
| **Applications** | Robotics, Raspberry Pi UPS, industrial IoT |
| **Official URL** | https://www.waveshare.com/wiki/UPS_HAT_(B) |

---

## **7. DC‑DC Step‑Up (Boost) Module – TI TPS61088**

<p align="left">
<img src="./readme images/17.jpg" style="width:60%; height:auto;">
</p>

This module plays a **critical role in power conversion**, boosting the 5 V rail from the UPS HAT (B) to the **12 V required** by the Cytron MD13S motor driver and the robot’s DC gear motors.

The **TPS61088** chip is well known for its **exceptional current capability** — up to **10 A peak**, far exceeding typical hobby-grade converters (2.5–5 A).  
This higher margin ensures:

- stable motor torque  
- no voltage sag during acceleration or turning  
- clean, synchronized drivetrain behavior  

Despite its compact size, the module offers:

- precise output tuning via onboard potentiometer  
- optional configuration headers  
- high efficiency and robust protection  

In this robot, the converter forms the **power bridge** between:

- **5 V logic system** (UPS HAT output)  
- **12 V actuator rail** (motor + driver supply)

It was carefully calibrated for **stable 12 V** operation even under rapid motor load variations.

---

## **TI TPS61088 Boost Converter — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Module / IC** | DC-DC Step-Up (Boost) – TI TPS61088 |
| **Input Voltage** | 2.7–10 V DC |
| **Output Voltages** | 5 V / 9 V / 12 V selectable |
| **Max Output Power** | Up to 30 W |
| **Typical Output Capability** | 5 V → 5–6 A, 9 V → 3.3 A, 12 V → 2.5 A |
| **Efficiency** | Peak > 95% |
| **Switching Frequency** | ~200 kHz–2.2 MHz (typically ~1 MHz) |
| **Control Modes** | PFM/PWM auto or selectable |
| **Protection** | Overcurrent, cycle-by-cycle limiting, thermal shutdown |
| **Ripple / Noise** | Low with appropriate capacitors |
| **Inductor** | High‑current shielded (onboard) |
| **Operating Temperature** | –40 °C to +85 °C |
| **Dimensions** | Compact PCB module (varies by vendor) |
| **Reference Datasheet** | https://www.ti.com/lit/gpn/TPS61088 |

---

## **8. Cytron MD13S – 13A 6–30V DC Motor Driver**

<p align="left">
<img src="./readme images/18.jpg" style="width:60%; height:auto;">
</p>

The **Cytron MD13S** serves as the **main motor control unit** for each DC drive motor.  
It is a robust, competition‑grade driver designed for **high‑current, high‑stress** robotics applications.

Earlier prototypes used an L293D‑based driver, which failed during stress testing — specifically burning out during a collision when the motor produced a sudden reverse surge.  
The **MD13S handled the same tests without any damage**, thanks to:

- strong overcurrent protection  
- solid‑state H‑bridge design  
- high efficiency  
- 13 A continuous current capability  

Although physically larger than lighter alternatives, its **thermal stability**, **current margin**, and **plug‑and‑play simplicity** make it the ideal choice for the drivetrain.

Once installed, it requires almost no maintenance and performs flawlessly under continuous operation.

---

## **Cytron MD13S — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model** | Cytron MD13S 13A 6–30V DC Motor Driver |
| **Motor Channel** | 1 (Single brushed DC motor) |
| **Operating Voltage** | 6–30 V DC |
| **Continuous Current** | 13 A |
| **Peak Current** | 30 A (≤10 s) |
| **Logic Voltage** | 3.3 V / 5 V compatible |
| **PWM Frequency** | Up to 20 kHz |
| **Control Modes** | Sign‑Magnitude PWM / Locked‑Antiphase |
| **Direction Control** | DIR pin |
| **Connectors** | Terminal blocks (power & motor), GROVE logic connector |
| **Protection** | Over‑current limit, solid‑state H‑bridge, NMOS |
| **Efficiency** | ~95% |
| **Dimensions** | 61 × 33 mm |
| **Mounting Holes** | 4 × 3.2 mm |
| **Operating Temperature** | –10 °C to +60 °C |
| **Applications** | Robotics, automation, Arduino/RPi motor control |
| **Manufacturer** | Cytron Technologies |
| **Official URL** | https://www.cytron.io/p-13amp-6v-30v-dc-motor-driver |

---

## **9. JGB37-520 DC Gear Motor (12 V) – High-Power Drive Unit**

<p align="left">
<img src="./readme images/19.jpg" style="width:100%; height:auto;">
</p>

This motor serves as the **main propulsion unit** for the robot’s four-wheel drive system, providing both forward and reverse motion.

Compared to last year’s robot, the current platform is **larger, heavier, and significantly more power-demanding**.  
Extensive tests demonstrated that small 6–9 V motors were insufficient for:

- fast acceleration  
- high-speed traversal  
- overcoming friction in the semi-spherical gear system  

The **JGB37-520 (12 V)** motor fully meets these requirements.

Although the model includes a **planetary gearbox**, the robot is configured to operate with **minimal effective gear reduction**, maximizing shaft RPM to achieve extremely high linear acceleration.  
This “brute-force speed” approach provides:

- rapid movement  
- strong and stable torque  
- excellent response to motor commands  

It is one of the most reliable and high‑performance brushed DC motors available for medium-size robotics.

---

## **JGB37-520 DC Gear Motor — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model** | JGB37-520 DC Gear Motor (12 V) |
| **Rated Voltage** | 12 V DC |
| **Gearbox Type** | Metal planetary gearbox |
| **Gear Ratios (available)** | 1:20, 1:30, 1:60, 1:90 |
| **No Load Speed (12 V)** | 500 rpm (R20) / 320 rpm (R30) / 170 rpm (R60) / 110 rpm (R90) |
| **Rated Speed & Torque (R20)** | ~390 rpm @ 1 kg·cm |
| **Stall Torque (R60)** | ~9.2 kg·cm |
| **No Load Current** | ~0.25 A |
| **Stall Current** | 3.2–3.9 A |
| **Motor Type** | Brushed DC |
| **Shaft Type** | 6 mm D‑shaped |
| **Encoder Option** | Available (Hall sensor) |
| **Dimensions** | Ø37 mm × 65–80 mm |
| **Weight** | 200–230 g |
| **Operating Voltage Range** | 6–15 V DC |
| **Efficiency** | 60–75% |
| **Operating Temperature** | –10 °C to +60 °C |

---

## **10. JX PDI-4409MG Coreless Metal Gear Digital Low Profile Servo – Steering System Actuator**

<p align="left">
<img src="./readme images/20.jpg" style="width:60%; height:auto;">
</p>

This servo forms the **core of the steering system**, responsible for turning **all four wheels simultaneously** through the robot’s synchronized steering linkage.

While the JGB37-520 DC motors generate linear propulsion, the **JX PDI-4409MG** manages all directional control.  
Its **coreless motor** and **metal geartrain** provide:

- high torque  
- fast angle response  
- excellent precision  
- stability under vibration  

Multiple stress tests confirmed that this servo can sustain the mechanical load needed to steer all wheels equally, even during:

- high-speed turns  
- collisions  
- rapid angle changes  

The servo maintains position reliably without gear slippage or drift, making it ideal for real-time steering corrections.

---

## **JX PDI-4409MG — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model** | JX PDI-4409MG Coreless Metal Gear Digital Low Profile Servo |
| **Type** | Digital servo, coreless motor, metal gears |
| **Stall Torque @ 4.8 V** | ~7.8 kg·cm |
| **Stall Torque @ 6.0 V** | ~9.2 kg·cm |
| **Speed @ 4.8 V** | 0.13 s / 60° |
| **Speed @ 6.0 V** | 0.11 s / 60° |
| **Dead Band** | 2 µs |
| **Motor Type** | Coreless DC |
| **Gear Material** | Metal |
| **Bearings** | Dual ball bearings |
| **Operating Voltage** | 4.8–6.0 V DC |
| **Connector** | JR-type, ~265 mm |
| **Dimensions** | 40.5 × 20.3 × 28 mm |
| **Weight** | 44.5 g |
| **Case Material** | Aluminum mid-case |
| **Control Signal** | PWM (1520 µs neutral) |
| **Operating Angle** | ~120° mechanical |
| **Applications** | Robotics, RC cars, UAVs, actuators |
| **Manufacturer** | JX Servo Co., Ltd. |

---

## **MECHANICAL ELEMENTS**

### **1. Deep Groove Ball Bearing 684ZZ – Core Rotational Support Element**

<p align="left">
<img src="./readme images/21.jpg" style="width:60%; height:auto;">
</p>

These bearings form the **core mechanical foundation** of the robot’s motion system.  
Whenever a component rotates, pivots, or transmits torque, it is almost certainly supported by one of these bearings.

Their small size and high precision make them ideal for **compact, high-speed robotic assemblies**.

The 684ZZ bearings:

- minimize friction and vibration  
- improve drivetrain smoothness  
- increase motor efficiency  
- stabilize wheel and differential modules  
- support high‑speed rotation without deformation  

They are especially important in the robot’s **semi-spherical drivetrain**, where tight tolerances and low resistance are essential.

---

## **684ZZ Deep Groove Ball Bearing — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Type** | Deep Groove Ball Bearing 684ZZ |
| **Inner Diameter (ID)** | 4 mm |
| **Outer Diameter (OD)** | 9 mm |
| **Width** | 4 mm |
| **Shield Type** | Double Metal Shield (ZZ) |
| **Material** | Chrome Steel (GCr15) |
| **Lubrication** | Pre-lubricated (high-speed grease) |
| **Dynamic Load Rating (Cr)** | ~0.68 kN |
| **Static Load Rating (Cor)** | ~0.25 kN |

---

## **2. Radial Deep-Groove Ball Bearing CX 61700 ZZ – Differential Support Bearing**

<p align="left">
<img src="./readme images/22.jpg" style="width:100%; height:auto;">
</p>

This larger bearing was introduced after the robot’s chassis and drivetrain grew significantly compared to last year's model.  
Many earlier mechanical solutions could no longer support the **increased torque and structural loads**, especially inside the differential assembly.

In previous versions, the differential suffered from:

- instability  
- lateral play (“wobbling”)  
- misalignment under load  

because the smaller bearings were not able to secure the shaft firmly.

The **CX 61700 ZZ** solved these issues by providing:

- rigid radial alignment  
- minimal lateral movement  
- stable shaft support  
- precise low-friction rotation  

Due to **tight spatial constraints** — with the differential body and central axle sharing the same structural axis — this bearing’s **thin 4 mm profile** made it the ideal choice.  
It provided the necessary rigidity without increasing chassis width or interfering with transmission geometry.

---

## **CX 61700 ZZ Bearing — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model** | CX 61700 ZZ |
| **Type** | Radial deep-groove ball bearing, single-row |
| **Inner Diameter (d)** | 10 mm |
| **Outer Diameter (D)** | 15 mm |
| **Width (B)** | 4 mm |
| **Shield Type** | ZZ — double metal shield |
| **Material** | Chrome Steel (GCr15) |
| **Mass** | ~0.002 kg |
| **Lubrication** | Pre-lubricated (high-speed grease) |

---

## **3. Metric Mounting Screw Set – M1, M2, M2.5 (Precision Fasteners Kit)**

<p align="left">
<img src="./readme images/23.jpg" style="width:60%; height:auto;">
</p>

This small hardware kit became an **unexpected turning point** in the robot’s mechanical assembly.  
Originally purchased simply to replace missing M2.5 screws for the Raspberry Pi, it revealed an entire **new design scale** — ultra‑small metric fasteners suitable for compact, high‑precision 3D‑printed assemblies.

What began as a coincidence quickly became a **design breakthrough**:

- Tiny M1–M2 screws enabled structural joints previously impossible with glue or snap‑fits  
- Allowed extremely compact mounts for sensors, linkages, and micro‑mechanisms  
- Greatly improved serviceability and component modularity  
- Gave the robot a more **professional, machine‑grade build quality**  

These miniature screws became one of the highest **cost‑to‑performance upgrades** in the entire project: inexpensive, widely available, yet transformative in what could be mechanically built at small scale.

---

## **Metric Screw Set — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Type** | Metric screw assortment — M1, M2, M2.5 |
| **Material** | Carbon steel |
| **Head Style / Drive Type** | Phillips cross‑head |
| **Surface Finish** | Nickel‑plated |

---

## **4. Threaded Rod RS PRO 530‑292 – Structural Torque Transmission Element**

<p align="left">
<img src="./readme images/24.jpg" style="width:60%; height:auto;">
</p>

The **RS PRO 530-292 threaded rod** serves as a rigid mechanical backbone for transmitting force and rotation across the robot’s frame.

Early prototypes suffered from **chassis flex** — even thick 3D‑printed beams would bend slightly under load.  
This caused:

- distortion in steering geometry  
- inaccurate servo feedback  
- unpredictable wheel angles  
- inconsistent turning precision  

Replacing flexible plastic connectors with a **precision-cut steel threaded rod** solved the issue completely.

This rod:

- links the servo output to the opposite-side steering assembly  
- ensures **pure torque transfer with zero angular error**  
- screws directly into 3D‑printed components  
- requires no adhesives or extra fittings  
- remains perfectly straight even under high stress  

This is critical for maintaining **precise steering alignment**, especially at high speeds.

Beyond its primary function, it opened new design possibilities — acting as a miniature structural rail for future metal‑reinforced 3D‑printed chassis concepts.

---

## **RS PRO 530-292 Threaded Rod — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Model / Stock Number** | RS PRO 530-292 |
| **Product Type** | Threaded Rod |
| **Thread Size** | M3 |
| **Thread Type** | Metric, fully threaded |
| **Material** | Steel (Grade 4.6 mild steel) |
| **Surface Finish** | Zinc plated (BZP) |

---

## **5. Tire 24 × 12 Low-Profile – LEGO Part 18977**

<p align="left">
<img src="./readme images/25.jpg" style="width:60%; height:auto;">
</p>

This tire was selected because its **small radius reduces the physical envelope** of the steering assembly.  
Since each wheel in the drivetrain can rotate **up to 180°**, any increase in tire radius would significantly enlarge the clearance required for full steering articulation.

The **24 × 12 low-profile** tire offers:

- a compact wheel size  
- a wide, stable contact patch  
- excellent traction on smooth indoor surfaces  
- minimal deformation during rapid direction changes  

The uniform tread and rounded shoulders provide **predictable lateral grip**, essential for high‑precision steering corrections.  
Despite being LEGO components, their **dimensional accuracy and material consistency** make them well suited for competitive robotics.

---

## **LEGO Tire 18977 — Technical Specifications**

| **Parameter** | **Specification** |
|--------------|-------------------|
| **Part Number** | 18977 |
| **Name** | Tire 24 × 12 Low Profile |
| **Material** | LEGO rubber compound (TPE-like) |
| **Color** | Black |
| **Mounting** | Press-fit on compatible LEGO rims |
| **Years Produced** | 2015 – present |
| **Notes** | Low-profile, wide footprint, excellent grip |

---

## **6. Cable System — Power and Signal Distribution**

The robot’s cabling system is designed for **stable power delivery** and **reliable data transmission** across all modules while keeping the layout clean and modular.

The main wire used is a **0.2 mm Arduino cable**, chosen for flexibility and compatibility with the Raspberry Pi GPIO.

### **Connection Types Used in the Robot**

1. **Arduino cable (main line):**  
   - primary wiring backbone  
   - connects directly to GPIO  
   - branches to secondary modules  

2. **Grove Cytron connector (4-pin, 2 mm pitch):**  
   - exclusive connection for the Cytron MD13S driver  

3. **Standard 2-pin Grove connector:**  
   - dedicated to powering the TPS61088 DC-DC boost converter  

---

## **Power Distribution Strategy**

Because the UPS HAT powers the entire system through a **single interface**, custom‑soldered branches were needed.

### **5 V Rail (GPIO pins 2 and 9)** — split into two lines:

- **Line A:** Arduino cable → Servo motor  
- **Line B:** 2‑pin Grove connector → TPS61088 boost converter  

### **Ground (GND)** — split into **three** lines:

- **GND A:** shared with 5 V servo line  
- **GND B:** for the TPS61088 module  
- **GND C:** for the 4‑pin Cytron Grove connector  

PWM and DIR pins for the Cytron driver are connected to the Raspberry Pi GPIO through **standard Arduino wiring**, ensuring stable signal levels and compatibility.

---

## **Other System Connections**

- **Camera (OV5647):** connected directly via CSI ribbon cable  
- **RPLIDAR S3:** connected via USB adapter (requires two USB ports: power + data)  
- **All remaining wiring:** standard Arduino conventions for easy modularity  

This custom system ensures:

- stable shared power from the UPS HAT  
- clean wiring layout  
- modular upgrades  
- sufficient current delivery across all components  
- prevention of overload on any single power line  

---
# PART 3: ROBOT CONSTRUCTION — Core Structure Elements

## **Introduction**

This section provides a detailed overview of every major **3D‑printed and structural component** used in the robot’s construction.  
The documentation follows a logical progression — from the **base frame**, drivetrain components, hinge and steering systems, to **wheel assemblies** and **auxiliary mounting modules**.

Each element includes:
- its **function**  
- its **relationship** with other components  
- key **engineering** or **assembly details**  

---

# **1. Frame System — Base and Structural Parts**

## **1.1. Frame Top — Sensor and Electronics Mounting Plate**

<p align="left">
<img src="./readme images/26.jpg" style="width:100%; height:auto;">
</p>

**Function:**  
The Frame Top serves as the primary **mounting platform** for all major electronics, including the Raspberry Pi, LiDAR interface board, camera module routing, and auxiliary sensor units.  
It is the uppermost structural layer of the chassis.

**Engineering Notes:**  
- Designed with **dedicated screw points** (M2 / M2.5) for secure mounting.  
- Integrates **cable routing channels** to prevent wire interference with the drivetrain.  
- Reinforced perimeter walls provide rigidity without unnecessary weight.  
- The geometry ensures efficient heat dissipation from the Raspberry Pi and UPS HAT.  
- Alignment tabs ensure perfect fit with the Main Frame.

**Component Role:**  
Forms the stable platform that ties together the sensing layer of the robot, ensuring that IMU, camera, Raspberry Pi, and LiDAR connections remain rigid and vibration‑free.

---

## **1.2. Main Frame — Structural Base of the Robot**

<p align="left">
<img src="./readme images/27.jpg" style="width:100%; height:auto;">
</p>

**Function:**  
The Main Frame is the **central chassis** of the robot — the structural foundation that distributes mechanical load, supports torque transfer, and integrates all major drivetrain and steering subsystems.

**Engineering Description:**  
- 3D‑printed from **PETG** for optimal stiffness and vibration resistance.  
- Designed as a **single-piece structural body** to maximize rigidity while keeping weight low.  
- Features reinforced ribs forming a load-bearing grid beneath the drivetrain.  
- Houses mounting channels for the differential, steering hinge frames, and motor supports.  
- Provides precise alignment for the **main axle**, bearing housings, and semi‑spherical gear system.  
- Includes symmetrical geometry to maintain accurate centerline alignment — essential for LiDAR mapping stability and IMU compensation.  

**Component Role:**  
The Main Frame serves as the **mechanical backbone** of the robot, connecting the front and rear sections, transferring torque between drivetrain modules, and ensuring that steering and differential components remain perfectly aligned.

---


The **main frame** is the primary structural platform that supports all mechanical and electronic subsystems.  
It provides mounting points for the wheel assemblies, differential mechanisms, the main drive motor, and the hinge connectors for the central axle.

### **Engineering Description**
- Symmetrical layout allowing identical wheel modules at the **front and rear**.  
- Paired mounting holes at both ends ensure interchangeable assembly.  
- Cylindrical cutouts and opposing slots secure the **differential housings**.  
- Distributed micro‑connectors along the central spine support hinge and axle elements.  
- Two hexagonal cutouts function as attachment points for the rear Raspberry Pi assembly.  

### **Structural Characteristics**
- 3D‑printed with **variable thickness**:
  - **3 mm** on the main surface to improve interior clearance for motor and battery.
  - **6 mm** on reinforcement beams for additional torsional stiffness.
- Slight wheel‑axis offset ensures **full 180° wheel rotation** without contacting the frame.
- Defines the robot’s **overall geometry**, ensuring:
  - precise drivetrain alignment  
  - stable torque distribution  
  - high structural rigidity  

---

## **2. Diamond–Diamond Axle Connector**

<p align="left">
<img src="./readme images/28.jpg" style="width:60%; height:auto;">
</p>


This component serves as a **universal coupling element** for the robot’s 4 mm axles — the standard diameter used across the drivetrain due to its excellent balance of compactness and mechanical strength.

### **Engineering Description**
- Cylindrical connector body with a **square central socket** for anti-slip engagement.  
- Two perpendicular **set‑screw holes** allow firm tightening onto both shafts.  
- Ensures stable transfer of torque between:
  - the main axle  
  - the steering linkage  
  - the differential assembly  

### **Functional Role**
The connector is used to **join two separate axle segments**, enabling:
- modular assembly  
- easy maintenance  
- adjustable shaft lengths  
- secure torque transmission  

Although some other printed parts integrate similar male/female axle interfaces, this standalone connector adds **flexibility and precision** during prototyping and fine‑tuning of the drivetrain.

---

## **3. Ball Bearing Frame — Differential Support Mount**

<p align="left">
<img src="./readme images/29.jpg" style="width:60%; height:auto;">
</p>

This component functions as a **structural adapter** for mounting a standard ball bearing onto the main frame near the differential assembly.  
It ensures **stable fixation of the main axle**, which passes directly through the bearing, allowing smooth and perfectly aligned rotation under load.

### **Engineering Description**
- Includes two **lateral mounting holes**, compatible with the robot’s standard metric screw set.  
- Designed for precise positioning of the bearing relative to the **differential gears**.  
- Distributes perpendicular forces generated during torque splitting, preventing axle bending or misalignment.  
- Provides a rigid stabilization point that locks the main axle’s position during high-speed movement.

### **Functional Role**
The Ball Bearing Frame is essential for drivetrain stability.  
The differential generates significant **lateral and vertical forces**, and this mount ensures the axle remains:

- rigid  
- centered  
- vibration-free  
- aligned with the drivetrain geometry  

Without this frame, the differential assembly would experience wobble, decreasing efficiency and increasing gear wear.

---

## **4. Ball Bearing Frame with Tube — Axle Protection and Support Mount**

<p align="left">
<img src="./readme images/30.jpg" style="width:60%; height:auto;">
</p>

This enhanced version of the standard ball bearing frame incorporates a **cylindrical tube holder** designed to support a lightweight protective tube surrounding the main axle.  
The tube acts simultaneously as:

- a **mechanical guard**, preventing accidental contact between the rotating axle and sensitive components (e.g., battery pack), and  
- a **stabilizing sleeve**, reducing lateral oscillation during high‑speed movement.

### **Engineering Description**
- Designed to hold a thin protective tube in perfect alignment with the **bearing axis**.  
- Maintains the axle’s central position between the differential and the primary bearing mount.  
- Prevents the axle from drifting outward under torque or impacts.  
- Printed with reinforced walls to withstand continuous dynamic loading.  
- Secured to the main frame using **two standard metric screw holes**, identical to the mounting pattern of the standard bearing frame.

### **Functional Role**
Without this component, the main axle would lack one of its critical lateral support points, increasing the risk of:

- misalignment  
- rotation-induced vibration  
- drivetrain inefficiency  
- full detachment under sudden torque spikes  

This frame ensures that the axle remains **perfectly centered, protected, and stable**, even under high mechanical load.

---

## **5. Main Axle and Support Structure**


<p align="left">
<img src="./readme images/31.jpg" style="width:100%; height:auto;">
</p>

The **main axle** is the central mechanical element responsible for transmitting rotational torque from the motor to both differentials.  
It spans the entire length of the robot, connecting the **motor-side drive gear** to the **opposite differential assembly**, forming the backbone of the drivetrain.

### **Engineering Description**
This part integrates multiple functional elements into a **single 3D‑printed structure**:

- A rigid torque-transfer shaft  
- An embedded bearing frame  
- A protective stabilization tube  
- Internal supports printed in-place  

Because of its complexity, the axle is printed **vertically**, which requires:

- a **controlled print pause** to insert a ball bearing near the motor interface  
- specialized support towers to prevent shaft deformation  
- precise alignment to ensure smooth rotation under load  

These internal supports are visible in the model and help maintain **perfect axial geometry**, especially around the gear teeth and bearing interface.

### **Functional Role**
The final printed axle is:

- rigid  
- lightweight  
- vibration-resistant  
- capable of transferring torque with high precision  

It ensures stable operation of both differentials and preserves drivetrain alignment even under long-term mechanical stress.

---

## **6. Hinge Frame — Robot Side**

<p align="left">
<img src="./readme images/32.jpg" style="width:60%; height:auto;">
</p>

This component forms one half of the wheel rotation mechanism and attaches directly to the main frame of the robot.  
It includes **four mounting contact points** for structural fixation and a **central bearing housing** to ensure precise rotational alignment of the hinge assembly.  
Two **cylindrical pins** extend outward from the frame to connect with the opposite hinge half, creating a stable pivot joint.

Adjacent to the pins are **two integrated gears** that synchronize the movement of both hinge sides, ensuring coordinated rotation during steering.  
The component provides both **mechanical support** and **transmission alignment**, allowing the wheel assembly to articulate smoothly while maintaining consistent torque transfer through the hinge system.

---

## **7. Hinge Frame — Wheel Side**

<p align="left">
<img src="./readme images/33.jpg" style="width:60%; height:auto;">
</p>

This component is a **compact counterpart** to the robot-side hinge frame and is engineered to minimize the total length of the steering mechanism while preserving full rotation clearance for the wheel module.

### **Engineering Description**
- Contains a **partial bearing housing**, designed so that **one half of the ball bearing seats in this frame**, while the remaining half is enclosed by the Robot-Side hinge.  
  This reduces assembly thickness and ensures tight axial alignment.
- The hinge geometry is shortened to create space for **full 180° wheel rotation**.
- Two **integrated gears** are positioned with a **one‑tooth offset** relative to the robot-side hinge frame.  
  This prevents interference and ensures smooth meshing during synchronized articulation.
- The structural body includes precise mounting contours that connect directly to the wheel module.

### **Functional Role**
The Wheel-Side Hinge Frame provides:

- **Accurate mechanical coupling** between the wheel and the chassis  
- **Stable rotational alignment** through the shared bearing system  
- **Synchronized steering movement** via the paired gear system  
- A compact form that maintains maneuverability without sacrificing structural rigidity  

This part is essential for delivering consistent steering response and for enabling the robot’s highly compact wheel rotation system.

---

## **8. Hinge Frame Cover**

<p align="left">
<img src="./readme images/34.jpg" style="width:60%; height:auto;">
</p>

This component attaches to the **wheel-side hinge frame** using four screws and secures the **outer half of the ball bearing** within the hinge assembly.  
Its primary function is to **lock the bearing in place**, preventing any axial movement or loosening during high‑speed wheel rotation.

### **Engineering Description**
- Fastens directly to the Wheel-Side hinge via **four mounting screws**.  
- Encloses the second half of the bearing, completing the hinge’s rotational chamber.  
- Maintains **precise axial alignment**, ensuring the bearing remains centered under load.  
- Adds **structural rigidity** to the hinge assembly by bracing its outer perimeter.  
- Designed with minimal thickness to reduce bulk while retaining high-strength support.

### **Functional Role**
The Hinge Frame Cover is essential for:

- Securing the bearing against outward displacement  
- Preserving smooth, consistent hinge rotation  
- Preventing vibration or misalignment during steering  
- Increasing durability of the entire wheel‑rotation subsystem  

By closing the hinge from the external side, it finalizes the complete bearing enclosure and ensures long-term stability during robot operation.

---

## **9. Hinge Connector and Hinge Arm (Left & Right)**

<p align="left">
<img src="./readme images/35.jpg" style="width:100%; height:auto;">
</p>

These components link the two halves of the hinge assembly from both sides, ensuring structural stability, correct spacing, and synchronized steering motion.

### **Engineering Description**

- The **Hinge Connector** forms the primary mechanical bridge between the Robot-Side and Wheel-Side hinge frames.  
  It prevents separation of the hinge halves and maintains the correct axial spacing needed for smooth rotation.

- The **Hinge Arms (Left & Right)** are modified extensions of the connector design.  
  Each arm includes **additional mounting points** used to attach the steering linkage that transfers motion from the servo system.

- The arms connect directly to the **Wheel-Side hinge frame**, enabling full **180° wheel rotation** without interference.  
- The L/R mirrored versions ensure perfect mechanical symmetry and balanced steering behavior across the robot.

### **Functional Role**

Together, these parts:

- Complete the mechanical hinge loop  
- Maintain rigid structural coupling between hinge halves  
- Provide mounting interfaces for steering linkages  
- Ensure synchronized, precise steering geometry across both sides  
- Allow smooth articulation of each wheel module under dynamic loads  

This tri-component system is essential for reliable turn mechanics, symmetry, and structural durability.

---

## **10. Hinge Cap and Hinge Cap (Gear)**

<p align="left">
<img src="./readme images/36.jpg" style="width:100%; height:auto;">
</p>
These components secure the hinge connectors in place, preventing them from slipping off the alignment pins and ensuring long-term mechanical reliability of the hinge assembly.

### **Engineering Description**

- The **standard Hinge Cap** mounts directly to the hinge frame using screws.  
  Its purpose is to lock the hinge connector on the pivot pins, preventing axial displacement during wheel articulation.

- The **Hinge Cap (Gear)** is a reinforced variation featuring an **integrated tooth profile**.  
  This profile provides optional mechanical engagement and additional resistance to lateral forces — especially beneficial when wheels experience side impacts or vibration during high‑speed operation.

### **Assembly Considerations**

Proper torque must be applied during installation:

- **Over‑tightening** increases rotational friction, restricting hinge movement and causing steering lag.  
- **Under‑tightening** allows unwanted play in the hinge, reducing steering accuracy and increasing wear.

Achieving the correct torque balance is essential for maintaining:

- Smooth, low‑friction rotation  
- Stable wheel alignment  
- Consistent steering response  
- Long-term durability of the hinge system  

---

## **11. Sun Gear Long — Wheel Rotation and Torque Transfer Element**

<p align="left">
<img src="./readme images/37.jpg" style="width:60%; height:auto;">
</p>

This component makes possible the full **180° steering rotation** of the wheel using a patented **semi-spherical gear mechanism**.  
Its geometry is optimized for both strength and smooth operation, enabling reliable power transmission through the steering axis.

### **Engineering Description**
- Features **reinforced gear teeth** shaped to maintain stable engagement even at extreme articulation angles.  
- The **central rounded hub** increases rigidity and prevents deformation under torque load.  
- An **extended shaft** allows deep insertion into the wheel housing, creating a strong and stable mechanical interface.  
- Designed to align precisely with the hinge structure and differential elements for uninterrupted motion.

### **Development & Iteration**
This gear underwent several experimental revisions to achieve:

- Smooth engagement at wide articulation angles  
- High resistance to bending loads  
- Self-supporting internal strength  
- Perfect compatibility with the semi-spherical drivetrain geometry  

### **Functional Role**
The Sun Gear Long is essential for:

- Transmitting rotational torque from the drivetrain into the wheel module  
- Maintaining alignment within the hinge assembly  
- Enabling articulated motion without power loss  
- Ensuring smooth, predictable wheel rotation during steering  

Its robust, high-precision form makes it one of the key elements of the robot’s semi-spherical steering and torque-transfer mechanism.

---

## **12. Sun Gear Short — Modular Torque Transmission Node**

<p align="left">
<img src="./readme images/38.jpg" style="width:60%; height:auto;">
</p>

The Sun Gear Short operates on the same mechanical principle as the **Sun Gear Long**, but features a **significantly shortened shaft** tailored for modular integration within the drivetrain system.

### **Engineering Description**
- Designed to interface directly with the **Diamond–Diamond Axle Connector**, enabling torque transmission between interchangeable drivetrain modules.  
- The shortened shaft minimizes spatial footprint without reducing alignment accuracy or structural strength.  
- Maintains a **rounded central hub**, ensuring rigidity under load and preventing deformation during high‑torque operation.  
- Compact geometry allows placement in areas where full-length gears cannot fit, while still ensuring proper engagement with the semi-spherical gear system.

### **Functional Role**
The Sun Gear Short is optimized for:

- **Modular drivetrain configurations**  
- **Quick replacement** during tuning or maintenance  
- **Precise torque transfer** through compact assembly zones  
- **Alignment stability** in multi-element gear networks  

Its efficient design ensures that the drivetrain remains flexible, serviceable, and mechanically robust even in confined structural regions.

---

## **13. Wheel Assembly — LEGO-Compatible Low-Profile Drive Interface**


<p align="left">
<img src="./readme images/39.jpg" style="width:100%; height:auto;">
</p>

This wheel serves as the primary mechanical interface between the robot’s drivetrain and the LEGO tire system, providing a **secure, precise, and modular** connection fully compatible with LEGO low-profile tires.

### **Engineering Description**
- The wheel hub includes a **central removable ring insert**, designed to:
  - reduce overall print height  
  - minimize support material usage  
  - improve circular dimensional accuracy  
  - strengthen the contact zone between the axle and the tire  

- The ring insert is printed separately to avoid distortion in tall cylindrical features, ensuring **high tolerances** where the tire attaches.

- The outer diameter strictly follows the **LEGO 24 × 12 Low Profile Tire standard**, enabling a perfect press-fit attachment.

- The wheel geometry is tuned to maintain the exact spacing required for:
  - torque transmission  
  - steering articulation  
  - clearance between the tire and structural frame  

### **Functional Role**
This wheel assembly ensures:

- **Reliable torque transfer** from the axle to the tire  
- **Stable alignment** during high-speed rotation  
- **Modular compatibility** with LEGO components  
- **Quick replacement** during testing or repairs  

Because of its hybrid printed–LEGO design, it combines precise engineering with the flexibility of modular robotics.

---

## **14. Hinge Support — Bearing Retention and Structural Reinforcement**

<p align="left">
<img src="./readme images/40.jpg" style="width:100%; height:auto;">
</p>

The hinge support components are designed to **securely hold the ball bearing** inside the Robot-Side hinge frame.  
Their primary function is to ensure **accurate bearing alignment** and prevent any axial displacement during wheel rotation.

### **Engineering Description**
- Provides a rigid mounting enclosure for the ball bearing, allowing smooth and stable hinge articulation.  
- Eliminates the need for complex “print‑in‑place” bearing traps, simplifying assembly and improving reliability.  
- Transfers structural loads from the steering mechanism into the chassis without deformation.  
- Strengthens the hinge’s side walls, preventing flex under torque.

### **Open Variation**
An alternative “open” version includes a **cutout** that grants screwdriver access to an internal screw deep inside the hinge assembly.  
This design:

- improves serviceability  
- enables fast disassembly  
- avoids removing the entire hinge module  

### **Functional Role**
The hinge support ensures:

- secure bearing retention  
- rigid steering geometry  
- reduced vibration during articulation  
- precise axle alignment across the hinge assembly  

It is an essential reinforcement element that maintains long‑term reliability of the robot’s steering system.

---

## **15. Differential — Torque Distribution and Height Optimization**

<p align="left">
<img src="./readme images/41.jpg" style="width:60%; height:auto;">
</p>

The differential is built around a single hypoid gear, enabling the main drive axle to sit **4 mm lower** than the wheel axis — a structural decision that significantly reduces the overall height of the robot. This compact design contributes to a lower center of gravity and improved stability during motion.

The differential casing is laterally offset due to the geometry of the hypoid gear, but this shift does not interfere with performance. Beneath the gear sits a connector mount for a **large ball bearing**, providing stability under rotational load.

Functionally, the differential plays a crucial role in torque distribution — it ensures both wheels receive appropriate power during turns. This is essential because each wheel follows its own path radius, and without differential control, one wheel would slip or drag.  
The combination of precision gearing, ball bearing support, and structural alignment allows this differential to deliver **smooth, balanced power transfer** across the drivetrain.

---

## 17. Differential Pin — Secondary Bearing Coupler

<p align="left">
<img src="./readme images/42.jpg" style="width:60%; height:auto;">
</p>

The Differential Pin is the companion component to the main differential housing, serving as the **rear coupling interface** that connects the differential assembly to the large ball bearing. It ensures that the rotational load from the hypoid gear is evenly distributed and transmitted to the support structure, maintaining stability under torque stress.

Its circular flange with **three mounting holes** allows secure attachment to the differential body while aligning precisely with the bearing mount.  
The pin’s centered bore provides a tight fit for the axle shaft, ensuring **smooth, low-friction rotation**.

Though small, this part is essential for maintaining mechanical precision in the robot’s drivetrain — without it, the entire differential system would lose its axial support and alignment.

---

## 18. Differential Sun Gear (Long and Short) — Angular Torque Transfer Pair

<p align="left">
<img src="./readme images/43.jpg" style="width:60%; height:auto;">
</p>

The differential sun gears form the active core of the hypoid transmission system, responsible for translating motor torque into balanced rotational output for both wheels. Both gears share identical tooth geometry, optimized for angular torque transfer at **45°**, but differ in axle configuration to accommodate the offset differential housing.

### Long Version
The long sun gear features an **extended cylindrical shaft** that passes through the differential body, ensuring stable alignment and continuous torque transfer.  
It connects to the wheel hinge via a **diamond–diamond connector**, allowing secure coupling between the drivetrain and steering assembly.

### Short Version
The short sun gear includes a **built‑in square diamond connector** instead of a shaft.  
This compact interface allows direct attachment to the hinge mechanism without additional adapters, minimizing space usage and increasing structural rigidity.

Together, these two gears form a synchronized torque‑distribution pair, enabling smooth angular power transfer across the differential assembly.

---

## 19. Differential Planetary Pinion — Core Gear of the Torque Balance System

<p align="left">
<img src="./readme images/44.jpg" style="width:60%; height:auto;">
</p>

This component is one of the key elements that allows the differential to function correctly.  
It features the same bevel gear profile as the previously described sun gears but serves a different mechanical role — it is **mounted inside the differential housing** and acts as the central balancing element between the two sun gears.

The planetary pinion transfers rotational motion between the sun gears, ensuring **balanced torque distribution** when the robot turns or when one wheel experiences resistance.

### Attachment & Mechanical Behavior
- Mounted via a **precision-fit screw connection**.  
- The gear rotates around its own pin and is subjected to significant lateral pressure.  
- A **partially threaded 2.5–2.6 mm screw** is used:  
  - Only the tip engages the thread.  
  - The smooth shaft portion allows free, low-friction rotation.  
- Over-tightening must be avoided, as excess pressure increases friction and restricts movement.

### Functional Importance
This gear is essential to proper differential operation.  
Without the planetary pinion:

- torque distribution between wheels would be uneven  
- turning stability would be compromised  
- drivetrain smoothness would degrade  
- wheel-slip during curves or terrain transitions would increase  

It is the core element that enables true differential action.

---

## 20. Big Ball Bearing Frame — Main Bearing Mount and Structural Adapter

<p align="left">
<img src="./readme images/45.jpg" style="width:60%; height:auto;">
</p>

This component serves as the primary mount for the robot’s **large ball bearing**, acting as a structural adapter between the bearing and the chassis.  
It ensures **precise drivetrain alignment** and provides a rigid interface for torque transfer.

Two identical units are used — positioned on both sides of the main differential — to secure the bearing and maintain **perfectly parallel geometry** across the drivetrain structure.

### Engineering Description
- Designed to hold the large bearing **flush inside the frame**, preventing axial or lateral shifting.  
- Ensures that rotational forces from the differential are evenly distributed across the mount.  
- Compact spacing in this section requires **short M2.4 or M2.6 screws** for fastening.  
- Geometry is optimized for both **strength and dimensional precision**, ensuring long-term mechanical stability.

---

## 21. Hypoid Pinion — Drive-to-Differential Connector

<p align="left">
<img src="./readme images/46.jpg" style="width:60%; height:auto;">
</p>

This gear connects the main axle to the differential, forming one half of the **hypoid gear pair** that transfers motor torque at an offset angle.  
The pinion ensures **smooth rotation** and **precise tooth contact** with its mating gear, delivering consistent torque across the drivetrain.

### Engineering Description
- Forms the angled drive interface between the motor-driven axle and the differential input.  
- Maintains stable contact geometry with the hypoid sun gear under dynamic load.  
- Geometry was **manually modeled and tuned**, since no automated generation tool exists for this exact configuration.  
- Parametric values were first produced using an online bevel/hypoid gear calculator, then rebuilt in **Blender**, and finally aligned and tested in **Fusion 360**.

### Development Effort
Each gear pair required:
- **20–40 minutes** of manual modeling  
- Multiple curvature and contact-angle iterations  
- Fine adjustments to tooth shape, root clearance, and surface alignment  

This made the hypoid pinion one of the **most technically refined and time‑intensive elements** in the entire drivetrain.

---

## 22. Frame Forward — Servo and Front Assembly Mount

<p align="left">
<img src="./readme images/47.jpg" style="width:60%; height:auto;">
</p>

The forward frame serves as the main **front mounting plate** of the robot, connecting the **servo motor**, the **wheel hinge assemblies**, and the **differential housing**. It also provides the **front attachment points for the Raspberry Pi**, with two dedicated holes positioned near the rear edge of the plate.

A **rectangular center cutout** allows free rotation of the driving mechanism while reducing overall weight and material usage.  
The geometry is reinforced around the servo mount area to maintain structural rigidity, while still providing adequate clearance for gear movement.

---

## 23. Servo Pin — Compact Servo Retention Plate

<p align="left">
<img src="./readme images/48.jpg" style="width:60%; height:auto;">
</p>

The servo pin is a small mechanical clamp used to secure the servo motor within the Frame Forward assembly.  
It allows the servo to be held firmly using **smaller, lightweight screws** instead of the large hardware typically included in servo kits — reducing overall mass and mechanical strain on the printed frame.

### Engineering Description
- Features a **dual‑slotted design**, enabling fast installation or removal of the servo without needing to disassemble the entire front mount.  
- Provides firm lateral compression around the servo tabs, preventing micro‑shifts during high‑load steering movements.  
- Designed to distribute clamping pressure evenly, protecting both the printed frame and the servo housing from stress or cracking.  

### Functional Role
Despite its compact size, the servo pin is essential for:
- Maintaining precise servo alignment  
- Preventing rotational or lateral motion during wheel articulation  
- Ensuring consistent steering geometry under dynamic loads  

Its minimalist structure makes it both **lightweight and highly reliable**, optimized for frequent assembly cycles during testing and tuning.

---

## 24. Servo Arm — Dual-Link Steering Transmission Lever

<p align="left">
<img src="./readme images/49.jpg" style="width:100%; height:auto;">
</p>
The servo arm connects directly to the output shaft of the steering servo and performs **two critical steering functions**:

1. The **short forward link** connects to the front hinge steering mechanism, enabling precise directional control of the front wheels.  
2. The **longer reinforced rear link** transfers rotational torque across the robot to the opposite side, synchronizing left–right wheel articulation.

### Engineering Description
- Features a **central gear interface** designed to securely mate with the servo spline, eliminating slippage during high‑torque operation.  
- Constructed with an **asymmetrical geometry** that provides mechanical leverage while avoiding interference with nearby frame components.  
- Allows smooth **180° steering rotation** without collisions or structural obstruction.  
- Reinforced sections ensure rigidity during rapid articulation and high‑force maneuvers.

### Functional Role
The servo arm ensures:
- Synchronized wheel steering across the entire robot  
- Stable, predictable motion transfer  
- High precision in angle control  
- Reliable performance under dynamic load  

Its dual‑link configuration makes it one of the most important elements of the steering transmission system.

---

## 25. Turn Axle Connector — Embedded Joint Interface

<p align="left">
<img src="./readme images/50.jpg" style="width:60%; height:auto;">
</p>

The Turn Axle Connector serves as the mechanical interface for attaching the metal rod (RS PRO 530-292).  
It is designed as a **print‑in‑place component**, meaning the 3D printer must pause mid‑process to insert a secondary mechanical element — such as the servo arm or the turn geared liftarm.

Once the insert is placed, printing resumes, encapsulating the element inside the connector and forming a **functional rotational hinge**.

### Engineering Description
- Designed to hold the **M3 metal rod** securely for torque transmission.  
- Relies on a controlled print pause to embed the secondary component at the correct layer height.  
- The insert must sit **exactly above the active layer** to avoid fusing while still being locked into place by subsequent layers.  
- Print‑in‑place geometry guarantees perfect alignment of the hinge and eliminates the need for screws or external mounts.  
- Greatly increases long‑term durability and precision of the turning mechanism.

### Functional Role
The Turn Axle Connector provides:
- A rigid and precise hinge connection  
- Accurate alignment for steering joints  
- High reliability due to fused‑structure architecture  
- Reduced mechanical play during steering motion  

This connector is one of the most assembly‑sensitive parts of the robot, requiring exact print timing and correct insert placement.

---

## 26. Turn Geared Liftarm — Direction Inversion and Torque Stabilization Link

<p align="left">
<img src="./readme images/51.jpg" style="width:60%; height:auto;">
</p>

The Turn Geared Liftarm serves as the rear counterpart to the servo arm and is specifically engineered to **invert the steering direction** between the front and rear wheel systems.  
In this robot’s geometry, the front and back steering assemblies must rotate in **opposite directions** to maintain coordinated turning.  
This liftarm enables that inversion.

### Engineering Description
- Features a **compact integrated gear section** engineered for maximum surface engagement.  
- Improved tooth profile increases load tolerance and reduces bending — addressing weaknesses found in earlier iterations.  
- Provides smooth meshing with minimal friction, even under high steering loads.  
- Interfaces directly with the **metal rod (RS PRO 530-292)**, converting its rotational or linear motion into synchronized articulation for the opposite hinge assembly.  
- Reinforced structure ensures rigidity during fast directional changes.

### Functional Role
The Turn Geared Liftarm ensures:
- Accurate **direction inversion** between front and rear steering  
- Stable torque transfer across the chassis  
- Consistent and synchronized wheel articulation  
- High durability during repeated high-speed steering cycles  

This component is critical for achieving symmetric, coordinated steering across the robot’s drivetrain.

---

## 27. Turn Geared Liftarm Small — Reverse Gear Link for Steering Mechanism

<p align="left">
<img src="./readme images/52.jpg" style="width:60%; height:auto;">
</p>
This smaller version of the Turn Geared Liftarm transfers and redirects rotational motion directly into the steering hinge assembly.  
Its gear orientation is intentionally **reversed** relative to the main liftarm, ensuring correct steering direction when paired with the servo‑driven linkage.

### Engineering Description
- Compact gear profile designed for tight mechanical spaces.  
- Reverse tooth orientation ensures correct motion phase relative to the primary liftarm.  
- Includes a **cutout along the beam** to prevent interference with neighboring components during articulation.  
- Interfaces cleanly with the hinge system to deliver precise steering adjustments.  
- Optimized to maintain smooth gear meshing even under dynamic steering loads.

### Functional Role
This component provides:
- Direction-corrective gear engagement  
- Smooth rotational transfer to the hinge joint  
- Low-friction operation within limited space  
- Structural stability for the steering mechanism  

Its compactness and reversed gear geometry make it a crucial link in achieving coordinated steering motion across the robot.

---

## 28. Turn Gear Pin — Retention and Stabilization Component

<p align="left">
<img src="./readme images/53.jpg" style="width:60%; height:auto;">
</p>

The Turn Gear Pin secures the **Turn Geared Liftarm** and **Turn Geared Liftarm Small** in place, preventing them from slipping out of alignment during operation.  
It reinforces the axle connection, reducing bending and ensuring **smooth, consistent gear engagement** under torque.

### Engineering Description
- Acts as a **locking pin** that prevents axial movement of the liftarms.  
- Strengthens the turning mechanism by stabilizing the rotational joint.  
- Ensures proper alignment of the gear teeth throughout the steering cycle.  
- Helps distribute mechanical stress evenly across the turning assembly.  
- Prevents long-term degradation of steering precision caused by gear flex.

### Functional Role
Without the Turn Gear Pin:
- The liftarms would drift out of alignment  
- Steering accuracy would degrade over time  
- Gear meshing would become inconsistent  
- Axial play would increase, causing jitter and torque loss  

The pin is a small but essential structural element in the servo-driven steering system.

---

## 29. Turn Liftarm Set — Front, Back, and Universal Steering Connectors

<p align="left">
<img src="./readme images/54.jpg" style="width:100%; height:auto;">
</p>

The Turn Liftarm components form the mechanical link between the **servo-driven rotation** (via the Turn Geared Liftarm system) and the **Wheel Hinge assemblies**, translating servo motion into synchronized wheel steering.

Each variant is designed to handle specific geometric and structural constraints inside the chassis:

### • Turn Liftarm Front
- Connects directly to the **front steering hinges**.  
- Features a **longer, reinforced pin** to absorb sharp servo movements without deformation.  
- Provides optimal leverage for the front steering geometry.

### • Turn Liftarm Back
- Designed for the **rear steering assembly**, where rotation is **inverted**.  
- Pin position is shifted closer to the chassis center to align with the reversed kinematic path.  
- Ensures symmetry and synchronized response with the front steering system.

### • Turn Liftarm Universal
- A flexible connector variant compatible with **either side** of the chassis.  
- Includes a dedicated **clearance cutout** allowing it to fit under the differential housing.  
- Prevents collisions during extreme articulation angles.

### Functional Role
Together, the three liftarms ensure:

- Consistent torque transmission  
- Perfect synchronization of front and rear steering  
- Smooth, precise articulation during high-speed maneuvers  
- Compatibility with compact chassis geometry  

---

## 30. Turn Liftarm Pin

<p align="left">
<img src="./readme images/55.jpg" style="width:60%; height:auto;">
</p>

Originally designed to secure the Turn Liftarm assemblies, this component now primarily serves a **compensating and decorative role** within the turning mechanism.  
With the introduction of the **Turn Liftarm Universal**, its structural importance decreased.

### Engineering Description
- Functions as a **spacer**, ensuring that screws tighten correctly without applying excessive pressure to nearby printed components.  
- Maintains clean mechanical spacing between the Turn Liftarm structures.  
- Prevents screw over‑tightening, which could deform surrounding parts or restrict motion.  
- Provides minor stabilization but no longer carries significant load.

### Functional Role
The Turn Liftarm Pin now ensures:
- Proper screw tension  
- Safe clearance between rotating components  
- A clean, finished fit for the turn assembly  

While no longer essential for structural strength, it contributes to the reliability and longevity of the assembly.

---

## 31. Frame Back — Rear Structural Base and Motor Mount Interface

<p align="left">
<img src="./readme images/56.jpg" style="width:100%; height:auto;">
</p>

The rear frame serves as the primary structural support for the **drive system**.  
It houses the **main DC motor**, provides mounting points for both the **Turn Geared Liftarm mechanisms** (standard and small variants), and includes a dedicated cutout for the **differential assembly**, mirroring the geometry of the main and forward frame structures.

### Engineering Description
- Contains a **central slot** designed to securely hold the motor mount, ensuring proper torque transfer into the drivetrain.  
- Two **extended upper arms** reinforce the rear structure and act as mounting points for the Raspberry Pi, improving overall rigidity and alignment accuracy.  
- Side mounting points accommodate both types of Turn Geared Liftarms, enabling smooth steering synchronization between the front and rear assemblies.  
- Rear geometry mirrors the forward and main frames, ensuring consistent spacing and mechanical compatibility across the chassis.

### Functional Role
The Frame Back provides:
- Secure housing for the DC motor  
- Structural reinforcement for the drivetrain  
- Consistent alignment across the robot’s frame  
- Reliable mounting for steering and differential components  

It is one of the central load‑bearing elements in the robot’s architecture.

---

## 32. Motor Mount — Structural Interface for Power Transmission

<p align="left">
<img src="./readme images/57.jpg" style="width:60%; height:auto;">
</p>

The motor mount serves as the central connector between the **main DC motor**, the **rear frame**, and the **main axle system**.  
It is engineered to ensure precise alignment, stable torque transmission, and reliable support for the drivetrain during high‑load operation.

### Engineering Description
- Features **three mounting holes** arranged in a triangular pattern to secure the motor firmly and prevent rotational slip.  
- Includes **grooved lower supports** that align perfectly with the Rear Frame, locking the motor mount into its correct structural position.  
- A large **central opening** allows the motor gear to pass through and engage the drivetrain without obstruction.  
- Two lower extensions interface with the Main Frame, slightly lifting the rear assembly to isolate the motor from wheel load and reduce vibration transfer.  
- A **dedicated internal slot** stabilizes the motor gear and its pin.  
- A **lateral cutout** provides screwdriver access for easy assembly and maintenance.  
- The mount also covers the front of the motor, concealing oil residue and improving overall appearance.

### Functional Role
The motor mount:
- Provides rigid and accurate positioning of the main DC motor  
- Ensures proper meshing between the motor gear and the drivetrain  
- Transmits torque from the motor to the main axle  
- Reduces stress on the motor by isolating wheel and frame loads  
- Enhances structural rigidity and visual cleanliness of the robot  

It is one of the key structural and functional elements ensuring efficient and stable drivetrain operation.

---

## 33. Motor Gear — Main Power Transfer Gear

<p align="left">
<img src="./readme images/58.jpg" style="width:60%; height:auto;">
</p>

The motor gear directly transmits rotation from the **main DC motor** to the **main axle**.  
During printing, the process is **paused to insert a ball bearing** inside the gear — a print‑in‑place technique that ensures smooth, high‑speed rotation with minimal vibration and reduced heat buildup.

### Engineering Description
- The **inner bore** is precisely matched to the motor shaft for a tight, slip‑free fit.  
- The **outer gear teeth** are angled to engage the hypoid gear on the main axle with maximum contact efficiency.  
- Designed for **efficient torque transfer** even at high rotational loads.  
- Print‑in‑place bearing integration improves:
  - rotational stability  
  - gear longevity  
  - thermal performance  
  - resistance to wobble at speed  

### Functional Role
This motor gear:
- Transmits all motor torque into the drivetrain  
- Maintains perfect alignment with the hypoid system  
- Ensures quiet, vibration‑free operation  
- Enables smooth acceleration and rapid directional transitions  

It is one of the most mechanically demanding printed components in the drivetrain.

---

## 34. Motor Gear Pin

<p align="left">
<img src="./readme images/59.jpg" style="width:60%; height:auto;">
</p>

This small structural element secures the **motor gear** to the **motor mount** using a single precision screw.  
Despite its minimal size, it plays a critical role in maintaining perfect alignment between the gear and the motor shaft, preventing slippage during **high‑torque operation**.

Its precise geometry ensures that the motor gear remains locked in its correct axial position, guaranteeing stable and reliable torque transfer throughout the drivetrain.

---

## 35. Motor Driver Module — Custom Expansion Bay Integration

<p align="left">
<img src="./readme images/60.jpg" style="width:100%; height:auto;">
</p>

The chassis of this robot includes modular side bays designed for adding custom hardware. On the left side, this space is used efficiently to house a combined control and power unit — the **Motor Driver Module**, which integrates the **Cytron MD13S motor driver** and the **TPS61088 voltage booster** (raising 5 V to 12 V).

This module demonstrates how the expansion bays can be used for compact and flexible system integration:
- The booster is attached with strong double-sided tape.
- The driver is mounted with screws.
- A small internal wall separates both components for added safety and stability.

At the rear, a **hexagonal connector pin** links the entire module directly to the chassis structure between the **Main Frame** and **Frame Back**, ensuring mechanical stability and clean routing of power lines.

---

## 36. Cable Box — Cable Management Module

<p align="left">
<img src="./readme images/61.jpg" style="width:60%; height:auto;">
</p>

The right-side expansion bay is used for cable organization.  
The LiDAR unit came with a **1-meter cable**, which was too long for the compact chassis layout.  
Instead of cutting or replacing it, the cable was **carefully coiled** and placed inside this cable box, then lightly secured with **zip ties** to prevent movement without applying excess pressure on the housing.

This solution preserves cable integrity, maintains a clean internal layout, and avoids stress on connectors during operation.

---

## 37. LiDAR Driver Module

<p align="left">
<img src="./readme images/62.jpg" style="width:60%; height:auto;">
</p>

This module holds the USB driver board for the RPLiDAR sensor, which comes included with the LiDAR unit.  
It is mounted between the Raspberry Pi and the top frame, where the mounting peg is slightly larger — making it impossible to attach from below without modifications.

For simplicity and reliability, the driver board is secured using **high-strength double-sided adhesive tape**.

# PART 4. PERFORMANCE

## 4.1 OPEN CHALLENGE ROUND

**PROGRAM LOGIC OVERVIEW**  
This section describes the software architecture that powers the robot’s autonomous behavior.  
The program is designed as a modular, multi-threaded system that integrates real-time data from LiDAR, camera, and IMU sensors, processes it efficiently, and converts it into stable motion control signals for the drivetrain and steering system.  

At a high level, the robot operates as a closed-loop system — continuously perceiving, analyzing, and acting:

1. **Perception Layer** – The robot gathers data from all sensors in parallel:  
   o LiDAR provides 2D environmental scans used to detect obstacles and walls.  
   o The camera supplies visual input for field recognition and visual correction.  
   o The IMU measures orientation and motion stability.

2. **Processing Layer** – Each data stream is handled in an independent thread:  
   o `scanThread()` updates LiDAR data through the **LidarStore** buffer.  
   o `cameraThread()` manages camera input through the **CameraStore**.  
   o Helper functions (`process_raw_frame`, `process_hsv`, `grayWorldWB`) normalize and filter images for geometric analysis.  
   o LiDAR data is preprocessed (`updateLidarData`, `combineLines`) to identify field edges and corners.

3. **Decision Layer** – Once the perception data is prepared, the robot interprets its position relative to the field geometry:  
   o `angle_between_lines`, `pointLinePerpendicularDistance`, and `lineIntersection` compute geometric relations.  
   o `nextCorner()` determines the next turn or checkpoint on the course based on these relationships.  
   o The robot keeps track of how many corners it has passed to evaluate mission progress.

4. **Action Layer** – Motor and steering commands are generated and executed:  
   o `motor()` controls the propulsion motor via PWM through the Cytron driver.  
   o `servo()` adjusts the steering angle with precision to match target headings.  
   o `fill()` and visualization utilities help debug and monitor the system’s understanding of its environment.

5. **Support and Control Infrastructure** –  
   The control logic is synchronized through lightweight shared-memory mechanisms:  
   o **LidarStore** and **CameraStore** manage real-time double-buffered data flow.  
   o `setup_GPIO()` initializes and configures GPIO pin modes.  
   o Diagnostic functions (`printDeviceInfo`, `checkHealth`) ensure hardware integrity before mission start.  
   o Power, logic, and sensor cycles are coordinated for stable operation at high frame rates.

# PROGRAM FLOW SUMMARY

## Initialization Phase

* The system boots, runs `setup_GPIO()`, and initializes LiDAR, camera, and IMU interfaces.
* Health checks are performed via `checkHealth()` and `printDeviceInfo()`.

## Runtime Phase

* Threads start: `scanThread()` (LiDAR) and `cameraThread()` (camera) run continuously.
* Buffers (`lidarStore`, `cameraStore`) update asynchronously, ensuring minimal latency.
* The main control loop periodically retrieves latest data, processes it, and computes the next steering and motor actions.

## Navigation Phase

* Image and LiDAR data are combined to estimate robot position relative to walls and corners.
* Using geometric computations (`angle between lines`, `nextCorner`, etc.), the robot plans turns and adjusts trajectory in real time.
* Every completed corner updates mission progress and stability checks.

## Safety and Reset

* If sensor data becomes unstable or incomplete, the system pauses and clears relevant buffers using `eraseLidarData()`.
* When safe, scanning and control threads resume normal operation.

## System Characteristics

* **Parallel execution** ensures fast response under changing field conditions.
* **Shared-memory synchronization** allows efficient data exchange without delay.
* **Compact C++ logic** minimizes overhead while providing real-time performance.
* **Geometry-based intelligence** replaces map-based navigation — the robot learns from live sensor feedback.

---

# SOFTWARE MODULE REFERENCE TABLE

| Layer | Module / Function | Purpose / Description |
|-------|-------------------|----------------------|
| **Core System** | `setup_GPIO()` | Initializes all GPIO pins and sets their modes according to predefined constants. |
| | `motor(float speed)` | Controls main DC motor speed (0–1 range) through PWM to Cytron driver. |
| | `servo(float angle)` | Sets servo position according to the desired steering angle. |
| **LiDAR Subsystem** | `lidarStore` | Stores LiDAR scan data in two alternating buffers for efficient access and reduced latency. |
| | `printDeviceInfo(ILidarDriver* drv)` | Displays LiDAR identification and hardware parameters for verification. |
| | `checkHealth(ILidarDriver* drv)` | Monitors LiDAR operational status before scan start. |
| | `scanThread(ILidarDriver* drv)` | Runs in parallel, continuously reading new LiDAR frames into `lidarStore`. |
| | `eraseLidarData()` | Clears LiDAR data buffers before a new scan cycle. |
| | `updateLidarData()` | Converts raw LiDAR data into Cartesian and polar coordinate forms. |
| **Camera Subsystem** | `cameraStore` | Manages two frame buffers for fast image capture and access. |
| | `cameraThread(cv::VideoCapture* cap)` | Continuously reads camera frames and stores them in `cameraStore`. |
| | `process_raw_frame()` | Crops and preprocesses raw camera images to remove upper background zones. |
| | `process_hsv()` | Detects key color regions and masks relevant areas in HSV color space. |
| | `grayWorldWB()` | Applies automatic white balance correction for lighting consistency. |
| **Geometry & Navigation** | `angle_between_lines()` | Calculates the angle formed by intersecting line segments. |

---

# DETAILED PROGRAM FUNCTIONS' DESCRIPTION:

## 1. LiDAR Data Handling — LidarStore Module

The LidarStore namespace manages how LiDAR scan data is stored and accessed during operation.

It contains two internal buffers — `bufA` and `bufB` — that alternate between data collection and reading.

One buffer is always active and receives new LiDAR data, while the other remains available for access by other processes.

The program can request the latest LiDAR scan and its corresponding index, allowing consistent synchronization between data acquisition and navigation.

A separate parallel process updates these buffers in the background without interrupting access.

This structure helps the robot continuously receive fresh LiDAR information from the sensor while ensuring data integrity, even when multiple program threads run at the same time.

## 2. Camera Frame Management — CameraStore Module

The CameraStore namespace manages how images from the onboard camera are processed and stored for use in navigation and perception tasks.

It maintains two image buffers — `bufA` and `bufB` — which alternate between active and inactive states to ensure continuous frame updates without delays.

When a new frame arrives, it is written into the inactive buffer. Once ready, the program publishes it by switching the active index.

At any time, other processes can request the latest frame and its corresponding index for analysis.

A separate parallel process handles ongoing frame acquisition from the camera.

This setup allows the robot to keep receiving real-time video input while simultaneously using the most recent image for processing, without interrupting data flow or causing performance drops.

## 3. Recursive Fill Function — Visualization Utility

The `fill()` function implements a simple recursive flood-fill algorithm, similar to the "paint bucket" tool used in graphic editors.

Its purpose in the robot's visualization system is to make it easier to render or highlight regions of interest — for example, to display sensor coverage, obstacle areas, or path boundaries in test visualizations.

---

The function takes a pointer to a pixel array (`uchar* ptr`) and fills a connected area with a given value (`val`).

It uses recursion to propagate the fill operation in four directions — left, right, up, and down — within a defined rectangular boundary (e.g., `0L`, `w`, `0`, `h`).

This lightweight tool was designed for fast, memory-efficient graphical debugging, allowing to visually confirm how sensor data maps onto the environment during experiments.

## 4. GPIO Setup Function — Initialization Routine

The `setup_GPIO()` function initializes all Raspberry Pi GPIO pins according to the predefined constants used in the robot's control system.

Its role is to configure each pin with the correct direction (input/output) and mode (PWM, digital, etc.), ensuring that connected components such as the servo actuators, motor driver, and power logic can operate safely and predictably.

This setup is executed at program start and forms the foundation for all subsequent hardware interactions, providing a stable and consistent configuration baseline for the robot's control signals.

## 5. Servo Control Function — Angle-Based Positioning

The `servo(float angle)` function sets the rotation angle of the servo motor relative to its central (neutral) position.

It converts the given `angle` value into a corresponding PWM signal that determines the servo's shaft orientation.

This allows the robot to precisely control steering direction or mechanical articulation based on calculated geometry, ensuring accurate motion alignment between the servo and the rest of the drivetrain.

# 6. Motor Function — Speed Control Routine

The `motor(float speed)` function defines the rotation speed of the DC motor using a normalized value between `0` and `1`.

This value is translated into a PWM duty cycle that controls the effective voltage applied to the motor driver.

By adjusting the `speed` parameter, the program can smoothly regulate motor output — from a complete stop (`0`) to full power (`1`) — providing precise control over the robot's linear motion and acceleration.

## 7. printDeviceInfo Function — LiDAR Verification Utility

The `printDeviceInfo(ILidarDriver* drv)` function retrieves and displays basic device information from the connected LiDAR unit.

It is used for verification during initialization to confirm that the sensor is properly recognized and communicating with the system.

This routine outputs key parameters such as the model, firmware version, and hardware ID — allowing quick diagnostic checks before the main scanning process begins.

## 8. checkHealth Function — LiDAR Status Validation

The `checkHealth(ILidarDriver* drv)` function verifies the operational health of the LiDAR sensor before it starts scanning.

It queries the device for internal status reports to ensure there are no critical or warning-level errors.

This check is essential because the LiDAR is a complex and sensitive component — if its health status is not "OK," the system will avoid activating it to prevent unreliable operation or hardware malfunction.

## 9. scanThread Function — Parallel LiDAR Data Acquisition

The `scanThread(ILidarDriver* drv)` function runs as a dedicated parallel process responsible for continuously receiving scan data from the LiDAR sensor.

It collects distance and angle readings and writes them into the active data buffer maintained by the program.

This separation of scanning into its own thread ensures uninterrupted data acquisition — allowing other system components, such as navigation or visualization, to access the latest LiDAR data without delay or blocking.

## 10. cameraThread Function — Parallel Camera Frame Capture
The `cameraThread(cv::VideoCapture* cap)` function runs in a parallel thread and continuously reads frames from the robot's onboard camera.

Each captured frame is stored into one of two alternating memory buffers maintained by the system, ensuring smooth data flow and minimal delay.

This architecture allows the camera to operate independently from other processes such as navigation or mapping — keeping visual data constantly updated and accessible for analysis, display, or perception algorithms running on the main controller.

## 11. Angle Between Lines — Geometry Utility
The `angle_between_lines(const cv::Vec4i l1, const cv::Vec4i l2)` function computes the angle between two lines defined by OpenCV vectors (e.g., `x1`, `y1`, `x2`, `y2`).

It derives the direction of each line from its endpoints and returns the angle between these directions, treating the lines as infinitely extended (independent of their lengths or positions).

We use it for checking alignment, detecting corners, and validating geometric constraints in navigation or vision pipelines.

## 12. Process Raw Frame — Camera Preprocessing Function

The `process_raw_frame(uchar* raw_frame_p, uchar*& frame_p)` function processes raw camera input before it's used by the robot's main logic.

Its key role is to trim the upper portion of each captured frame — this removes unnecessary visual elements (like ceiling or distant background) that could distract the robot's perception algorithms.

The function operates as a lightweight preprocessing step, simplifying the visual field and ensuring that subsequent analysis focuses solely on the relevant ground-level area where navigation and obstacle detection occur.

## 13. Process HSV — Color Mask Extraction Function

The `process_hsv(uchar*& hsv_p, uchar*& red_p)` function processes the camera frame converted into the HSV color space and identifies specific color masks within it.

It isolates the areas of interest by filtering hues, saturation, and brightness levels corresponding to the target colors.

This allows the robot to recognize and track specific visual markers or colored objects on the field efficiently. The function prepares the image data for further analysis, ensuring consistent detection even under changing lighting conditions.

## 14. Point-to-Line Perpendicular Distance — Geometric Measurement Function

The `double pointLinePerpendicularDistance(const cv::Point2ds p, const cv::Vec4i& line)` function calculates the perpendicular distance from a given point to a specified line segment.

It is used to measure how far an object or detected feature lies from a reference line — a fundamental geometric operation for alignment, correction, or path-tracking tasks.

This function is essential for robot determining positional accuracy in navigation or for validating how closely detected contours match expected trajectories.

## 15. Middle Point — Line Center Calculation Helper
The `cv::Point2d middlePoint(const cv::Vec4i line)` function computes the midpoint of a given line segment.

It is a compact helper routine used to simplify geometric operations by quickly finding the center position between two endpoints.

We apply this function for visualization, alignment, and structural analysis of detected lines, helping reduce repetitive code and improve performance efficiency.

## 16. Line Intersection — Geometric Crosspoint Finder
The `cv::Point2d lineIntersection(const cv::Vec4i a, const cv::Vec4i b)` function determines the exact coordinates of the intersection point between two given lines, assuming they can be infinitely extended in both directions.

This function is used in the robot's geometry processing to analyze map structures, detect corners, and align navigation paths. It serves as a core mathematical utility that supports precise geometric reasoning based on LiDAR and camera line data.

## 17. Gray World White Balance — Brightness and Color Correction Function

The `void grayWorldWB(const cv::Mat& bgr, cv::Mat& bgrOut)` function performs automatic white balance correction based on the *gray world* assumption.

It adjusts the brightness and color balance of the image so that the program receives a more consistent visual feed — compensating for lighting variations such as direct sunlight, shadows, or low-light conditions.

This correction ensures that the robot's camera perceives colors and brightness more uniformly across environments, improving the accuracy of further image analysis and object detection.

## 18. Ray to Line Intersection Length — Directional Distance Calculation

The `long long RayToLineIntersectLength(cv::Vec4i a, const cv::Point2d b, double angle)` function calculates the distance between a given point and the intersection with a specified ray, along a ray projection from that point at a defined angle.

If the ray and line do not intersect, the function reports that no intersection exists.

We use it for geometric navigation logic — allowing the robot to determine how far it can "see" or move in a certain direction before encountering a boundary or detected line.

## 19. eraseLidarData — LiDAR Data Reset Routine

The `void eraseLidarData(all lidar arrays)` function clears all variables and buffers associated with LiDAR data.

Its purpose is to reset the LiDAR-related arrays after each processing cycle, ensuring that no residual data interferes with the next scan and that the system always works with a clean dataset for accurate readings.

## 20. updateLidarData — LiDAR Coordinate Formatting Function

The `void updateLidarData(auto& nodes, int nodes_count, auto& scanXY, auto& scanDa)` function converts incoming LiDAR measurements into three synchronized formats for further processing and visualization:

1. **Image Representation:** data is structured as a 2D visual map, where the LiDAR position is the center of the frame.
2. **Cartesian Coordinates:** points are translated into an X–Y coordinate system, where the origin is the upper-left corner of the image.
3. **Polar Coordinates:** each point is represented by its angle and distance from the LiDAR, treating the LiDAR as the origin.

This multi-format conversion allows the system to efficiently visualize, interpret, and process spatial data from the LiDAR for navigation and obstacle analysis.

## 21. combineLines — Line Grouping Function

The `void combineLines(auto& lines)` function processes an array of detected lines and merges those that are either very close to each other or nearly identical.

Its main goal is to simplify the dataset by combining redundant or overlapping lines into single, distinct segments.

This reduces computational load and ensures that only meaningful, distinct structural features are kept for further geometric analysis or path detection.

## 22. nextCorner — Next Turn Detection Function

The `cv::Point2d nextCorner(auto lines, int mainLineIdx)` function determines the coordinates of the next corner that the robot should reach while following a square path on the field.

Using the current main line (the robot's active trajectory) and the detected set of lines from the environment, it identifies the logical next corner in the robot's rotation cycle.

This is essential for navigation logic — since the robot must complete three laps around the field, it needs to continuously recognize when it reaches each corner to evaluate progress and adjust steering for the next segment of motion.

## 1. System Strategy — Autonomous Field Navigation

At program startup, the robot initializes all GPIO signals, essential constants, and runtime variables.

Once the activation button is pressed, the control system launches parallel threads for the LiDAR and camera subsystems, allowing real-time perception before motion begins.

Before enabling the motor, the LiDAR performs a pre-scan of the environment, measuring distances in four key diagonal directions — 45°, 135°, 225°, and 315°.

By comparing these measurements, the robot determines its initial orientation on the field without any prior movement.

This self-calibration method represents a major improvement over previous versions, where orientation drift at startup often caused early navigation errors.

After orientation is established, the robot enters its main operational loop, which continuously executes two parallel subsystems:

* **LiDAR-based Spatial Awareness** – The LiDAR system approximates the robot's current position relative to the map, allowing it to identify when it approaches a corner. The field is conceptually divided into eight angular sectors for simplified navigation and state tracking.
* **Camera-based Motion Control** – The camera subsystem maintains wall-following behavior using a PD algorithm. The image-processing pipeline detects the nearest boundary and adjusts the steering angle dynamically.

Together, these modules enable a hybrid control logic:

*"The camera shows the robot how to drive; the LiDAR tells it when to stop."*

When the final corner is passed, the robot continues forward for a fixed, pre-calibrated time interval — a constant determined experimentally to ensure it reaches the finish zone.

Upon arrival, the system stops the motor, safely terminates all background threads, and logs telemetry data such as camera frames, active masks, and other useful sensor information.

# 2. Tactical Implementation — Coordinated Sensor Logic

The Open Challenge solution follows a **two-layered control flow**:

| Stage | Subsystem | Function | Outcome |
|-------|-----------|----------|---------|
| **Initialization** | GPIO, I²/²S, Threads | Activate I/O and parallel processes | System ready for perception |
| **Pre-Scan Orientation** | LiDAR | Measures 4 diagonal ranges | Detects field alignment |
| **Navigation Phase** | LiDAR + Camera | Continuous sensing and PD control | Drives along perimeter via field |
| **Corner Detection** | LiDAR | Identifies sector transitions | Confirms robot's position in field |
| **Final Run** | Camera | Fixed-time drive toward goal | Reaches finish line |
| **Shutdown & Data Logging** | System Core | Stops all processes, saves telemetry | Safe termination and data storage |

This approach integrates precise mechanical control, real-time perception, and intelligent calibration.

The robot operates with predictable movement logic while maintaining adaptive behavior based on sensory feedback.

The balance between LiDAR spatial logic and camera-driven stability provides both accuracy and robustness — a distinctive feature of this year our Kyivrobomagic's 2025 team engineering solution☺.

# 4.2 OBSTACLE CHALLENGE ROUND.

## STRATEGY OVERVIEW:

The Obstacle Challenge uses the same multi-sensor framework as the Open Challenge, but with an extended logic layer designed to detect, classify, and bypass barriers. All core modules (LiDAR, Camera, IMU, GPIO control, and background threads) are initialized identically, but the navigation strategy diverges once the field is scanned.

After activation, the robot performs a LiDAR-based pre-scan to determine orientation and verify sensor stability. The system then enters the obstacle-navigation mode, where LiDAR and the camera operate in cooperative roles:

1. The LiDAR system approximates the robot's current position relative to the map, allowing it to identify when it approaches a corner. The field is conceptually divided into eight angular sectors for simplified navigation and state tracking.

2. Camera identifies immediate frontal obstacles using HSV masks, and gathers data for the PD-based steering adjustments.

3. The IMU provides high-frequency data to support fine adjustments in position. Mostly used in parking related activities.

## PROGRAM FLOW SUMMARY

Below is the simplified runtime flow of the full software system:

1. **System Initialization**
   - GPIO is configured for motor, servo, and button inputs.
   - IMU bridge process (Python) is launched.
   - LiDAR drivers and camera capture device are initialized.
   - Sensor buffers (LiDAR, Camera, IMU) are cleared and prepared.
2. **Wait for Start Button**
   - The robot remains idle until the physical start button is pressed.
3. **Start Multithreaded Sensor Acquisition**
   - **LiDAR Thread:** Reads scans, writes to double-buffer, updates coordinate maps.
   - **Camera Thread:** Captures frames, performs preprocessing, publishes latest image.
   - **IMU Thread:** Receives gyro/accel/quaternion packets, logs samples to IMUBuffer.
4. **Initial Field Orientation Detection**
   - Using LiDAR distance symmetry (90°, 270° beams),
   - the robot estimates its initial rotation before moving.
5. **Mission Execution Loop**
   - Runs until the challenge is finished or safety conditions stop the program.
     - **Camera subsystem**
       - Detects wall lines, calculates angle error, PD steering correction applied.
     - **LiDAR subsystem**
       - Tracks approximate location relative to corners.
       - Determines when a "segment" of the field is completed.
     - **IMU subsystem**
       - Handles accurate angle holding (rideAtAngle) and angle-based stopping (rideUntilAngle).
       - Used when fine adjustments are needed.
6. **Mission Completion**
   - Capture telemetry frames (masks, camera output, LiDAR visualization).
   - Gracefully terminate sensor threads.
   - Stop motor, center servo, stop IMU Python bridge.

# SOFTWARE MODULE REFERENCE TABLE

| Module / Namespace | Purpose | Notes |
|-------|---------|-------|
| **LidarStore** | Stores LiDAR scans in double-buffer | Provides thread-safe, latest-scan access |
| **CameraStore** | Stores two alternating camera frames | Enables real-time vision without blocking |
| **IMUBuffer + IMUStore** | Collects IMU samples continuously | Used for angle stabilization & PD steering |
| **cameraThread()** | Parallel camera→RAM acquisition thread | Creates live visual feed for perception |
| **scanThread()** | Parallel LiDAR scanning thread | Updates polar, map, and cartesian structures |
| **imuThread()** | UDP listener for IMU packets | Converts packets → IMUSample → buffer |
| **rideAtAngle()** | PD gyro steering at fixed angle for N nanoseconds | Used for long straight segments |
| **rideUntilAngle()** | Wait until robot reaches target angle | Used for corner alignment, re-orientation |
| **wait()** | Pure time-based delay | Motor state must be set externally |
| **motor(), servo()** | Actuators control | Use GPIO PWM |
| **combineLines()** | Vision post-processing | Merges noisy Hough lines |
| **nextCorner()** | Geometric corner locator | Key to multi-lap logic |
| **updateLidarData()** | Converts LiDAR scan → 3 representations | Map, Cartesian, Polar |
| **process_raw_frame(), process_hsv()** | Vision preprocessing | Cropping, HSV masking |
| **pointLinePerpendicular Distance()** | Math helper | Used for line-distance calculations |

# DETAILED PROGRAM FUNCTIONS' DESCRIPTION:

## 1) struct ImuHeader — IMU Timestamp and Metadata Block

ImuHeader is a small data structure used to store the essential metadata that accompanies every IMU sample.

Its purpose is not to process motion itself, but to provide *context* for each measurement so the program can correctly interpret when and in which order the data was captured.

```
struct ImuHeader {
    uint8_t  type;
    uint8_t  ver;
    uint16_t rsv;
    uint64_t t_ns;
    uint32_t seq;
};
```

## Field Purpose

* **type** — identifies the IMU message type.
  - Used so the program knows what kind of data it is about to read.
* **ver** — protocol or packet version.
  - Ensures compatibility between the IMU data format and the parser.
* **rsv** — reserved field (unused data).
  - Present for alignment or future expansion.
* **t_ns** — timestamp in nanoseconds.
  - This is the most important element:
    the robot receives many IMU samples per second, and precise timing is required to understand *when* each acceleration or rotation event occurred.
* **seq** — sequential packet counter.
  - Helps detect lost packets and maintain correct ordering during calculations.

## Role in the Robot's Software

ImuHeader allows the program to:

* track the exact moment each IMU reading was generated,
* correlate IMU readings with LiDAR and camera updates using unified timing,
* calculate motion over time (e.g., drift, rotation, acceleration changes),
* detect missing or out-of-order IMU messages.

Without this metadata, the robot could not reconstruct motion history or reliably integrate IMU data into navigation logic.

## 2) struct ImuPacket — Complete IMU Measurement Sample

ImuPacket represents a full IMU data message received from the Python IMU reader.

Each packet contains both the metadata (ImuHeader) and the actual motion measurements captured by the BNO085 sensor.

```
struct ImuPacket {
    ImuHeader h;
    float ax, ay, az;              // linear accel (body)
    float qi, qj, qk, qr;          // quaternion (x,y,z,w)
    float gz;                      // gyro Z (rad/s) in body frame
};
```

## Field Purpose and Function

### 1. Header

* **ImuHeader h**
  - Contains timing, sequence number, and packet format.
  - This allows the program to synchronize IMU data with LiDAR and camera inputs and maintain correct ordering of measurements.

### 2. Linear Acceleration

* **ax, ay, az** — linear acceleration of the robot in the IMU's body frame.
  - These values describe how the robot accelerates along its X, Y, and Z axes at the moment the sample was recorded.

Used for:

* short-term motion estimation between LiDAR scans,
* detecting subtle shifts (e.g., wheel slip),
* filling gaps where LiDAR resolution is too slow.
*

### 3. Orientation (Quaternion)

* **qi, qj, qk, qr** — quaternion components (x, y, z, w) representing rotation of the robot.
  - The program later converts this quaternion into yaw to understand the robot's orientation on the field.

Quaternions are used because:

* they do not suffer from gimbal lock,
* they allow smooth, stable rotation tracking,
* they integrate well with the robot's real-time navigation logic.

### 4. Gyro

* **gz** — angular velocity around the vertical axis (Z-axis), measured in radians per second.
  - This value describes how fast the robot is rotating left or right at the current moment.

Used for:

* detecting rotational drift over time,
* smoothing heading estimation between LiDAR measurements,
* stabilizing control loops that depend on robot orientation.

## Role in the Robot's Software

Together, the fields in ImuPacket provide the robot with a complete snapshot of its physical motion at a specific time.

The program uses these packets to:

* understand how the robot moves between LiDAR scans,
* stabilize orientation when camera data is unclear,
* keep consistent navigation even during turns, bumps, or uneven acceleration,
* maintain synchronized sensor fusion between IMU, LiDAR, and camera.

This packet format is the foundation of the robot's inertial compensation system.

# 3) struct IMUSample — Internal IMU Data Format for Fast Processing

After the program receives a raw ImuPacket from the Python IMU reader, the data is converted into a simplified internal format called IMUSample.

This structure keeps only the fields that the control algorithm actually needs, without transport-layer metadata.

It stores:

* the timestamp (in nanoseconds) when the program **received** the packet,
* the sequence number extracted from the original message,
* linear acceleration (`ax`, `ay`, `az`),
* the orientation quaternion (`qi`, `qj`, `qk`, `qr`),
* the angular velocity around Z (`gz`).

This conversion allows the program to work with a compact, uniform IMU representation, optimized for real-time navigation and fusion with LiDAR data.

```
struct IMUSample {
    uint64_t t_ns;      // time (receiving moment) in nanoseconds
    uint32_t seq;       // sequence number
    float ax, ay, az;   // linear acceleration
    float qi, qj, qk, qr; // orientation quaternion
    float gz;           // angular velocity around Z, rad/s
};
```

## 4) class IMUBuffer — Thread-Safe IMU Sample Storage and Retrieval

IMUBuffer is a small thread-safe container used to store incoming IMU readings (IMUSample) before processing cycles.

IMU data arrives asynchronously from a separate thread, and the robot's main loop requests IMU updates only once per control cycle.

This class guarantees that no IMU samples are lost during that time.

### Key Responsibilities

1. **Buffering incoming IMU samples**
   - Each time a new IMUSample is produced, the IMU thread calls `push()`.
1. The sample is appended to an internal vector under mutex protection, ensuring safe concurrent access.
2. **Batch retrieval of samples**
   - When the main program requests IMU data, it calls `take_and_clear()`.
   - This returns **all samples accumulated since the previous request**, in a single vector.
3. **Automatic clearing after retrieval**
   - After returning the data, the internal buffer is cleared so the next cycle starts fresh.
4. **Memory stability**
   - The class maintains a reserve size (default: 1024) to avoid unnecessary reallocations when pushing many samples.

## Code Reference

```
class IMUBuffer {
public:
    void reserve(size_t n) {
        std::lock_guard<std::mutex> lk(m_);
        buf_.reserve(n);
    }

    void push(const IMUSample& s) {
        std::lock_guard<std::mutex> lk(m_);
        buf_.emplace_back(s);
    }

    std::vector<IMUSample> take_and_clear() {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<IMUSample> out;
        out.swap(buf_);
        if (buf_.capacity() > reserve_)
            buf_.reserve(reserve_);
        return out;
    }

private:
    std::mutex m_;
    std::vector<IMUSample> buf_;
    size_t reserve_{1024};
};
```

# 5) namespace IMUStore — Global IMU Buffer Access Point

IMUStore is a minimal namespace that exposes a single global instance of IMUBuffer.

It serves as the centralized access point for all IMU-related data inside the program.

## Purpose

* The IMU acquisition thread pushes incoming samples into `IMUStore::log`.
* The main control loop retrieves accumulated samples on demand through the same object.

This design keeps IMU communication simple and avoids passing references throughout the codebase.

## Code Reference

```
namespace IMUStore {
    IMUBuffer log;
}
```
# 6) spawn_python_bridge — Launches the External Python IMU Process

This function starts the secondary Python program responsible for handling IMU communication and preprocessing.

Because the IMU works reliably only through the official Python library, the C++ program launches this helper script as a separate process.

## Purpose

* Fork the current process.
* In the child: replace execution with the Python interpreter running the IMU script.
* In the parent: return the PID of the Python process so the main program can manage or monitor it.

This is the bridge that connects the C++ control system to the Python-based IMU driver.

## Code Reference

```
static pid_t spawn_python_bridge(const char* py, const char* script) {
    pid_t pid = fork();
    if (pid == 0) {
        execl(py, "-u", script, (char*)nullptr);
        perror("execl");
        _exit(127);
    }
    if (pid < 0) perror("fork");
    return pid;
}
```

## 7) stop_python_bridge — Controlled Shutdown of the External IMU Process
This function stops the external Python program responsible for reading IMU data.

Because the IMU helper runs as a separate background process, it must be shut down explicitly; otherwise, it would continue running independently and interfere with the robot's operation.

The function works in two stages:

1. **Graceful stop:**
   - It sends a `SIGTERM` signal to the Python process and waits a short period (`grace_ms`) to allow the process to exit cleanly.
2. **Forced stop (fallback):**
   - If the process does not terminate within the allowed time window, the function sends `SIGKILL` and performs a final `waitpid` to ensure the process is fully stopped.

This guarantees that the IMU bridge process is always shut down correctly and will not remain active in the background after the robot program finishes.

## Code Reference
```
static void stop_python_bridge(pid_t pid, int grace_ms = 300) {
    if (pid <= 0) return;

    // Request a clean shutdown
    kill(pid, SIGTERM);

    // Wait for the process to exit gracefully
    for (int i = 0; i < grace_ms / 10; ++i) {
        int st = 0;
        pid_t r = waitpid(pid, &st, WNOHANG);
        if (r == pid)
            return; // Process exited normally
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Forcefully terminate if still running
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, WNOHANG);
}
```

## 8) imuThread — IMU Data Receiver Thread

This function establishes and manages the communication channel between the main robot program and the separate Python process responsible for reading and decoding the IMU sensor.

It creates a dedicated parallel thread that continuously listens for IMU data packets and stores them into the shared IMU buffer so the rest of the system can use them.

### Operational flow:

1. **Protocol Setup**
   - The function begins by declaring two packet structures (ImuHeader and ImuPacket), which describe the exact binary layout of the data sent across the Python bridge.
   - These structures allow the program to correctly interpret the incoming packet.
2. **Socket Initialization**
   - A UDP socket is opened on a fixed port (5556) using the loopback interface (127.0.0.1).
   - This means all IMU communication stays inside the Raspberry Pi and does not depend on external networks.

The function configures:

- `SO_REUSEADDR` to allow immediate rebinding,
- A large receive buffer (1 MB) to avoid packet loss,
- Non-blocking mode so the thread can continuously work without waiting.
2. **Buffer Preparation**
   - Before entering the main loop, the function reserves space inside the global `IMUStore::log` buffer so that bursts of IMU data can be absorbed without reallocation.
3. **Main Receive Loop**
   - While the robot program is running, the thread repeatedly:
     - Reads all available IMU packets from the socket,
     - Validates that the packet has the expected type and version (`type = 3`, `ver = 2`),
     - Converts the raw packet into an internal `IMUSample` format by adding local timestamp,
     - Pushes the sample into the IMU buffer for later use.

Once the first valid packet arrives, the thread prints a confirmation message indicating that IMU streaming has started.

4. **Idle Behavior**
   - If no packets are available during a cycle, the thread sleeps for 1 ms to avoid unnecessary CPU load.
5. **Shutdown**
   - When the `running` flag becomes false, the function exits the loop and closes the socket.
   - This ensures the IMU receiver stops cleanly when the robot ends its mission.

## Code Reference:

```
void imuThread() {
    #pragma pack(push,1)
    struct ImuHeader { uint8_t type, ver; uint16_t rsv; uint64_t t_ns; uint32_t seq; };
    struct ImuPacket { ImuHeader h; float ax, ay, az, qi, qj, qk, qr, gz; };
    #pragma pack(pop)

    constexpr int IMU_PORT = 5556;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("imu socket"); return; }

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    int rcvbuf = 1<<20;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port   = htons(IMU_PORT);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (sockaddr*)&a, sizeof(a)) < 0) {
        perror("imu bind");
        close(fd);
        return;
    }

    fcntl(fd, F_SETFL, O_NONBLOCK);
    IMUStore::log.reserve(4096);

    bool announced = false;
    while (running.load(std::memory_order_relaxed)) {
        for (;;) {
            ImuPacket p{};
            ssize_t n = recv(fd, &p, sizeof(p), 0);
            if (n < 0) {
                break;
            }
            if (n == (ssize_t)sizeof(p) && p.h.type == 3 && p.h.ver == 2) {
                IMUSample s{
                    now_ns(),
                    p.h.seq,
                    p.ax, p.ay, p.az,
                    p.qi, p.qj, p.qk, p.qr,
                    p.gz
                };
                IMUStore::log.push(s);
                if (!announced) { std::cerr << "[Imu] streaming (ver=2)\n"; announced = true; }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(fd);
}
```
# 9) rideAtAngle — Gyro-Based Direction Control for a Fixed Duration

The `rideAtAngle(double A, double time)` function maintains the robot's steering angle based solely on IMU orientation feedback for a specified duration measured in nanoseconds.

This routine **does not drive the motor** — it only controls the steering servo so the robot keeps its heading while another part of the program handles forward motion.

## Operational Logic

1. **Initialization**
   - The function records the starting timestamp.
   - Yaw error variables (`err` and `err_old`) are initialized for use in the proportional–derivative (PD) steering controller.
2. **Main Control Loop**
   - The loop continues while the global `running` flag is true.

   **a. Retrieve IMU samples**
   - The function pulls all available IMU readings by calling:
     `auto imu_batch = IMUStore::log.take_and_clear();`
   - This ensures the loop always works with the newest available data and avoids accumulation of stale measurements.

   **b. Process each IMU sample**
   - For every sample in the batch:

     - The timestamp difference `dt` is computed and clamped to avoid extreme values.
     - The quaternion orientation is normalized.
     - The robot's yaw angle `a` is computed from the quaternion using a standard `atan2` expression and normalized into the `[0, 2π]` interval.
   **c. Steering PD Control**
   - The robot computes the error between the measured yaw and the desired angle `A`:

```
err = a - A;
```

Then uses a PD formula to compute the servo position:

```
dir = 0.40 * err * 0.25 * (err - errOld) * 5;
```

The result is clamped to a safe servo range `[0.42, 0.54]`.

After computing the corrected direction, the function calls:

```
servo(dir);
```

to apply the new steering command.

The previous error is stored for the next iteration.

### 3. Termination Condition
The loop exits when:

```
now_ns() - beginTime >= time
```

meaning the function has maintained the angle for the requested duration.

### 4. Safety Button Check
During execution, the function checks the physical button state:

* Button holds increment a counter.
* If the counter exceeds a threshold, the global `running` flag is set to false.
* If the button is not pressed, the counter decreases but never below zero.

This provides an emergency shutdown mechanism during operation.

### 5. End of Function
Once the time expires or the program stops, the function exits without altering motor state (motor control is handled elsewhere in the program).

## Code Reference:
```cpp
void rideAtAngle(double A, double time) {
    double beginTime = now_ns();
    double err = 0, errOld = 0;

    while (running.load(std::memory_order_relaxed)) {
        auto imu_batch = IMUStore::log.take_and_clear();

        if (!imu_batch.empty()) {
            static uint64_t prev_ns = 0;
            double ax1, ay1, ax, ay;

            for (const auto& s : imu_batch) {
                double dt = (prev_ns == 0) ? 0.0 : (double)(s.t_ns - prev_ns) * 1e-9;
                if (dt <= 0.0) dt = 0.00001;
                dt = (dt > 0.02) ? 0.02 : dt;
                prev_ns = s.t_ns;

                float w = s.qr, x = s.qi, y = s.qj, z = s.qk;
                float n = std::sqrt(w*w + x*x + y*y + z*z);
                if (n > 0.0f) { w /= n; x /= n; y /= n; z /= n; }

                double a = std::atan2(float(2.0f * (w*z + x*y)),
                                     float(1.0f - 2.0f * (y*y + z*z)));
                a = a + 2 * M_PI;
                if (a > 2 * M_PI) a -= 2 * M_PI;

                double dir = 0.48;
                err = a - A;
                dir = 0.48 + err * 0.25 + (err - errOld) * 5;
                dir = std::min(std::max(dir, 0.42), 0.54);

                servo(dir);
                errOld = err;
            }
        }

        if (now_ns() - beginTime >= time)
            break;

        if (is_button_down()) buttonSum++;
        else buttonSum--;

        if (buttonSum < 0) buttonSum = 0;
        if (buttonSum > 100) running = false;
    }
}
```
# 10) rideUntilAngle — IMU-Based Wait Until Target Heading

The `rideUntilAngle(double A, bool dir, double time)` function acts as a **blocking wait** based on the robot's orientation.

It **does not control** the motor or servo directly. Instead, it repeatedly checks the IMU yaw angle and exits when:

* the robot's heading reaches the specified angle `A` (in the chosen direction), or
* an optional maximum time limit is reached, or
* the safety button stops the program.

This function is typically used **after** the motor and servo have already been set, for example as a delay before braking or before switching to another maneuver.

## Parameters

* **A** — target yaw angle (in radians, in `[0, 2π]`) that the robot should reach.
* **dir** — direction flag:
  - If `true` → the function waits until the current angle **grows to or beyond** `A` (e.g. turning from right to left).
  - If `false` → the function waits until the current angle **decreases to or below** `A`.
* **time** — maximum allowed duration (in nanoseconds) for this waiting:
  - If `time == -1`, the function exits when this time interval has passed, even if the target angle has not been reached
  - If `time == -1`, the time limit is disabled and only the angle (or stop button) can end the function.

This time limit is useful when, for example, the robot is blocked by a wall and cannot physically rotate to the requested angle.

## Operation

1. Store the start time with `beginTime = now_ns()`.
2. In a loop (while `running` is true):
   - Retrieve all new IMU samples with `IMUStore::log.take_and_clear()`.
   - For each sample:
     * Compute yaw angle `a` from the quaternion (`qr`, `qi`, `qj`, `qk`).
     * Normalize `a` into the range `[0, 2π]`.
     * Based on `dir`:
       - If `dir == true` and `a >= A` → set `endThis = true` and break.
       - If `dir == false` and `a <= A` → set `endThis = true` and break.
   - If `endThis` is set, exit the loop.
   - If `time` is not `-1` and the elapsed time exceeds `time`, exit the loop.
   - Update the `buttonSum` counter using `is_button_down()`; if it exceeds the threshold, set `running = false`.

After exiting, the function simply returns; it does not change motor or servo state.

## Code Reference:
```cpp
void rideUntilAngle(double A, bool dir, double time) {
    double beginTime = now_ns();
    bool endThis = false;

    while (running.load(std::memory_order_relaxed)) {
        auto imu_batch = IMUStore::log.take_and_clear();

        if (!imu_batch.empty()) {
            static uint64_t prev_ns = 0;
            double ax1, ay1, ax, ay;

            for (const auto& s : imu_batch) {
                double dt = (prev_ns == 0) ? 0.0 : (double)(s.t_ns - prev_ns) * 1e-9;
                if (dt <= 0.0) dt = 0.00001;
                dt = (dt > 0.02) ? 0.02 : dt;
                prev_ns = s.t_ns;

                float w = s.qr, x = s.qi, y = s.qj, z = s.qk;
                float n = std::sqrt(w*w + x*x + y*y + z*z);
                if (n > 0.0f) { w /= n; x /= n; y /= n; z /= n; }
                double a = std::atan2(float(2.0f * (w*z + x*y)),
                                     float(1.0f - 2.0f * (y*y + z*z)));
                a = a + 2 * M_PI;
                while (a < 0)        a += 2 * M_PI;
                while (a > 2 * M_PI) a -= 2 * M_PI;

                if (dir) {
                    if (a >= A) {
                        endThis = true;
                        break;
                    }
                } else {
                    if (a <= A) {
                        endThis = true;
                        break;
                    }
                }
            }
        }
        if (endThis) break;

        if (time != -1 && now_ns() - beginTime >= time)
            break;

        if (is_button_down()) buttonSum++;
        else buttonSum--;

        if (buttonSum < 0)    buttonSum = 0;
        if (buttonSum > 100) running = false;
    }
}
```

# 11) wait — Simple Blocking Delay

The `wait(double time)` function implements a **pure time-based blocking delay**.

It does not read any sensors, does not modify motor or servo state, and does not perform safety checks.

It simply pauses program execution for the specified number of nanoseconds.

This function is typically used in movement sequences where the robot must run the motor for a fixed duration.

For example: "drive forward for X nanoseconds, then stop", or "pause before braking".

## Behavior

1. Record the current timestamp using `now_ns()`.
2. Enter a tight loop that exits only when the elapsed time reaches the requested duration.
3. Return immediately once the condition is met.

## Code Reference:

```cpp
void wait(double time) {
    double startTime = now_ns();
    while (now_ns() - startTime < time) {}
}
```

# PART 5. RESOURCES LIBRARY

## INTERESTING INFORMATION:
https://patents.google.com/patent/US5129275A/en

## GITHUB LINK: https://github.com/KyivRoboMagic/WRO2025.git

## YOUTUBE LINK: https://www.youtube.com/@Peter-dp4nd

## OBJECT CHALLENGE: https://youtu.be/LZ57Y01bQvU

## OPEN CHALLENGE: https://youtu.be/bDnhjbADkbo

<img src="./readme images/66 logo.jpg" style="width:100%; height:auto;">
