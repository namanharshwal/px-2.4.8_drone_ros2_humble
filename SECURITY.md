# Security Policy & Autonomous Drone Safety Standards

**Project**: Autonomous F450 Quadcopter with Pixhawk 2.4.8 & ROS 2 Humble  
**Author & Maintainer**: Naman Harshwal ([@namanharshwal](https://github.com/namanharshwal))  
**Copyright**: © 2025–2026 Naman Harshwal. All Rights Reserved.

---

## 1. Supported Versions

Security updates, vulnerability patches, and safety advisories are actively maintained for the following versions:

| Version | Status | Supported | Recommended Action |
| :--- | :--- | :--- | :--- |
| **v1.0.x** | Current Production Release | ✅ Yes | Recommended for all physical F450 deployments |
| **< v1.0.0** | Prototyping / Pre-release | ❌ No | Deprecated — Upgrade to v1.0.0 immediately |

---

## 2. Reporting a Vulnerability or Safety Issue

If you discover a security vulnerability, buffer overflow in middleware serialization, or an unhandled hardware failsafe state that could cause autonomous drone flyaways:

1. **Do NOT open a public GitHub issue.**
2. Send a detailed report directly to the project author:
   - **Lead Developer**: Naman Harshwal
   - **Email / Security Contact**: `namanharshwal@github.com` (or via private GitHub Security Advisory)
3. **Include in your report**:
   - Drone firmware version (PX4 1.14 build hash) and ROS 2 package commit.
   - Step-by-step reproduction steps (e.g. sequence of MAVLink or DDS topics published).
   - Observed behavior vs. expected failsafe trigger.
   - Any hardware telemetry logs (`.ulg` files from `/fs/microsd/log/`).
4. **Response Timeline**:
   - Acknowledgement within **48 hours**.
   - Technical evaluation and validation within **5 business days**.
   - Fix release or safety advisory published within **14 business days**.

---

## 3. Hardware & Cyber-Physical Security Architecture

Autonomous aerial robotics involves high-speed rotating propellers, high-discharge lithium-polymer batteries, and high-frequency wireless communications. This system incorporates the following layered defense mechanisms:

### 3.1 Physical Isolation of Companion Link
- The link between the Raspberry Pi 4 companion computer and Pixhawk 2.4.8 operates over a **hardwired physical UART serial bus** (`/dev/ttyAMA0` on GPIO Pins 8 & 10).
- This serial line is physically isolated from the internet and local Wi-Fi, preventing remote injection of raw MAVLink actuator test commands from unauthorized network hosts.

### 3.2 Network Domain Isolation (`ROS_DOMAIN_ID = 42`)
- Default ROS 2 implementations use `ROS_DOMAIN_ID=0`, which broadcasts discovery packets across the entire local subnet.
- In shared laboratory, educational, or flight-field Wi-Fi networks, third-party ROS 2 nodes can inadvertently cross-talk with `/fmu/in/` topics.
- Locking both the Pixhawk parameter (`XRCE_DDS_DOM_ID=42`) and the companion environment (`export ROS_DOMAIN_ID=42`) restricts DDS communication strictly to nodes explicitly tagged with Domain 42.

### 3.3 Hardware Radio Failsafe Override
- The physical 2.4GHz FlySky FS-i6 radio transmitter maintains **absolute manual override priority** over companion computer autonomy.
- Flipping auxiliary **Channel 5** out of Offboard mode into Position, Altitude, or Manual mode instantly strips the companion computer of actuator control and hands flight authority back to the manual pilot.

### 3.4 Physical Safety Button Lockout
- The Pixhawk hardware safety button (mounted on the GPS mast) physically disconnects PWM power rails to the ESCs via the onboard STM32F100 safety co-processor until deliberately armed.

---

## 4. Legal & Operational Disclaimer

This software, schematics, and documentation are provided for educational, academic, and research prototyping purposes under the **Apache License 2.0**.

- **No Aerial Warranty**: Operating autonomous unmanned aerial vehicles (UAVs) carries inherent risks of physical injury, property damage, and airspace violations.
- **Operator Responsibility**: The operator assumes full and sole legal responsibility for compliance with all local aviation authorities (e.g., FAA Part 107, DGCA, EASA), safety clearances, and pre-flight inspections.
- **Exclusion of Liability**: Under no circumstances shall **Naman Harshwal** or project contributors be held liable for any direct, indirect, incidental, special, exemplary, or consequential damages resulting from the assembly, bench testing, or flight of this vehicle.
