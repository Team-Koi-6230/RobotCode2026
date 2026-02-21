# 🏗️ Team Koi 6230 — REBUILT™ (2026)
### *Official Software Overview — Presented by Haas*

This is the official 2026 software for Team Koi 6230's REBUILT™ robot. After a 2025 season where software limitations forced us into a defensive role, this code represents a total ground-up refactor. We have moved to a state-based architecture to ensure we are a dominant offensive force in the 2026 ISR District.

---

## 🚀 Technical Specifications
| Component | Technology |
| :--- | :--- |
| **Swerve Library** | YAGSL (Yet Another Generic Swerve Library) |
| **Autonomous** | PathPlanner 2026 |
| **Hardware Control** | REVLib (SPARK MAX/Flex) |
| **Telemetry** | Elastic Dashboard & Advantage Scope |
| **Control Strategy** | Superstructure-led Periodic States |

---

## 🧠 Software Architecture
In our 2026 "REBUILT" framework, subsystems are **passive followers**. They don't make decisions; they simply execute the state provided by the Superstructure. 

### 🏎️ Drivetrain (Swerve)
Powered by **YAGSL**, this subsystem is responsible for all chassis movement.
* **Odometry:** Updates real-time pose estimation to the **Elastic Dashboard**.
* **Input:** Consumes `ChassisSpeeds` from the Superstructure, ensuring movement is always smooth and field-oriented.
* **Abstraction:** The `SwerveDrive` object abstracts our four modules, eliminating the inconsistencies that plagued us in 2025.

### ⚙️ Mechanism Subsystems (Intake, Elevator, Shooter)
Each mechanism follows a standard **State-Follower** template:
* **Desired State:** Internal variables represent targets (e.g., target RPM for shooter, target height for elevator).
* **Periodic Loop:** The `periodic()` method uses **REVLib’s internal PID controllers** to drive motors to the desired state. 
* **1ms Update Rate:** By running PID directly on the SPARK MAX, we achieve frequencies far higher than the RoboRIO could provide.
* **Hardened Safety:** Subsystems monitor current draw and limit switches. If a stall is detected, the subsystem overrides the Superstructure to prevent hardware damage.

---

## 📈 Evolution: Leaving Defense Behind
> "In 2025, our software limitations forced us into a defensive role. Even in the finals at ISR District Event #1, we were a 'fourth robot' because our auto was mid and our controls were laggy."

The **REBUILT project** changes that narrative. By switching to YAGSL and PathPlanner, we have eliminated the drivetrain inconsistencies that made us a target for penalties. 
* **Seamless Transitions:** Transition from intaking to scoring with a single button press zero delay, zero command conflicts.
* **Aggressive Play:** We are no longer just a defensive blocker; we are a system designed to out-score the best teams in Israel (1690, 5614, we're coming for you). 🇮🇱

---

## 🛠️ Setup and Development
1. **Toolchain:** Ensure **2026 WPILib Game Tools** are installed.
2. **Workspace:** The `.vscode` folder contains optimized settings for the team.
3. **Build:** Run `./gradlew build` to fetch the latest **YAGSL** and **REVLib** dependencies.
4. **Simulation:** Use **WPILib Simulation** (`simgui-ds.json`) to verify State transitions before testing on the physical bot.

---
*Created by the Team Koi #6230 Software Team.*