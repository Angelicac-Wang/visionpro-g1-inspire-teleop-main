# Whole-Body Teleoperation with Apple Vision Pro  
## Internship Progress Report

| | |
|---|---|
| **Author** | zhiying |
| **Program** | Mitacs Globalink Research Internship (GRI) — summer internship |
| **Report date** | March 2026 |
| **Period covered** | Approximately the past two to three months of internship work |
| **Reference** | REMS draft: `Bare_Hand_Humanoid_Teleoperation_for___Hazardous_Confined_Environments__REMS.pdf` |

---

## 1. Purpose of This Report

I joined this lab as a **Mitacs GRI summer intern**. This report summarizes **what I have done over the past two to three months** and shares that progress with my supervisor and senior. 

The work builds on code and documentation left by a Huyue. My contribution is the **extension, integration, tuning, and documentation** described below.

---

## 2. Problem Statement and Motivation

The problem I set out to address is **whether a single Apple Vision Pro headset—without extra controllers or body-worn hardware—can teleoperate an entire Unitree G1 humanoid**, including walking, turning, arm motion, and finger control, rather than only one arm or the upper body.

Relatively **few groups are exploring whole-robot control using only a Vision Pro (virtual glass) as the sole input device**. In **civil engineering and infrastructure inspection**, teleoperation setups still rely heavily on **quadrupeds (robot dogs) or fixed robot arms**; **humanoid platforms remain uncommon**. Yet many hazardous or confined sites—tunnels, culverts, partially collapsed structures—require a **bipedal, dual-arm humanoid** if the goal is to **substitute for a human who must walk, look, and manipulate in the same space**. A single lightweight headset interface is **simple for the operator to wear and deploy**: no extra controllers, no body-mounted mocap suit, and both hands stay free for natural gesturing during transit. That combination motivated us to push beyond arm-only teleop toward **full-body** control from one consumer device.

---

## 3. Starting Point (Prior Work in the Lab)

The baseline repository is `visionpro-g1-inspire-teleop` by Huyue. At the start of my internship it already provided:

| Component | Capability |
|-----------|------------|
| Vision Pro → robot arms | Hand motion mapped to arm targets |
| Finger → Inspire hand | Pinch/open commands to sim or hardware |
| SONIC interface | Commands to the whole-body planner |
| Basic calibration | Operator “zero” pose before teleop |
| Real robot, right hand | One physical Inspire hand via a Modbus driver |
| Usage notes (Chinese) | Startup order and calibration |

What was **not** yet in place: head-driven walking, a full MuJoCo sim workflow, staged calibration (F → ] → S → T), dual-hand drivers, keyboard-assisted walking for lab debugging, experiment logging, and consolidated English operations documentation.

---

## 4. Timeline and Technical Work

### 4.1 MuJoCo simulation and Vision Pro interaction (early internship)

I was first asked to **learn how the robot stack is operated in a safe virtual setting** before touching hardware. The main deliverable here was **building a MuJoCo simulation environment** in which the **Unitree G1 appears as a full humanoid** and can be controlled end-to-end. I integrated the G1 model with the SONIC whole-body policy and the Vision Pro bridge so that **an operator wearing Apple Vision Pro can interact with the simulated robot**—moving the head and hands and seeing the robot respond in the scene.

### 4.2 Right-hand alignment (limited progress)

I initially spent time on **right-hand alignment**—improving how closely the robot right arm and Inspire hand follow the operator’s hand in space and orientation. Progress was ** slower than expected** because of mapping sensitivity, calibration drift, and coupling with the whole-body controller. Rather than blocking the rest of the project on perfect arm alignment, we **prioritized head-driven locomotion** so that whole-body teleoperation could move forward in parallel. 

### 4.3 Head-driven locomotion

I implemented **head-driven locomotion** so the operator can walk the robot **without using the hands for movement**. The design separates two signals from the Vision Pro head pose:

- **Head displacement** (translation): horizontal motion of the head relative to a calibrated reference is converted into **forward, backward, and lateral walking** commands.
- **Head facing** (orientation): head yaw defines **which direction the robot body faces**, independent of the instantaneous travel direction. This allows behaviors such as walking backward while the torso keeps a fixed heading.

The pipeline applies smoothing and velocity dead zones so small head jitter does not move the robot. When the operator **lowers the head** beyond a threshold, the system commands a **lower pelvis height** (squat/kneel band), which supports inspection in reduced vertical clearance. On the real robot, **base orientation feedback** is used to reduce gradual heading drift during corridor-style walking.

### 4.4 Staged calibration

I added a **four-step calibration protocol** so operator motion matches the robot more consistently:

1. **F (CALIB_FULL)** — Operator holds a forearms-forward L-shape for ~2 s; the system records reference head and wrist poses.  
2. **]** — Engage the balance policy; the robot holds a stable init pose.  
3. **S (CALIB_SYNC)** — Operator matches the robot on screen and holds ~2 s; wrist zeros and mapping base update.  
4. **T (TELEOP)** — Live teleoperation begins.

**H (HEAD zero)** re-zeros walking facing and squat height mid-session without repeating the full arm calibration. This protocol reduced sudden arm drops at policy start and improved repeatability between sessions.

### 4.5 Egocentric video (robot first-person view) — **not yet complete**

Infrastructure was added to stream the **simulated head camera** toward the Vision Pro via WebRTC (MuJoCo image publish on port 5555, bridge forwarding). **However, this feature is not successfully usable today.** Due to **rendering / streaming issues**, the operator **cannot reliably see the robot’s first-person view inside the Apple Vision Pro**.

### 4.6 Real-robot stability and lab controls

On hardware, the robot initially **tracked the operator too aggressively**: small or noisy hand motions were amplified into **visible arm jitter**, and rapid operator movements could produce **sudden large joint commands**. That behavior is undesirable on a real platform—it stresses the mechanism and creates risk for **nearby people** in a shared lab. I addressed this in three layers.

**Smoothing and rate limits.** I added **arm command smoothing** and **velocity caps** so the robot does not react instantaneously to every frame of hand motion. Smoothing reduces high-frequency shaking in the arms; limiting how fast targets may change prevents **abrupt full-range swings** when the operator moves quickly or when tracking flickers. Together, these changes make teleop feel less “twitchy” and safer to run beside the physical robot.

**Keyboard-assisted locomotion for small labs.** Pure head-driven walking is hands-free, but it assumes the operator can **physically turn and lean** in open space. In our lab, space is limited. I added an optional **keyboard overlay** (`--hybrid-locomotion`: W/A/S/D) that works **together with head control of the feet**: the operator can **steer walking direction from the keyboard while standing roughly in place**, reach a desired spot without walking around the room, and then use **small head motions for fine positioning** once the robot is near the target. Default teleop remains **head-only** for consistency with the hands-free design; hybrid mode is mainly for **tight indoor debugging** on the real robot. 

**Arm tracking hold.** Vision Pro hand tracking is not continuous: the left or right wrist can **drop out of view** for a frame or longer. Previously, a lost wrist caused the arm target to **snap back toward the init pose**, which pulled the **whole upper body** into sudden, chaotic motion. I added **hold-last-valid-pose** behavior (`--arm-tracking-hold`, on by default): when either hand is not visible, the robot **keeps the last good arm command** instead of resetting, then **smoothly resumes** when tracking returns. This makes dropouts much less disruptive during real-robot sessions.

### 4.7 Left-arm tuning (ongoing)

A large share of late-internship effort went into the **left arm**, which remained less reliable than the right even after the right-hand pipeline was working.

**What we saw.** Copying the right-hand mapping to the left produced poor results: raising the left hand often caused the robot to **hunch or twist the torso** instead of reaching upward cleanly, and **reachable height** was lower than the operator’s hand. 

**Why the left is harder.** Arm targets are not symmetric left/right in our bridge: the default **left-hand delta remap** (`unitree-left-arm`) **couples vertical (Z) motion into forward (X) motion**, so “reach up” reads partly as “reach forward” and fights the SONIC whole-body policy. **Wrist orientation** also needs a different axis basis and sign convention on the left; the right-hand palm mapping does not transfer directly.

**What we changed.** I settled on a **left-specific preset** (now the sim/real default in the launch scripts):
- Hand translation: **`identity` delta remap** (decouple Z from X for freer reach-up).  
- Wrist orientation: **`calibrated` mode** with **`avp-palm-left` axis remap** and **Y rotation sign +1.0** (other wrist signs unchanged from the tuned right-side baseline).  
- Together with §4.6 smoothing, velocity limits, and **arm tracking hold**, left-arm teleop is **usable for basic single-hand tasks** but still not as clean as the right.

---

## 5. Current Capabilities (Informal Testing)

These are **informal lab observations**, not formal study results.

**Tasks that currently work reasonably well (mostly single-hand, basic manipulation):**

| Task | Notes |
|------|--------|
| Grasping a tool | Single-hand grasp of tool-like objects |
| Pick-and-place into a box | Grasp → move → release into a container |
| Opening a door| Push/pull handle motions |

**Tasks that remain difficult:**

| Task | Notes |
|------|--------|
| Two-hand balanced box carrying | Unstable; whole-body controller struggles with dual-arm load and balance |
| Left arm reach without torso distortion | Mapping and wrist settings still being tuned |
| Robot first-person view in the headset | Egocentric video pipeline exists but is **not reliably viewable** in Vision Pro (rendering/streaming issue) |
| Sustained teleoperation sessions | **Shoulder and pelvis motors overheat** after extended use; long continuous runs are not practical today without cooldown |

---

## 6. Summary

**Completed during the internship**

1. MuJoCo sim workflow and one-click scripts for operator training.  
2. Head-driven locomotion from **head displacement and facing**, including squat on head lowering.  
3. Staged calibration F → ] → S → T and mid-session **H** reset.  
4. Real-robot smoothing and heading stabilization; dual-hand driver plumbing; `g1_teleop` refactor and operations docs.  
5. Extensive parameter tuning (behavior smoothing, tracking loss hold, left arm).  
6. From **August**: with my supervisor, scoped paper/experiments from REMS-scale ideas to **basic manipulation**.

**Not completed / in progress**

- **Egocentric video in Vision Pro** (rendering issue; not demo-ready).  
- Two-hand box carrying and other heavy bimanual tasks.  
- Formal multi-trial experiment statistics for Task A and manipulation tasks.  
- Stable long-run dual-hand hardware teleop.

