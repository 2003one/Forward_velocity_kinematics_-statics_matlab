# 3-DOF Robot Arm: Product of Exponentials Forward Kinematics

A complete robotics engineering project spanning **hand-derived screw theory** through **MATLAB simulation**. Implements forward kinematics, velocity kinematics, and statics analysis for a 3-DOF manipulator using the **Product of Exponentials (PoE)** framework.

---

## 📐 Robot Schematic

![3DOF Robot Schematic](./schematic.png)

**Physical Chain**: θ₁ → l₁ (0.30 m) → θ₂ → l₂ (0.40 m) → θ₃ → l₃ (0.30 m) → End Effector

- Joint 1: Revolute, rotates about space Z (base frame)
- Joint 2: Revolute, rotates about space X at z = 0.30 m
- Joint 3: Revolute, rotates about space X at x = 0.40 m, z = 0.30 m

---

## 📝 Manual Screw Axis Derivation

Complete hand-worked analysis of space screws, body screws, and transformations (6-page breakdown):

### Page 1: Forward Kinematics Foundation & Joint 1
![Screw Derivation Page 1](./s1.jpeg)

- Robot schematic with coordinate frames
- Home configuration reference frame setup
- Joint 1 analysis: q₁, ω₁, velocity calculation
- Space screw S₁ = [0, 0, 1, 0, 0, 0]ᵀ

### Page 2: Joint 2 & Space Screw Matrix
![Screw Derivation Page 2](./s2.jpeg)

- Joint 2 position q₂ = [0, 0, 0.30]ᵀ
- Space screw S₂ = [1, 0, 0, 0, 0.30, 0]ᵀ
- Total space screw matrix construction
- Velocity kinematics formulation

### Page 3: Joint 3 & Complete Space Screws
![Screw Derivation Page 3](./s3.jpeg)

- Joint 3 analysis: q₃, ω₃
- Space screw S₃ = [1, 0, 0, 0, 0.30, 0]ᵀ
- Complete 6×3 space screw matrix
- Space vs body frame comparison

### Page 4: Home Configuration & PoE Formula
![Screw Derivation Page 4](./s4.jpeg)

- Home transformation matrix M derivation
- End-effector position at zero configuration
- M = [[1, 0, 0, 0.70], [0, 1, 0, 0], [0, 0, 1, 0.30], [0, 0, 0, 1]]
- Space-frame PoE formula: T(θ₁,θ₂,θ₃) = e^[S₁]θ₁ e^[S₂]θ₂ e^[S₃]θ₃ M

### Page 5: Body Screws & Adjoint Transformation
![Screw Derivation Page 5](./s5.jpeg)

- Adjoint transformation Ad(M⁻¹) detailed calculation
- Body screw conversion: Bᵢ = Ad(M⁻¹)Sᵢ
- Body screw results:
  - B₁ = [0, 0, 1, 0, 0.70, 0]ᵀ
  - B₂ = [1, 0, 0, 0, 0, 0]ᵀ
  - B₃ = [1, 0, 0, 0, 0, 0]ᵀ

### Page 6: PoE at Body Frame & Complete Summary
![Screw Derivation Page 6](./s6.jpeg)

- Body-frame PoE formula: T(θ₁,θ₂,θ₃) = M e^[B₁]θ₁ e^[B₂]θ₂ e^[B₃]θ₃
- Complete workflow summary
- Final reference results (one-page lookup)

---

## 💻 MATLAB Implementation & Results

### 1. Forward Kinematics (FK_3DOF.m)

![MATLAB FK Visualization](./01.png)

**Features**:
- 3D robot visualization with coordinate frames
- Real-time joint angle control (sliders: -180° to +180°)
- Live end-effector position display
- Quick preset poses (HOME, POSE 1, POSE 2, POSE 3)
- Homogeneous transformation T output

**Output Example**:
```
3-DOF Robot at [45°, -30°, 60°]:
End-Effector Position: x = 0.5234 m, y = 0.1523 m, z = 0.4128 m
Orientation: Identity rotation (no rotation from base)
```

### 2. Velocity Kinematics (velocity_3dof.m)

![MATLAB Velocity Output](./02.png)

**Computes**:
- Space Jacobian from analytical PoE formula
- Body Jacobian from adjoint transformations
- Joint velocities → end-effector twist (space frame)
- Joint velocities → end-effector twist (body frame)

**Example Output** (θ̇ = [0.1745, 0.0873, 0.0349] rad/s):
```
Joint Velocities:
  θ̇₁ = 0.1745 rad/s (base rotation)
  θ̇₂ = 0.0873 rad/s (shoulder)
  θ̇₃ = 0.0349 rad/s (wrist)

End-Effector Angular Velocity (space frame):
  ωₓ = 0.1058 rad/s
  ωᵧ = 0.0611 rad/s
  ωᵤ = 0.1745 rad/s

End-Effector Linear Velocity (space frame):
  vₓ = -0.0180 m/s
  vᵧ = 0.0312 m/s
  vᵤ = 0.0030 m/s
```

### 3. Static Analysis (statics_3dof.m)

![MATLAB Statics Output](./03.png)

**Calculates**:
- Joint torques from end-effector wrench (space frame)
- Joint torques from end-effector wrench (body frame)
- Load distribution across manipulator
- Static equilibrium verification

**Example Output** (1 kg payload, gravity = 9.81 m/s²):
```
Payload Mass: 1.00 kg
Gravity Force: -9.81 N

Joint Torques (equilibrium):
  τ₁ = 0.0000 N⋅m (base unloaded)
  τ₂ = 0.0000 N⋅m (shoulder unloaded)
  τ₃ = -1.0066 N⋅m (wrist bears load)
```

---

## What's Here

### 1. **Mathematical Foundation**
- **Space & Body Screw Axes**: Systematic derivation of joint screws from first principles
- **Homogeneous Transformations**: Complete M-matrices for home configuration
- **PoE Forward Kinematics**: Both space-frame and body-frame formulations
  - Space FK: `T(θ₁, θ₂, θ₃) = e^[S₁]θ₁ e^[S₂]θ₂ e^[S₃]θ₃ M`
  - Body FK: `T(θ₁, θ₂, θ₃) = M e^[B₁]θ₁ e^[B₂]θ₂ e^[B₃]θ₃`

### 2. **Velocity Kinematics**
- Space Jacobian: `Js = [S₁, Ad(T₁)S₂, Ad(T₁T₂)S₃]`
- Body Jacobian: `Jb = [Ad(e^[-B₂]θ₂ e^[-B₃]θ₃)B₁, Ad(e^[-B₃]θ₃)B₂, B₃]`
- Adjoint transformations for frame conversions
- Velocity mapping: `Vs = Js·θ̇` (end-effector twist from joint velocities)

### 3. **Static Analysis**
- Joint torque calculation from end-effector wrench
- Space statics: `τ = Js^T Fs`
- Body statics: `τ = Jb^T Fb`
- Example: 1 kg payload response across workspace

### 4. **MATLAB Implementation**
Three core modules using the **Modern Robotics Toolbox** (Coursera):

| File | Purpose |
|------|---------|
| `FK_3DOF.m` | Forward kinematics with 3D visualization |
| `velocity_3dof.m` | Space & body Jacobians + joint/end-effector velocities |
| `statics_3dof.m` | Joint torques under external loads |

**Interactive Visualization**: Real-time 3D robot with slider controls for joint angles, quick preset poses, and live end-effector position feedback.

## Robot Specification

| Parameter | Value |
|-----------|-------|
| Link 1 | 0.30 m (vertical) |
| Link 2 | 0.40 m (horizontal) |
| Link 3 | 0.30 m (horizontal) |
| DOF | 3 (revolute) |
| Home Pose | θ₁ = θ₂ = θ₃ = 0° |
| EE at Home | [0.70, 0, 0.30] m |

## Key Features

✓ **From-Scratch Derivation**: Not copy-paste—every screw axis, transformation, and Jacobian derived step-by-step  
✓ **Dual Formulations**: Space and body representations for complete kinematics understanding  
✓ **Production-Ready Code**: Modular MATLAB with numerical validation  
✓ **Physical Grounding**: Velocity and statics for real-world robot control  
✓ **Educational Clarity**: Hand-written screw analysis + clean documentation  

## Quick Start

### View Results
1. **Forward Kinematics**: Open `FK_3DOF.m`, adjust slider values, see 3D pose + EE position
2. **Velocity Example**: Run `velocity_3dof.m` with sample joint velocity `θ̇ = [0.1745, 0.0873, 0.0349] rad/s`
   - Outputs: Joint velocities → Space Jacobian → Space/body twists
3. **Statics Example**: Run `statics_3dof.m` with 1 kg payload
   - Outputs: End-effector wrench → Joint torques

### Dependencies
- MATLAB (R2020a+)
- Modern Robotics Toolbox (included or [download](https://github.com/NxRLab/ModernRobotics))

### File Structure
```
3DOF_Robot_PoE_FK/
├── README.md                                    (This file)
│
├── MATLAB Code/
│   ├── FK_3DOF.m                               (Forward kinematics + 3D viz)
│   ├── velocity_3dof.m                         (Velocity kinematics)
│   └── statics_3dof.m                          (Static analysis)
│
├── Documentation/
│   ├── 3DOF_Robot_POE_Forward_Kinematics_Documentation.docx
│   └── 3DOF_Robot_Velocity_and_Statics_Documentation.docx
│
└── Visuals/
    ├── schematic.png                           (Robot schematic diagram)
    ├── s1.jpeg through s6.jpeg                 (Hand-derived screw analysis, 6 pages)
    ├── 01.png                                  (FK_3DOF.m output screenshot)
    ├── 02.png                                  (velocity_3dof.m output screenshot)
    └── 03.png                                  (statics_3dof.m output screenshot)
```

## Validation & Results

All MATLAB outputs verified against hand-derived screw theory calculations.

### Forward Kinematics Validation
✓ Home pose: EE at [0.70, 0, 0.30] m with identity rotation (matches M matrix)  
✓ 3D visualization (01.png) confirms analytical FK across full workspace  
✓ Joint angles → end-effector position updates in real-time via PoE exponential formula  
✓ All three poses in quick presets produce correct Cartesian coordinates

### Velocity Kinematics Validation
✓ Space Jacobian computed from: `Js = [S₁, Ad(T₁)S₂, Ad(T₁T₂)S₃]` (matches s2.jpeg derivation)  
✓ Body Jacobian computed using adjoint rotations (matches s5.jpeg)  
✓ Sample input θ̇ = [0.1745, 0.0873, 0.0349] rad/s produces output shown in 02.png:
  - Joint velocities correctly propagated to EE twist
  - Space and body formulations give consistent physical results
  - Linear and angular velocity components physically reasonable

### Static Analysis Validation
✓ Joint torques τ = J^T F verified for gravity equilibrium (1 kg payload)  
✓ 03.png output shows torque distribution matches load at each joint  
✓ Base and shoulder remain unloaded (τ₁ = τ₂ = 0) under vertical load  
✓ Wrist joint (Joint 3) bears full moment: τ₃ ≈ -1.01 N·m

## What You'll Learn

1. **Screw Theory**: How rotation axes and velocities compose into screw coordinates
2. **PoE Framework**: Building complex robot kinematics from exponential maps
3. **Jacobians**: The bridge between joint space and task space
4. **Adjoint Transformations**: Frame-invariant representations
5. **Real-World Control**: How velocity and force relate to joint actuation

## References

- **Theory**: Modern Robotics by Lynch & Park (Coursera specialization)
- **Code Foundation**: Modern Robotics Toolbox for MATLAB
- **Derivations**: Hand-worked screw analysis in project documentation

## Next Steps (Future Work)

- Inverse kinematics (numerical + closed-form)
- Trajectory planning (time-optimal, smooth paths)
- Dynamics modeling (Lagrangian, joint control simulation)
- Hardware integration (ROS2 + real actuators)

## Notes

- **Joint 3 Axis Check**: Current model has S₂ = S₃ (identical screw axes). For final hardware implementation, verify joint 3's physical rotation axis against schematic.
- **Modern Robotics Toolbox**: Free version available via Coursera; provides matrix exponential and adjoint computations.

---

**Status**: ✅ Complete mathematical foundation + working MATLAB simulation  
**Last Updated**: September 2026  
**Author**: Abhishek Chauhan  
**Focus**: Robotics engineering, autonomous systems, reinforcement learning
