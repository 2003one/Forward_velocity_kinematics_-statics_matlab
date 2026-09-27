# 3-DOF Robot Arm: Product of Exponentials Forward Kinematics

A complete robotics engineering project spanning mathematical derivation through MATLAB simulation. Implements forward kinematics, velocity kinematics, and statics analysis for a 3-DOF manipulator using the **Product of Exponentials (PoE)** framework and **screw theory**.

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
├── FK_3DOF.m
├── velocity_3dof.m
├── statics_3dof.m
├── 3DOF_Robot_POE_Forward_Kinematics_Documentation.docx
├── 3DOF_Robot_Velocity_and_Statics_Documentation.docx
└── README.md
```

## Validation & Results

### Forward Kinematics
- Home pose verified: EE at [0.70, 0, 0.30] with identity rotation
- 3D visualization matches analytical results
- Joint angles → end-effector position & orientation (continuous update)

### Velocity Kinematics
Example output (θ̇ = [0.1745, 0.0873, 0.0349] rad/s):
```
Joint Velocities:     [0.1745, 0.0873, 0.0349] rad/s
Space Twist (EE):     ω = [0.1058, 0.0611, 0.1745] rad/s
                      v = [-0.0180, 0.0312, 0.0030] m/s
Body Twist (EE):      ω = [0.0873, 0.0873, 0.0349] rad/s
                      v = [0, 0.30, 0] m/s
```

### Static Analysis
1 kg payload (9.81 N gravity):
```
Payload Force:        Fz = -9.81 N
Joint Torques:        τ₁ = 0 N·m
                      τ₂ = 0 N·m
                      τ₃ = -1.01 N·m (wrist bears load)
```

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
