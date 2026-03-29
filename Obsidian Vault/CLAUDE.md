# Ball Balancing System — Equation Vault

This folder contains the mathematical derivation documents for a ball-on-table
balancing system. When working here, read the relevant document(s) before making
any changes or additions to ensure notation and conventions stay consistent.

---

## Project Documents

| File | Contents |
|------|----------|
| `Ball Dynamics Equations - Ball Balancing System.md` | Equations of motion for the ball (contact mode, free flight, bouncing) |
| `Table Dynamics Equations - Ball Balancing System.md` | Inverse and forward kinematics of the 3-arm table mechanism |

---

## System Overview

A ball rests on a flat table that is actuated by three servo-driven arms arranged
120° apart. The system has two coupled subsystems:

**Table** (actuator side): Three arms each with a lower link (servo-driven, length
$L_1$) and upper link (passive, length $L_2$). Ball-and-socket joints at both
ends of each upper link. Ground mounting radius $R_g$, table mounting radius $R_t$.

**Ball** (plant side): A sphere of mass $m$ and radius $r$ resting on the table
surface, subject to gravity, normal force, friction, and (when airborne) free
flight under gravity alone.

The information flows as:

```
servo angles (α₁, α₂, α₃)
        │ forward kinematics (numerical, Newton-Raphson)
        ▼
table pose (φ, θ, z_t)   ← the bridge between the two documents
        │ ball dynamics (contact or free-flight mode)
        ▼
ball position (x_b, y_b, z_b)
```

For control, the inverse direction is used:
```
ball position → control law → desired pose → inverse kinematics → servo commands
```

---

## Shared Notation and Conventions

These must be respected across both documents and any future additions.

### Coordinate Frame

- **Inertial frame $\{I\}$**: origin at center of ground mounting circle; $Z$ up
- **Table frame $\{T\}$**: origin at table center, rotates with the table
- $X$ axis points toward arm 1's ground point; $Y$ is 90° counterclockwise

### Rotation Convention

Rotation from table frame to inertial frame is $R = R_y(\theta) \cdot R_x(\varphi)$
(pitch applied after roll, i.e. pitch is about the world y-axis):

$$R = \begin{bmatrix}
\cos\theta & 0 & \sin\theta \\
\sin\theta\sin\varphi & \cos\varphi & -\cos\theta\sin\varphi \\
-\sin\theta\cos\varphi & \sin\varphi & \cos\theta\cos\varphi
\end{bmatrix}$$

### Table State Variables

| Symbol | Meaning |
|--------|---------|
| $\varphi(t)$ | Roll angle — rotation about x-axis |
| $\theta(t)$ | Pitch angle — rotation about y-axis |
| $z_t(t)$ | Vertical position of table center |

### Ball State Variables

| Symbol | Meaning |
|--------|---------|
| $(x_b, y_b, z_b)$ | Ball position in inertial frame |
| $m$ | Ball mass |
| $r$ | Ball radius |
| $e$ | Coefficient of restitution |
| $\mu$ | Coefficient of friction |
| $N$ | Normal force magnitude |

### Arm / Mechanism Variables

| Symbol | Meaning |
|--------|---------|
| $L_1$ | Lower link length (servo to elbow) |
| $L_2$ | Upper link length (elbow to table point) |
| $R_g$ | Ground mounting radius |
| $R_t$ | Table mounting radius |
| $\psi_i = \frac{2\pi(i-1)}{3}$ | Angular position of arm $i$, $i \in \{1,2,3\}$ |
| $\alpha_i$ | Servo angle of arm $i$, measured from horizontal |
| $\mathbf{G}_i$ | Ground attachment point of arm $i$ |
| $\mathbf{T}_i$ | Table attachment point of arm $i$ (in inertial frame) |
| $\mathbf{E}_i$ | Elbow position of arm $i$ (in inertial frame) |

### Small-Angle Approximation

When used: $\sin\varphi \approx \varphi$, $\cos\varphi \approx 1$, $\sin\theta \approx \theta$,
$\cos\theta \approx 1$. Cross terms $\varphi\theta$ are second-order and dropped.
Always derive exact form first, then apply approximation as a separate step.

---

## Key Results (Quick Reference)

### Ball Dynamics — Contact Mode
$$\ddot{x}_b = -g\theta - \dot\theta^2 x_b - \ddot\theta z_b - \tfrac{\mu N}{m}\,\text{sgn}(v_{\text{rel},x})$$
$$\ddot{y}_b = g\varphi - \dot\varphi^2 y_b - \ddot\varphi z_b - \tfrac{\mu N}{m}\,\text{sgn}(v_{\text{rel},y})$$
$$\ddot{z}_b = -g + \tfrac{N}{m} + \ddot\varphi y_b - \ddot\theta x_b - \dot\varphi^2 z_b - \dot\theta^2 z_b$$

Contact constraint (small angles): $z_b = z_t + r + x_b\theta - y_b\varphi$

### Ball Dynamics — Free Flight
$$\ddot{x}_b = 0, \quad \ddot{y}_b = 0, \quad \ddot{z}_b = -g$$

### Bounce (coefficient of restitution $e$)
$$\dot{z}_b^{(\text{after})} = (1+e)\dot{z}_{\text{table contact}} - e\,\dot{z}_b^{(\text{before})}$$

### Inverse Kinematics (exact, per arm)
$$\alpha_i = \underbrace{\text{atan2}(d_{z,i},\, d_{r,i})}_{\phi_i} \pm \arccos\!\left(\frac{d_{r,i}^2 + d_{z,i}^2 + L_1^2 - L_2^2}{2L_1\sqrt{d_{r,i}^2 + d_{z,i}^2}}\right)$$

### Small-Angle Pose from Arm Heights
$$z_t = \frac{d_{z,1}+d_{z,2}+d_{z,3}}{3}, \quad
\theta \approx \frac{d_{z,2}+d_{z,3}-2d_{z,1}}{3R_t}, \quad
\varphi \approx \frac{d_{z,2}-d_{z,3}}{\sqrt{3}\,R_t}$$

---

## Document Style Guide

When adding new sections or creating new equation documents in this vault, follow
the conventions established in these two files:

- **Structure**: System description → coordinate frames → kinematics → forces/constraints → equations of motion → approximations → summary
- **Equations**: `$$...$$` display math always; `\boxed{...}` for final results; `align` with `&=` for multi-step derivations
- **Vectors**: bold (`\mathbf{F}`), unit vectors with hat (`\hat{\mathbf{r}}`)
- **Matrices**: `\begin{bmatrix}...\end{bmatrix}`
- **Derivation rhythm**: state the law/constraint → show every algebraic step → box the result → give physical interpretation → ASCII diagram if geometry is involved
- **Sections**: `##` major topic, `###` subtopic, `####` sub-step; `---` between major sections
- **Never skip** a non-obvious algebraic step; always explain sign conventions
- **Limiting cases**: after a key result, verify it reduces correctly when one variable is set to zero
