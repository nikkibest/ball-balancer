## System Description

The system consists of:
- **Table**: Can roll around its x-axis $\varphi(t)$, pitch around its y-axis $\theta(t)$, and translate vertically along its z-axis $z_t(t)$
- **Ball**: Rests on the table, can move freely in x, y, and z directions, and can bounce on the table surface

The ball has:
- Mass: $m$
- Radius: $r$
- Coefficient of restitution: $e$ (for bouncing)
- Coefficient of friction: $\mu$

---

## Coordinate Frames

1. **Inertial frame $\{I\}$**: Fixed reference frame with axes $(X, Y, Z)$, where $Z$ points upward
2. **Table frame $\{T\}$**: Attached to the table center, rotates with the table
3. **Ball position**: $(x_b, y_b, z_b)$ in inertial frame

---

## Table Kinematics

### Degrees of Freedom

The table has 3 degrees of freedom:
- $\varphi(t)$ = roll angle (rotation about x-axis)
- $\theta(t)$ = pitch angle (rotation about y-axis)
- $z_t(t)$ = vertical position of table center

### Rotation Matrices

Rotation about x-axis (roll):
$$
R_x(\varphi) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos(\varphi) & -\sin(\varphi) \\
0 & \sin(\varphi) & \cos(\varphi)
\end{bmatrix}
$$

Rotation about y-axis (pitch):
$$
R_y(\theta) = \begin{bmatrix}
\cos(\theta) & 0 & \sin(\theta) \\
0 & 1 & 0 \\
-\sin(\theta) & 0 & \cos(\theta)
\end{bmatrix}
$$

Combined rotation matrix from table frame to inertial frame:
$$
R = R_y(\theta) \cdot R_x(\varphi) = \begin{bmatrix}
\cos(\theta) & 0 & \sin(\theta) \\
\sin(\theta)\sin(\varphi) & \cos(\varphi) & -\cos(\theta)\sin(\varphi) \\
-\sin(\theta)\cos(\varphi) & \sin(\varphi) & \cos(\theta)\cos(\varphi)
\end{bmatrix}
$$

### Angular Velocity

The angular velocity vector of the table in the inertial frame:
$$
\boldsymbol{\omega}_{\text{table}} = \begin{bmatrix}
\dot{\varphi} \\
\dot{\theta} \\
0
\end{bmatrix}
$$

---

## Ball Kinematics

### Position, Velocity, and Acceleration

Ball position, velocity and acceleration in inertial frame:
$$
\mathbf{r}_b = \begin{bmatrix}
x_b \\
y_b \\
z_b
\end{bmatrix} , 
\mathbf{v}_b = \begin{bmatrix}
\dot{x}_b \\
\dot{y}_b \\
\dot{z}_b
\end{bmatrix}
, \mathbf{a}_b = \begin{bmatrix}
\ddot{x}_b \\
\ddot{y}_b \\
\ddot{z}_b
\end{bmatrix}
$$

---

## Forces Acting on the Ball

When the ball is in contact with the table:

### 1. Gravitational Force
$$
\mathbf{F}_g = \begin{bmatrix}
0 \\
0 \\
-mg
\end{bmatrix}
$$

### 2. Normal Force

The normal direction (perpendicular to table surface, pointing upward):
$$
\hat{\mathbf{n}} = R \cdot \begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}  = \begin{bmatrix}
\cos(\theta) & 0 & \sin(\theta) \\
\sin(\theta)\sin(\varphi) & \cos(\varphi) & -\cos(\theta)\sin(\varphi) \\
-\sin(\theta)\cos(\varphi) & \sin(\varphi) & \cos(\theta)\cos(\varphi)
\end{bmatrix} \cdot \begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix} = \begin{bmatrix}
\sin(\theta) \\
-\sin(\varphi)\cos(\theta) \\
\cos(\varphi)\cos(\theta)
\end{bmatrix}
$$

Normal force:
$$
\mathbf{F}_N = N \cdot \hat{\mathbf{n}} = N \begin{bmatrix}
\sin(\theta) \\
-\sin(\varphi)\cos(\theta) \\
\cos(\varphi)\cos(\theta)
\end{bmatrix}
$$

### 3. Friction Force

Friction opposes the relative motion of the ball with respect to the table surface:
$$
\mathbf{F}_{\text{friction}} = -\mu N \cdot \hat{\mathbf{v}}_{\text{rel}}
$$

where $\hat{\mathbf{v}}_{\text{rel}}$ is the unit vector in the direction of relative velocity on the table surface.

---

## Velocity of Contact Point on Table

The contact point on the table surface moves due to table translation and rotation.

### Table Origin Velocity
$$
\mathbf{v}_{\text{table,origin}} = \begin{bmatrix}
0 \\
0 \\
\dot{z}_t
\end{bmatrix}
$$

### Velocity Due to Rotation

The velocity of a point on the table at position $\mathbf{r}_{\text{contact}}$ relative to the table origin:
$$
\mathbf{v}_{\text{rotation}} = \boldsymbol{\omega}_{\text{table}} \times \mathbf{r}_{\text{contact}}
$$

For the ball contact point at approximately $(x_b, y_b, z_t)$:
$$
\boldsymbol{\omega}_{\text{table}} \times \mathbf{r}_{\text{contact}} = \begin{bmatrix}
\dot{\varphi} \\
\dot{\theta} \\
0
\end{bmatrix} \times \begin{bmatrix}
x_b \\
y_b \\
z_b - z_t
\end{bmatrix} = \begin{vmatrix}
i & j & k \\
\dot{\varphi} & \dot{\theta} & 0 \\
x_b & y_b & z_b - z_t
\end{vmatrix}$$
$$ = i\begin{vmatrix}
\dot{\theta} & 0 \\ y_b & z_b - z_t
\end{vmatrix} - j\begin{vmatrix}
\dot{\varphi} & 0 \\ x_b & z_b - z_t
\end{vmatrix} + k\begin{vmatrix}
\dot{\varphi} & \dot{\theta} & \\ x_b & y_b
\end{vmatrix}$$
$$= \begin{bmatrix}
\dot{\theta}(z_b - z_t) \\
-\dot{\varphi}(z_b - z_t) \\
\dot{\varphi} y_b - \dot{\theta} x_b
\end{bmatrix}
$$

### Total Contact Point Velocity
$$
\mathbf{v}_{\text{contact}} = \mathbf{v}_{\text{table,origin}} + \mathbf{v}_{\text{rotation}} = \begin{bmatrix}
\dot{\theta}(z_b - z_t) \\
-\dot{\varphi}(z_b - z_t) \\
\dot{z}_t + \dot{\varphi} y_b - \dot{\theta} x_b
\end{bmatrix}
$$

### Relative Velocity

Ball velocity relative to the table surface:
$$
\mathbf{v}_{\text{rel}} = \mathbf{v}_b - \mathbf{v}_{\text{contact}}
$$

In component form:
$$
\begin{align}
v_{\text{rel},x} &= \dot{x}_b - \dot{\theta}(z_b - z_t) \\
v_{\text{rel},y} &= \dot{y}_b + \dot{\varphi}(z_b - z_t) \\
v_{\text{rel},z} &= \dot{z}_b - \dot{z}_t - \dot{\varphi} y_b + \dot{\theta} x_b
\end{align}
$$

---

## Newton's Second Law

Applying Newton's second law in the inertial frame:
$$
m \mathbf{a}_b = \mathbf{F}_g + \mathbf{F}_N + \mathbf{F}_{\text{friction}} = \begin{bmatrix}
0 \\
0 \\
-mg
\end{bmatrix} + N \begin{bmatrix}
\sin(\theta) \\
-\sin(\varphi)\cos(\theta) \\
\cos(\varphi)\cos(\theta)
\end{bmatrix}  -\mu N \cdot \hat{\mathbf{v}}_{\text{rel}}
$$

### Component-wise (Initial Form)

**X-direction:**
$$
m \ddot{x}_b = N\sin(\theta) - \mu N \cdot \text{sgn}(v_{\text{rel},x})
$$
**Y-direction:**
$$
m \ddot{y}_b = -N\sin(\varphi)\cos(\theta) - \mu N \cdot \text{sgn}(v_{\text{rel},y})
$$
**Z-direction:**
$$
m \ddot{z}_b = -mg + N\cos(\varphi)\cos(\theta)
$$

---

## Inertial Forces (Non-Inertial Frame Effects)

### Why Do We Need Inertial Forces?

The ball is being analyzed in an **inertial frame** (fixed in space), but the table underneath it is **rotating and accelerating**. When the table rotates, the ball experiences additional accelerations beyond just gravity and contact forces. These are called **fictitious forces** or **pseudo-forces** because they arise from the non-inertial motion of the reference frame.

There are three main types of inertial accelerations:
1. **Centrifugal acceleration**: Due to constant rotation (depends on $\omega^2$)
2. **Euler acceleration**: Due to angular acceleration (depends on $\dot{\omega} = \alpha$)
3. **Coriolis acceleration**: Due to motion in a rotating frame (depends on $\omega \times v$)

In our case, we'll focus on centrifugal and Euler accelerations. The Coriolis effect is already captured in the relative velocity calculations.

---

### General Formula for Acceleration in Rotating Frames

When observing from an inertial frame, the acceleration of a point on a rotating body is:
$$
\mathbf{a}_{\text{inertial}} = \mathbf{a}_{\text{rotating}} + \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r}) + \boldsymbol{\alpha} \times \mathbf{r} + 2\boldsymbol{\omega} \times \mathbf{v}_{\text{rel}}
$$

where:
- $\mathbf{a}_{\text{rotating}}$ = acceleration in the rotating frame
- $\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})$ = **centrifugal acceleration**
- $\boldsymbol{\alpha} \times \mathbf{r}$ = **Euler (tangential) acceleration**
- $2\boldsymbol{\omega} \times \mathbf{v}_{\text{rel}}$ = **Coriolis acceleration**

---

### Centrifugal Acceleration

The centrifugal acceleration is the outward acceleration experienced by an object in a rotating frame:
$$
\mathbf{a}_{\text{centrifugal}} = \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})
$$

This is a **double cross product**. Let's compute it step by step.

#### Step 1: Define the angular velocity and position

The table rotates with angular velocity and the ball position relative to the table rotation center:
$$
\boldsymbol{\omega}_{\text{table}} = \begin{bmatrix}
\dot{\varphi} \\
\dot{\theta} \\
0
\end{bmatrix}
\quad , \quad
\mathbf{r}_b = \begin{bmatrix}
x_b \\
y_b \\
z_b
\end{bmatrix}
$$

#### Step 2: Compute $\boldsymbol{\omega} \times \mathbf{r}$ (first cross product)

Using the cross product formula $\mathbf{a} \times \mathbf{b} = \begin{bmatrix} a_y b_z - a_z b_y \\ a_z b_x - a_x b_z \\ a_x b_y - a_y b_x \end{bmatrix}$:

$$
\boldsymbol{\omega} \times \mathbf{r} = \begin{bmatrix}
\dot{\varphi} \\
\dot{\theta} \\
0
\end{bmatrix} \times \begin{bmatrix}
x_b \\
y_b \\
z_b
\end{bmatrix}
= \begin{bmatrix}
\dot{\theta}z_b \\
-\dot{\varphi}z_b \\
\dot{\varphi} y_b - \dot{\theta} x_b
\end{bmatrix}
$$
#### Step 3: Compute $\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})$ (second cross product)

Now we compute:
$$
\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r}) = \begin{bmatrix}
\dot{\varphi} \\
\dot{\theta} \\
0
\end{bmatrix} \times \begin{bmatrix}
\dot{\theta} z_b \\
-\dot{\varphi} z_b \\
\dot{\varphi} y_b - \dot{\theta} x_b
\end{bmatrix}
$$

Computing each component:
$$
\begin{align}
[\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})]_x &= \dot{\theta} \cdot (\dot{\varphi} y_b - \dot{\theta} x_b) - 0 \cdot (-\dot{\varphi} z_b) \\
&= \dot{\theta}\dot{\varphi} y_b - \dot{\theta}^2 x_b
\end{align}
$$

$$
\begin{align}
[\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})]_y &= 0 \cdot (\dot{\theta} z_b) - \dot{\varphi} \cdot (\dot{\varphi} y_b - \dot{\theta} x_b) \\
&= -\dot{\varphi}^2 y_b + \dot{\varphi}\dot{\theta} x_b
\end{align}
$$

$$
\begin{align}
[\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})]_z &= \dot{\varphi} \cdot (-\dot{\varphi} z_b) - \dot{\theta} \cdot (\dot{\theta} z_b) \\
&= -\dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
\end{align}
$$

Therefore:
$$
\mathbf{a}_{\text{centrifugal}} = \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r}) = \begin{bmatrix}
\dot{\theta}\dot{\varphi} y_b - \dot{\theta}^2 x_b \\
-\dot{\varphi}^2 y_b + \dot{\varphi}\dot{\theta} x_b \\
-\dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
\end{bmatrix}
$$

#### Simplification for Decoupled Rotations

In practice, if we assume the roll and pitch don't happen simultaneously at high rates, we can neglect the cross terms $\dot{\varphi}\dot{\theta}$ and separate the effects.

The centrifugal acceleration has the sign convention where it points **outward** from the axis of rotation. However, when we write the equations of motion, we account for the fact that this acceleration must be subtracted from the ball's motion (it represents the non-inertial nature of the frame).

From the full expression, setting $\dot{\theta} = 0$ gives the **centrifugal acceleration due to roll only**:
$$
\mathbf{a}_{\text{centrifugal},\varphi} = \begin{bmatrix}
0 \\
-\dot{\varphi}^2 y_b \\
-\dot{\varphi}^2 z_b
\end{bmatrix}
$$

Setting $\dot{\varphi} = 0$ gives the **centrifugal acceleration due to pitch only**:
$$
\mathbf{a}_{\text{centrifugal},\theta} = \begin{bmatrix}
-\dot{\theta}^2 x_b \\
0 \\
-\dot{\theta}^2 z_b
\end{bmatrix}
$$

Note: The negative signs indicate that when we write the equations of motion $m\ddot{x}_b = F_x + F_{\text{inertial}}$, these terms will subtract from the acceleration (or equivalently, we add $+m\dot{\theta}^2 x_b$ to the force side).

---

### Euler Acceleration (Tangential Acceleration)

When the angular velocity is **changing** (angular acceleration $\boldsymbol{\alpha} = \dot{\boldsymbol{\omega}}$), there's an additional tangential acceleration:
$$
\mathbf{a}_{\text{Euler}} = \boldsymbol{\alpha} \times \mathbf{r}
$$

This represents the linear acceleration due to the **speeding up or slowing down** of the rotation.

#### Roll Acceleration $\ddot{\varphi}$

The angular acceleration vector for roll:
$$
\boldsymbol{\alpha}_{\varphi} = \begin{bmatrix}
\ddot{\varphi} \\
0 \\
0
\end{bmatrix}
$$

The cross product with the ball position:
$$
\boldsymbol{\alpha}_{\varphi} \times \mathbf{r}_b = \begin{bmatrix}
\ddot{\varphi} \\
0 \\
0
\end{bmatrix} \times \begin{bmatrix}
x_b \\
y_b \\
z_b
\end{bmatrix}
$$

Computing each component:
$$
\begin{align}
(\boldsymbol{\alpha}_{\varphi} \times \mathbf{r})_x &= 0 \cdot z_b - 0 \cdot y_b = 0 \\
(\boldsymbol{\alpha}_{\varphi} \times \mathbf{r})_y &= 0 \cdot x_b - \ddot{\varphi} \cdot z_b = -\ddot{\varphi} z_b \\
(\boldsymbol{\alpha}_{\varphi} \times \mathbf{r})_z &= \ddot{\varphi} \cdot y_b - 0 \cdot x_b = \ddot{\varphi} y_b
\end{align}
$$

Therefore:
$$
\mathbf{a}_{\text{Euler},\varphi} = \begin{bmatrix}
0 \\
-\ddot{\varphi} z_b \\
\ddot{\varphi} y_b
\end{bmatrix}
$$

#### Pitch Acceleration $\ddot{\theta}$

The angular acceleration vector for pitch:
$$
\boldsymbol{\alpha}_{\theta} = \begin{bmatrix}
0 \\
\ddot{\theta} \\
0
\end{bmatrix}
$$

The cross product:
$$
\boldsymbol{\alpha}_{\theta} \times \mathbf{r}_b = \begin{bmatrix}
0 \\
\ddot{\theta} \\
0
\end{bmatrix} \times \begin{bmatrix}
x_b \\
y_b \\
z_b
\end{bmatrix}
$$

Computing:
$$
\begin{align}
(\boldsymbol{\alpha}_{\theta} \times \mathbf{r})_x &= \ddot{\theta} \cdot z_b - 0 \cdot y_b = \ddot{\theta} z_b \\
(\boldsymbol{\alpha}_{\theta} \times \mathbf{r})_y &= 0 \cdot x_b - 0 \cdot z_b = 0 \\
(\boldsymbol{\alpha}_{\theta} \times \mathbf{r})_z &= 0 \cdot y_b - \ddot{\theta} \cdot x_b = -\ddot{\theta} x_b
\end{align}
$$

Therefore:
$$
\mathbf{a}_{\text{Euler},\theta} = \begin{bmatrix}
\ddot{\theta} z_b \\
0 \\
-\ddot{\theta} x_b
\end{bmatrix}
$$

---

### Physical Interpretation

**Centrifugal acceleration**: Imagine you're on a merry-go-round spinning at constant speed. You feel pushed outward - that's centrifugal acceleration. It's proportional to $\omega^2$ (faster spin = stronger outward push) and to distance from the axis.

**Euler acceleration**: Now imagine the merry-go-round is speeding up or slowing down. You feel a tangential push (forward if speeding up, backward if slowing down). That's Euler acceleration. It's proportional to $\alpha$ (angular acceleration) and distance from the axis.

In our ball-on-table system:
- When the table tilts at constant rate $\dot{\varphi}$, the ball experiences centrifugal forces pushing it away from the rotation axis
- When the table's tilt is accelerating $\ddot{\varphi} \neq 0$, the ball experiences tangential (Euler) forces

---

### Summary of Inertial Accelerations

**Total centrifugal acceleration** (simplified, neglecting cross terms):
$$
\mathbf{a}_{\text{centrifugal}} = \begin{bmatrix}
\dot{\theta}^2 x_b \\
\dot{\varphi}^2 y_b \\
\dot{\varphi}^2 z_b + \dot{\theta}^2 z_b
\end{bmatrix}
$$

**Total Euler acceleration**:
$$
\mathbf{a}_{\text{Euler}} = \begin{bmatrix}
\ddot{\theta} z_b \\
-\ddot{\varphi} z_b \\
\ddot{\varphi} y_b - \ddot{\theta} x_b
\end{bmatrix}
$$

These accelerations must be **subtracted** from the ball's motion (or equivalently, the corresponding inertial forces $-m\mathbf{a}_{\text{inertial}}$ must be added to the force balance), because they represent the acceleration of the reference frame, not forces acting on the ball.

---

## Complete Equations of Motion (Contact Mode)

Now we combine all the effects to get the complete equations. We need to:
1. Start with Newton's 2nd law in component form (from earlier)
2. Add the inertial accelerations (centrifugal and Euler)
3. Simplify using small angle approximations

### Derivation Step-by-Step

#### Starting Point: Newton's 2nd Law Components

From earlier, we had:
$$
\begin{align}
m \ddot{x}_b &= N\sin(\theta) - \mu N \cdot \text{sgn}(v_{\text{rel},x}) \\
m \ddot{y}_b &= -N\sin(\varphi)\cos(\theta) - \mu N \cdot \text{sgn}(v_{\text{rel},y}) \\
m \ddot{z}_b &= -mg + N\cos(\varphi)\cos(\theta)
\end{align}
$$

These equations only account for gravity, normal force, and friction. But we're missing the **inertial effects** from the rotating table!

#### Adding Inertial Accelerations

The **actual acceleration** of the ball in the inertial frame must account for:
- The forces acting on it (gravity, normal, friction)
- The accelerations due to the rotating/accelerating reference frame

The corrected force balance becomes:
$$
m \ddot{x}_b = F_x - m \cdot a_{\text{inertial},x}
$$

where $a_{\text{inertial},x}$ includes both centrifugal and Euler contributions.

#### X-Direction (Detailed)

Forces in x-direction:
$$
F_x = N\sin(\theta) - \mu N \cdot \text{sgn}(v_{\text{rel},x})
$$

Inertial accelerations in x-direction:
$$
a_{\text{inertial},x} = \underbrace{\dot{\theta}^2 x_b}_{\text{centrifugal}} + \underbrace{\ddot{\theta} z_b}_{\text{Euler}}
$$

The **net gravitational effect** on a tilted table causes the ball to roll downhill. When $\theta > 0$ (table pitches upward in positive x-direction), gravity creates a force component:
$$
F_{x,\text{gravity}} = -mg\sin(\theta)
$$

Combining all forces and inertial effects:
$$
m\ddot{x}_b = -mg\sin(\theta) - m\dot{\theta}^2 x_b - m\ddot{\theta} z_b - \mu N \cdot \text{sgn}(v_{\text{rel},x})
$$

#### Y-Direction (Detailed)

Gravity on a tilted table creates a force component:
$$
F_{y,\text{gravity}} = mg\sin(\varphi)\cos(\theta) \approx mg\varphi
$$

Inertial accelerations in y-direction (from our earlier derivation):
- Centrifugal: $-\dot{\varphi}^2 y_b$
- Euler: $-\ddot{\varphi} z_b$

These accelerations must be subtracted from the left-hand side (equivalently, we add the corresponding forces to the right-hand side):
$$
m\ddot{y}_b = mg\varphi - m\dot{\varphi}^2 y_b - m\ddot{\varphi} z_b - \mu N \cdot \text{sgn}(v_{\text{rel},y})
$$

#### Z-Direction (Detailed)

Forces in z-direction:
$$
F_z = -mg + N\cos(\varphi)\cos(\theta)
$$

Inertial accelerations in z-direction:
$$
a_{\text{inertial},z} = \underbrace{(\dot{\varphi}^2 z_b + \dot{\theta}^2 z_b)}_{\text{centrifugal}} + \underbrace{(\ddot{\varphi} y_b - \ddot{\theta} x_b)}_{\text{Euler}}
$$

Using the small angle approximation $\cos(\varphi)\cos(\theta) \approx 1$, we get:
$$
m\ddot{z}_b = -mg + N - m\dot{\varphi}^2 z_b - m\dot{\theta}^2 z_b + m\ddot{\varphi} y_b - m\ddot{\theta} x_b
$$

---

### Final Equations with Small Angle Approximations

Using $\sin(\theta) \approx \theta$, $\sin(\varphi) \approx \varphi$, $\cos(\theta) \approx 1$, $\cos(\varphi) \approx 1$:

### X-Direction
$$
m\ddot{x}_b = -mg\sin(\theta) - m\dot{\theta}^2 x_b - m\ddot{\theta} z_b - \mu N \cdot \text{sgn}(v_{\text{rel},x})
$$

For small angles:
$$
\ddot{x}_b = -g\theta - \dot{\theta}^2 x_b - \ddot{\theta} z_b - \frac{\mu N}{m} \cdot \text{sgn}(v_{\text{rel},x})
$$

### Y-Direction
$$
m\ddot{y}_b = mg\sin(\varphi)\cos(\theta) - m\dot{\varphi}^2 y_b - m\ddot{\varphi} z_b - \mu N \cdot \text{sgn}(v_{\text{rel},y})
$$

For small angles:
$$
\ddot{y}_b = g\varphi - \dot{\varphi}^2 y_b - \ddot{\varphi} z_b - \frac{\mu N}{m} \cdot \text{sgn}(v_{\text{rel},y})
$$

### Z-Direction
$$
m\ddot{z}_b = -mg + N\cos(\varphi)\cos(\theta) + m\ddot{\varphi} y_b - m\ddot{\theta} x_b - m\dot{\varphi}^2 z_b - m\dot{\theta}^2 z_b
$$

For small angles:
$$
\ddot{z}_b = -g + \frac{N}{m} + \ddot{\varphi} y_b - \ddot{\theta} x_b - \dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
$$

---

## Normal Force Calculation

The normal force is determined by the constraint that the ball must remain in contact with the table surface. We'll derive it by using the contact constraint and the equation of motion in the z-direction.

### Contact Constraint

When the ball is in contact with the table, the ball's center position must satisfy:
$$
z_b = z_{\text{table surface}}(x_b, y_b, t) + r
$$

where $r$ is the ball radius.

For a **flat table** that is tilted, the table surface height at position $(x_b, y_b)$ is:
$$
z_{\text{table surface}}(x_b, y_b, t) = z_t(t) + x_b \tan(\theta) - y_b \tan(\varphi)\cos(\theta)
$$

**Explanation:**
- When the table pitches by angle $\theta$, a point at distance $x_b$ from the center rises by $x_b \tan(\theta)$
- When the table rolls by angle $\varphi$, a point at distance $y_b$ from the center drops by $y_b \tan(\varphi)\cos(\theta)$

For small angles, $\tan(\theta) \approx \theta$ and $\tan(\varphi) \approx \varphi$:
$$
z_b = z_t + r + x_b\theta - y_b\varphi
$$

### Taking Time Derivatives

To find the constraint on acceleration, we differentiate the constraint equation twice.

#### First derivative (velocity):
$$
\dot{z}_b = \dot{z}_t + \dot{x}_b\theta + x_b\dot{\theta} - \dot{y}_b\varphi - y_b\dot{\varphi}
$$

#### Second derivative (acceleration):
$$
\begin{align}
\ddot{z}_b &= \frac{d}{dt}\left(\dot{z}_t + \dot{x}_b\theta + x_b\dot{\theta} - \dot{y}_b\varphi - y_b\dot{\varphi}\right) \\
&= \ddot{z}_t + \ddot{x}_b\theta + \dot{x}_b\dot{\theta} + \dot{x}_b\dot{\theta} + x_b\ddot{\theta} \\
&\quad - \ddot{y}_b\varphi - \dot{y}_b\dot{\varphi} - \dot{y}_b\dot{\varphi} - y_b\ddot{\varphi} \\
&= \ddot{z}_t + \ddot{x}_b\theta + 2\dot{x}_b\dot{\theta} + x_b\ddot{\theta} - \ddot{y}_b\varphi - 2\dot{y}_b\dot{\varphi} - y_b\ddot{\varphi}
\end{align}
$$

This is the **constraint acceleration** - it tells us what $\ddot{z}_b$ must be if the ball stays in contact.

### Solving for Normal Force

From the Z-direction equation of motion (derived earlier):
$$
\ddot{z}_b = -g + \frac{N}{m} + \ddot{\varphi} y_b - \ddot{\theta} x_b - \dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
$$

We substitute the constraint acceleration into this equation:
$$
\ddot{z}_t + \ddot{x}_b\theta + 2\dot{x}_b\dot{\theta} + x_b\ddot{\theta} - \ddot{y}_b\varphi - 2\dot{y}_b\dot{\varphi} - y_b\ddot{\varphi} = -g + \frac{N}{m} + \ddot{\varphi} y_b - \ddot{\theta} x_b - \dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
$$

Now we rearrange to solve for $N$. Moving all terms except $N/m$ to the left side:
$$
\frac{N}{m} = g + \ddot{z}_t + \ddot{x}_b\theta + 2\dot{x}_b\dot{\theta} + x_b\ddot{\theta} - \ddot{y}_b\varphi - 2\dot{y}_b\dot{\varphi} - y_b\ddot{\varphi} - \ddot{\varphi} y_b + \ddot{\theta} x_b + \dot{\varphi}^2 z_b + \dot{\theta}^2 z_b
$$

Combining like terms:
- Terms with $x_b\ddot{\theta}$: $x_b\ddot{\theta} + \ddot{\theta} x_b = 2x_b\ddot{\theta}$
- Terms with $y_b\ddot{\varphi}$: $-y_b\ddot{\varphi} - \ddot{\varphi} y_b = -2y_b\ddot{\varphi}$

This gives:
$$
\frac{N}{m} = g + \ddot{z}_t + \ddot{x}_b\theta - \ddot{y}_b\varphi + 2\dot{x}_b\dot{\theta} - 2\dot{y}_b\dot{\varphi} + 2x_b\ddot{\theta} - 2y_b\ddot{\varphi} + \dot{\varphi}^2 z_b + \dot{\theta}^2 z_b
$$

**Important observation**: This expression contains $\ddot{x}_b$ and $\ddot{y}_b$, which themselves depend on $N$ through the equations of motion. This creates a **coupled system** that must be solved simultaneously.

### Simplified Approach: Neglecting Higher-Order Terms

For a **quasi-static approximation** or when the ball's lateral accelerations are small compared to gravity, we can simplify.

If we assume:
- The ball is approximately at rest on the table in x and y directions: $\ddot{x}_b \approx 0$, $\ddot{y}_b \approx 0$
- The table tilts slowly: $\dot{\varphi} \approx 0$, $\dot{\theta} \approx 0$
- The centrifugal terms are small: $\dot{\varphi}^2 z_b \approx 0$, $\dot{\theta}^2 z_b \approx 0$

Then the expression simplifies dramatically to:
$$
N \approx m\left[g + \ddot{z}_t + 2x_b\ddot{\theta} - 2y_b\ddot{\varphi}\right]
$$

However, this is overly simplified for a dynamic system.

### More Accurate Expression

For the general dynamic case, we should keep the key terms. Assuming small lateral velocities ($\dot{x}_b$, $\dot{y}_b$ small) but allowing accelerations, we get:

$$
\boxed{
N = m\left[g + \ddot{z}_t + \ddot{x}_b\theta - \ddot{y}_b\varphi + 2x_b\ddot{\theta} - 2y_b\ddot{\varphi} + \dot{\varphi}^2 z_b + \dot{\theta}^2 z_b\right]
}
$$

where:
- $g$ = gravitational acceleration
- $\ddot{z}_t$ = vertical acceleration of table
- $\ddot{x}_b\theta$ = coupling between ball's x-acceleration and table pitch
- $\ddot{y}_b\varphi$ = coupling between ball's y-acceleration and table roll
- $2x_b\ddot{\theta}$ = effect of angular acceleration of pitch on ball position
- $2y_b\ddot{\varphi}$ = effect of angular acceleration of roll on ball position
- $\dot{\varphi}^2 z_b + \dot{\theta}^2 z_b$ = centrifugal effects in vertical direction

### Important Notes

1. **This is an implicit equation**: Since $\ddot{x}_b$ and $\ddot{y}_b$ depend on $N$ (through the equations of motion), we have a coupled system that must be solved simultaneously.

2. **For simulation**: You would typically solve the system iteratively or use the equations of motion to substitute for $\ddot{x}_b$ and $\ddot{y}_b$, creating a more complex but fully explicit expression for $N$.

3. **Approximate explicit form**: If friction is small and we neglect the friction terms momentarily:
$$
N \approx \frac{m\left[g + \ddot{z}_t + 2x_b\ddot{\theta} - 2y_b\ddot{\varphi} + \dot{\varphi}^2(z_t + r) + \dot{\theta}^2(z_t + r)\right]}{1 - \theta^2 - \varphi^2}
$$

where we've substituted $z_b \approx z_t + r$ for small angles.

**Contact condition:**
- $N > 0$ → ball in contact with table
- $N \leq 0$ → ball loses contact, enters free flight

---

## Free Flight Dynamics

When the ball loses contact with the table ($N \leq 0$), it enters **free flight** mode. During free flight, the ball is only subject to gravity - there is no normal force and no friction force from the table.

### Condition for Free Flight

The ball enters free flight when:
$$
N \leq 0 \quad \text{or} \quad z_b > z_{\text{table surface}}(x_b, y_b, t) + r
$$

This occurs when:
1. The table is accelerating downward faster than gravity can pull the ball down
2. The table tilts/rotates so rapidly that centrifugal effects overcome the normal force
3. The ball bounces off the table

### Equations of Motion in Free Flight

During free flight, the ball behaves as a **projectile** under gravity alone. The equations are extremely simple:

$$
\boxed{
\begin{align}
\ddot{x}_b &= 0 \\
\ddot{y}_b &= 0 \\
\ddot{z}_b &= -g
\end{align}
}
$$

**Explanation:**
- **No horizontal forces**: In the inertial frame, there are no forces in the x or y directions, so the ball maintains constant horizontal velocity
- **Only gravity in z-direction**: The ball accelerates downward with acceleration $-g$

### Integration During Free Flight

Given initial conditions at the moment of liftoff $t_0$:
- Position: $(x_b(t_0), y_b(t_0), z_b(t_0))$
- Velocity: $(\dot{x}_b(t_0), \dot{y}_b(t_0), \dot{z}_b(t_0))$

The trajectory during free flight (for $t > t_0$) is:

**Velocity:**
$$
\begin{align}
\dot{x}_b(t) &= \dot{x}_b(t_0) \\
\dot{y}_b(t) &= \dot{y}_b(t_0) \\
\dot{z}_b(t) &= \dot{z}_b(t_0) - g(t - t_0)
\end{align}
$$

**Position:**
$$
\begin{align}
x_b(t) &= x_b(t_0) + \dot{x}_b(t_0)(t - t_0) \\
y_b(t) &= y_b(t_0) + \dot{y}_b(t_0)(t - t_0) \\
z_b(t) &= z_b(t_0) + \dot{z}_b(t_0)(t - t_0) - \frac{1}{2}g(t - t_0)^2
\end{align}
$$

### Transition Back to Contact

The ball returns to contact with the table when:
$$
z_b(t) = z_{\text{table surface}}(x_b(t), y_b(t), t) + r
$$

For the tilted table:
$$
z_b(t) = z_t(t) + r + x_b(t)\theta(t) - y_b(t)\varphi(t)
$$

This is typically solved numerically by detecting when the ball's height equals the table surface height.

---

## Collision and Bouncing

When the ball returns to the table surface after free flight, a **collision** occurs. The collision is modeled using the coefficient of restitution.

### Detecting Collision

A collision occurs at time $t_c$ when:
1. The ball reaches the table surface: $z_b(t_c) = z_{\text{table surface}}(x_b(t_c), y_b(t_c), t_c) + r$
2. The ball is moving toward the table: $\dot{z}_b(t_c) - \dot{z}_{\text{table contact point}}(t_c) < 0$

### Velocity of Table Contact Point

At the collision point, the table surface velocity (at the location where the ball hits) is:

$$
\dot{z}_{\text{table contact}} = \dot{z}_t + \dot{x}_b\theta + x_b\dot{\theta} - \dot{y}_b\varphi - y_b\dot{\varphi}
$$

### Relative Normal Velocity

The relative velocity of the ball with respect to the table surface in the normal direction (z-direction for small angles):

**Before collision:**
$$
v_{n,\text{rel}}^{(\text{before})} = \dot{z}_b^{(\text{before})} - \dot{z}_{\text{table contact}}
$$

### Coefficient of Restitution

The coefficient of restitution $e$ relates the relative velocities before and after collision:

$$
\boxed{
v_{n,\text{rel}}^{(\text{after})} = -e \cdot v_{n,\text{rel}}^{(\text{before})}
}
$$

where:
- $e$ = coefficient of restitution ($0 \leq e \leq 1$)
  - $e = 1$: perfectly elastic collision (no energy loss)
  - $e = 0$: perfectly inelastic collision (ball sticks to table)
  - $0 < e < 1$: realistic collision (some energy dissipated)

### Post-Collision Velocity

After collision, the ball's z-velocity becomes:

$$
\boxed{
\dot{z}_b^{(\text{after})} = \dot{z}_{\text{table contact}} - e \left(\dot{z}_b^{(\text{before})} - \dot{z}_{\text{table contact}}\right)
}
$$

Expanding:
$$
\dot{z}_b^{(\text{after})} = (1 + e)\dot{z}_{\text{table contact}} - e\dot{z}_b^{(\text{before})}
$$

**Tangential velocity** (x and y directions) remains unchanged during collision (assuming no tangential impulse):
$$
\begin{align}
\dot{x}_b^{(\text{after})} &= \dot{x}_b^{(\text{before})} \\
\dot{y}_b^{(\text{after})} &= \dot{y}_b^{(\text{before})}
\end{align}
$$

### After Collision

After the collision, check the normal force:
- If $N > 0$: Ball remains in contact, switch to **contact mode** dynamics
- If $N \leq 0$: Ball immediately lifts off again, continue in **free flight mode**

---

## Mode Switching Summary

The complete system has two modes:

### Contact Mode
- **Condition**: $z_b = z_{\text{table surface}} + r$ and $N > 0$
- **Equations**:
  $$
  \begin{align}
  \ddot{x}_b &= -g\theta - \dot{\theta}^2 x_b - \ddot{\theta} z_b - \frac{\mu N}{m} \cdot \text{sgn}(v_{\text{rel},x}) \\
  \ddot{y}_b &= g\varphi - \dot{\varphi}^2 y_b - \ddot{\varphi} z_b - \frac{\mu N}{m} \cdot \text{sgn}(v_{\text{rel},y}) \\
  \ddot{z}_b &= -g + \frac{N}{m} + \ddot{\varphi} y_b - \ddot{\theta} x_b - \dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
  \end{align}
  $$
- **Constraint**: $z_b = z_t + r + x_b\theta - y_b\varphi$

### Free Flight Mode
- **Condition**: $N \leq 0$ or $z_b > z_{\text{table surface}} + r$
- **Equations**:
  $$
  \begin{align}
  \ddot{x}_b &= 0 \\
  \ddot{y}_b &= 0 \\
  \ddot{z}_b &= -g
  \end{align}
  $$
- **No constraint**: Ball follows ballistic trajectory

### Transitions
- **Contact → Free Flight**: When $N$ becomes negative or zero
- **Free Flight → Contact**: When ball reaches table surface with collision velocity update using coefficient of restitution $e$

---

## Summary of Key Equations

### Ball Dynamics (In Contact)
$$
\boxed{
\begin{align}
\ddot{x}_b &= -g\theta - \dot{\theta}^2 x_b - \ddot{\theta} z_b - \frac{\mu N}{m} \cdot \text{sgn}(v_{\text{rel},x}) \\
\ddot{y}_b &= g\varphi - \dot{\varphi}^2 y_b - \ddot{\varphi} z_b - \frac{\mu N}{m} \cdot \text{sgn}(v_{\text{rel},y}) \\
\ddot{z}_b &= -g + \frac{N}{m} + \ddot{\varphi} y_b - \ddot{\theta} x_b - \dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
\end{align}
}
$$

### Normal Force (General Form)

$$
\boxed{
N = m\left[g + \ddot{z}_t + \ddot{x}_b\theta - \ddot{y}_b\varphi + 2\dot{x}_b\dot{\theta} - 2\dot{y}_b\dot{\varphi} + 2x_b\ddot{\theta} - 2y_b\ddot{\varphi} + \dot{\varphi}^2 z_b + \dot{\theta}^2 z_b\right]
}
$$

**Note**: This is an implicit equation since $\ddot{x}_b$ and $\ddot{y}_b$ depend on $N$.

**Simplified form** (for small lateral velocities $\dot{x}_b \approx 0$, $\dot{y}_b \approx 0$ and $z_b \approx z_t + r$):
$$
\boxed{
N \approx m\left[g + \ddot{z}_t + 2x_b\ddot{\theta} - 2y_b\ddot{\varphi} + \dot{\varphi}^2(z_t + r) + \dot{\theta}^2(z_t + r)\right]
}
$$

### Contact Condition

$$
\boxed{
z_b \geq z_t + r + x_b\theta - y_b\varphi \quad \text{and} \quad N > 0
}
$$

### Relative Velocity

$$
\boxed{
\begin{align}
v_{\text{rel},x} &= \dot{x}_b - \dot{\theta}(z_b - z_t) \\
v_{\text{rel},y} &= \dot{y}_b + \dot{\varphi}(z_b - z_t)
\end{align}
}
$$

### Ball Dynamics (In Free Flight)

$$
\boxed{
\begin{align}
\ddot{x}_b &= 0 \\
\ddot{y}_b &= 0 \\
\ddot{z}_b &= -g
\end{align}
}
$$

### Collision Velocity Update

When the ball collides with the table (coefficient of restitution $e$):

$$
\boxed{
\dot{z}_b^{(\text{after})} = (1 + e)\dot{z}_{\text{table contact}} - e\dot{z}_b^{(\text{before})}
}
$$

where $\dot{z}_{\text{table contact}} = \dot{z}_t + \dot{x}_b\theta + x_b\dot{\theta} - \dot{y}_b\varphi - y_b\dot{\varphi}$

Tangential velocities remain unchanged:
$$
\boxed{
\dot{x}_b^{(\text{after})} = \dot{x}_b^{(\text{before})}, \quad \dot{y}_b^{(\text{after})} = \dot{y}_b^{(\text{before})}
}
$$

---

## State Space Form

The complete system is a **hybrid dynamical system** with two discrete modes (contact and free flight) and continuous dynamics within each mode.

### State Vector

$$
\mathbf{x} = \begin{bmatrix}
x_b \\
y_b \\
z_b \\
\dot{x}_b \\
\dot{y}_b \\
\dot{z}_b
\end{bmatrix} \in \mathbb{R}^6
$$

where:
- States 1-3: Ball position $(x_b, y_b, z_b)$
- States 4-6: Ball velocity $(\dot{x}_b, \dot{y}_b, \dot{z}_b)$

### Input Vector

The table motion serves as the system input:
$$
\mathbf{u} = \begin{bmatrix}
\varphi \\
\theta \\
z_t \\
\dot{\varphi} \\
\dot{\theta} \\
\dot{z}_t \\
\ddot{\varphi} \\
\ddot{\theta} \\
\ddot{z}_t
\end{bmatrix} \in \mathbb{R}^9
$$

where:
- $\varphi, \theta, z_t$: Table configuration (roll, pitch, vertical position)
- $\dot{\varphi}, \dot{\theta}, \dot{z}_t$: Table angular and linear velocities
- $\ddot{\varphi}, \ddot{\theta}, \ddot{z}_t$: Table angular and linear accelerations

### Mode Variable

$$
q \in \{\text{contact}, \text{free flight}\}
$$

---

## State Space Dynamics

The system dynamics are:
$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}, q)
$$

### Mode 1: Contact Dynamics

**Mode condition**: $q = \text{contact}$ when $N > 0$ and $z_b = z_t + r + x_b\theta - y_b\varphi$

The state derivative is:
$$
\boxed{
\dot{\mathbf{x}} = \begin{bmatrix}
\dot{x}_b \\
\dot{y}_b \\
\dot{z}_b \\
\ddot{x}_b \\
\ddot{y}_b \\
\ddot{z}_b
\end{bmatrix} = \begin{bmatrix}
\dot{x}_b \\
\dot{y}_b \\
\dot{z}_b \\
-g\theta - \dot{\theta}^2 x_b - \ddot{\theta} z_b - \frac{\mu N}{m} \cdot \text{sgn}(v_{\text{rel},x}) \\
g\varphi - \dot{\varphi}^2 y_b - \ddot{\varphi} z_b - \frac{\mu N}{m} \cdot \text{sgn}(v_{\text{rel},y}) \\
-g + \frac{N}{m} + \ddot{\varphi} y_b - \ddot{\theta} x_b - \dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
\end{bmatrix}
}
$$

where:

**Relative velocities:**
$$
\begin{align}
v_{\text{rel},x} &= \dot{x}_b - \dot{\theta}(z_b - z_t) \\
v_{\text{rel},y} &= \dot{y}_b + \dot{\varphi}(z_b - z_t)
\end{align}
$$

**Normal Force:**

*Simplified form* (assuming small lateral velocities $\dot{x}_b \approx 0$, $\dot{y}_b \approx 0$):
$$
N = m\left[g + \ddot{z}_t + 2x_b\ddot{\theta} - 2y_b\ddot{\varphi} + \dot{\varphi}^2(z_t + r) + \dot{\theta}^2(z_t + r)\right]
$$

*General form* (implicit):
$$
\begin{align}
N = m\Bigg[&g + \ddot{z}_t + \ddot{x}_b\theta - \ddot{y}_b\varphi \\
&+ 2\dot{x}_b\dot{\theta} - 2\dot{y}_b\dot{\varphi} + 2x_b\ddot{\theta} - 2y_b\ddot{\varphi} \\
&+ \dot{\varphi}^2 z_b + \dot{\theta}^2 z_b\Bigg]
\end{align}
$$

**Note**: The general form requires solving implicitly since $N$ appears in $\ddot{x}_b$ and $\ddot{y}_b$.

---

### Friction Models

The equations above use **Coulomb friction** with the sign function. For numerical simulation, **viscous friction** is often preferred:

#### Coulomb Friction (as shown above)
$$
F_{\text{friction}} = -\mu N \cdot \text{sgn}(v_{\text{rel}})
$$

**Issues**: Discontinuous at $v_{\text{rel}} = 0$, can cause numerical chatter.

#### Viscous Friction (recommended for simulation)
$$
F_{\text{friction}} = -b \cdot v_{\text{rel}}
$$

where $b$ is the viscous friction coefficient (units: N·s/m).

**Contact mode dynamics with viscous friction:**
$$
\boxed{
\dot{\mathbf{x}} = \begin{bmatrix}
\dot{x}_b \\
\dot{y}_b \\
\dot{z}_b \\
-g\theta - \dot{\theta}^2 x_b - \ddot{\theta} z_b - \frac{b}{m} v_{\text{rel},x} \\
g\varphi - \dot{\varphi}^2 y_b - \ddot{\varphi} z_b - \frac{b}{m} v_{\text{rel},y} \\
-g + \frac{N}{m} + \ddot{\varphi} y_b - \ddot{\theta} x_b - \dot{\varphi}^2 z_b - \dot{\theta}^2 z_b
\end{bmatrix}
}
$$

**Advantages**:
- Smooth and continuous
- No numerical issues at zero velocity
- Easier to integrate numerically

**Choosing friction coefficient**:
- Approximate relationship: $b \approx \mu N / v_{\text{typical}}$
- For ball radius $r$ and typical sliding speed $v_{\text{slide}} \approx 0.1$ m/s: $b \approx 10\mu mg$

#### Smoothed Coulomb Friction (alternative)
$$
F_{\text{friction}} = -\mu N \cdot \tanh\left(\frac{v_{\text{rel}}}{\epsilon}\right)
$$

where $\epsilon$ is a small smoothing parameter (e.g., $\epsilon = 0.001$ m/s).

This approximates Coulomb friction but remains smooth near zero velocity.

---

### Mode 2: Free Flight Dynamics

**Mode condition**: $q = \text{free flight}$ when $N \leq 0$ or $z_b > z_t + r + x_b\theta - y_b\varphi$

The state derivative is:
$$
\boxed{
\dot{\mathbf{x}} = \begin{bmatrix}
\dot{x}_b \\
\dot{y}_b \\
\dot{z}_b \\
\ddot{x}_b \\
\ddot{y}_b \\
\ddot{z}_b
\end{bmatrix} = \begin{bmatrix}
\dot{x}_b \\
\dot{y}_b \\
\dot{z}_b \\
0 \\
0 \\
-g
\end{bmatrix}
}
$$

**Note**: In free flight mode, the dynamics are **independent of the input** $\mathbf{u}$ (the table motion doesn't affect the ball).

---

## Mode Transitions and Guards

### Guard Conditions

**Transition from Contact to Free Flight**:
$$
G_{\text{contact} \to \text{free}} : N \leq 0
$$

**Transition from Free Flight to Contact**:
$$
G_{\text{free} \to \text{contact}} : z_b \leq z_t + r + x_b\theta - y_b\varphi \quad \text{and} \quad \dot{z}_b < \dot{z}_{\text{table contact}}
$$

### Reset Map (Collision)

When transitioning from free flight to contact (collision event), apply the reset map:

$$
\boxed{
\mathbf{x}^+ = R_{\text{collision}}(\mathbf{x}^-, \mathbf{u}) = \begin{bmatrix}
x_b^- \\
y_b^- \\
z_t + r + x_b^- \theta - y_b^- \varphi \\
\dot{x}_b^- \\
\dot{y}_b^- \\
(1+e)\dot{z}_{\text{table}} - e \dot{z}_b^-
\end{bmatrix}
}
$$

where:
- $\mathbf{x}^-$ = state just before collision
- $\mathbf{x}^+$ = state just after collision
- $\dot{z}_{\text{table}} = \dot{z}_t + \dot{x}_b^- \theta + x_b^- \dot{\theta} - \dot{y}_b^- \varphi - y_b^- \dot{\varphi}$
- $e$ = coefficient of restitution

---

## Complete State Space Representation

The complete hybrid system can be written as:

$$
\boxed{
\dot{\mathbf{x}} = \begin{cases}
f^{\text{contact}}(\mathbf{x}, \mathbf{u}) & \text{if } q = \text{contact} \\
f^{\text{free flight}}(\mathbf{x}, \mathbf{u}) & \text{if } q = \text{free flight}
\end{cases}
}
$$

$$
\boxed{
q^+ = \begin{cases}
\text{free flight} & \text{if } q = \text{contact and } G_{\text{contact} \to \text{free}} \\
\text{contact} & \text{if } q = \text{free flight and } G_{\text{free} \to \text{contact}} \\
q & \text{otherwise}
\end{cases}
}
$$

$$
\boxed{
\mathbf{x}^+ = \begin{cases}
R_{\text{collision}}(\mathbf{x}^-, \mathbf{u}) & \text{if transition from free flight to contact} \\
\mathbf{x}^- & \text{otherwise}
\end{cases}
}
$$

---

## Summary of System Properties

This is a **nonlinear hybrid dynamical system** with the following characteristics:

1. **Nonlinearities**:
   - Trigonometric functions (angles $\varphi$, $\theta$)
   - Sign functions in friction terms
   - Quadratic terms (centrifugal accelerations)
   - Product terms (coupling between states and inputs)

2. **Hybrid nature**:
   - Two discrete modes (contact and free flight)
   - Mode-dependent continuous dynamics
   - Guard conditions for mode transitions
   - Reset map at collision events

3. **State dimension**: 6 (position and velocity in 3D)

4. **Input dimension**: 9 (table position, velocity, and acceleration in 3 DOF)

5. **Constraints**:
   - Contact mode: holonomic constraint $z_b = z_t + r + x_b\theta - y_b\varphi$
   - Unilateral constraint: $N \geq 0$

6. **Discontinuities**:
   - State jumps at collision (z-velocity changes by reset map)
   - Force discontinuities at mode transitions
   - Friction sign function discontinuity at zero relative velocity

---

## Numerical Integration with Event Detection

Simulating this hybrid dynamical system requires a numerical integration method that can handle:
1. Smooth continuous dynamics within each mode
2. Discontinuous mode transitions (events)
3. State resets at collisions

The recommended approach is **Runge-Kutta integration with event detection**.

---

### Runge-Kutta Method (RK4)

The classic 4th-order Runge-Kutta (RK4) method integrates the continuous dynamics:

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}, q, t)
$$

#### RK4 Algorithm

Given current state $\mathbf{x}_n$ at time $t_n$, compute the next state $\mathbf{x}_{n+1}$ at time $t_{n+1} = t_n + h$:

**Step 1**: Compute intermediate slopes:
$$
\begin{align}
\mathbf{k}_1 &= f(\mathbf{x}_n, \mathbf{u}_n, q_n, t_n) \\
\mathbf{k}_2 &= f(\mathbf{x}_n + \frac{h}{2}\mathbf{k}_1, \mathbf{u}_{n+1/2}, q_n, t_n + \frac{h}{2}) \\
\mathbf{k}_3 &= f(\mathbf{x}_n + \frac{h}{2}\mathbf{k}_2, \mathbf{u}_{n+1/2}, q_n, t_n + \frac{h}{2}) \\
\mathbf{k}_4 &= f(\mathbf{x}_n + h\mathbf{k}_3, \mathbf{u}_{n+1}, q_n, t_{n+1})
\end{align}
$$

**Step 2**: Combine slopes to get next state:
$$
\mathbf{x}_{n+1} = \mathbf{x}_n + \frac{h}{6}\left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\right)
$$

where $h$ is the time step (e.g., $h = 0.001$ s for 1 kHz simulation).

**Note**: $\mathbf{u}_{n+1/2}$ is the input at the midpoint, which can be interpolated from $\mathbf{u}_n$ and $\mathbf{u}_{n+1}$.

---

### Event Detection

Events occur when the system crosses a **guard condition** (mode transition). We need to detect these events accurately.

#### Event Functions

Define event functions that change sign when an event occurs:

**Event 1: Contact to Free Flight** (liftoff)
$$
g_1(\mathbf{x}, \mathbf{u}, t) = N(\mathbf{x}, \mathbf{u})
$$

Event occurs when $g_1$ changes from positive to negative (or reaches zero).

**Event 2: Free Flight to Contact** (collision)
$$
g_2(\mathbf{x}, \mathbf{u}, t) = z_b - z_t - r - x_b\theta + y_b\varphi
$$

Event occurs when $g_2$ changes from positive to negative **and** the ball is moving toward the table.

#### Event Detection Algorithm

After each RK4 step, check if an event occurred:

**Step 1**: Evaluate event functions at $t_n$ and $t_{n+1}$:
$$
\begin{align}
g_i^n &= g_i(\mathbf{x}_n, \mathbf{u}_n, t_n) \\
g_i^{n+1} &= g_i(\mathbf{x}_{n+1}, \mathbf{u}_{n+1}, t_{n+1})
\end{align}
$$

**Step 2**: Check for sign change (zero crossing):
$$
\text{Event detected if: } g_i^n \cdot g_i^{n+1} < 0 \quad \text{or} \quad g_i^{n+1} = 0
$$

**Step 3**: If event detected, refine the event time using **bisection** or **interpolation**:

##### Bisection Method
1. Set $t_{\text{left}} = t_n$, $t_{\text{right}} = t_{n+1}$
2. While $|t_{\text{right}} - t_{\text{left}}| > \epsilon_{\text{time}}$:
   - $t_{\text{mid}} = \frac{t_{\text{left}} + t_{\text{right}}}{2}$
   - Integrate from $t_n$ to $t_{\text{mid}}$ to get $\mathbf{x}_{\text{mid}}$
   - Evaluate $g_i(\mathbf{x}_{\text{mid}}, \mathbf{u}_{\text{mid}}, t_{\text{mid}})$
   - If $g_i^n \cdot g_i^{\text{mid}} < 0$: $t_{\text{right}} = t_{\text{mid}}$
   - Else: $t_{\text{left}} = t_{\text{mid}}$
3. Event time: $t_{\text{event}} \approx t_{\text{mid}}$

##### Linear Interpolation (faster, less accurate)
$$
t_{\text{event}} \approx t_n - g_i^n \frac{h}{g_i^{n+1} - g_i^n}
$$

**Step 4**: Integrate to the event time $t_{\text{event}}$ to get state $\mathbf{x}_{\text{event}}^-$ just before the event.

**Step 5**: Apply the appropriate action:
- **Liftoff**: Switch mode to free flight, $q = \text{free flight}$, state unchanged
- **Collision**: Apply reset map $\mathbf{x}_{\text{event}}^+ = R_{\text{collision}}(\mathbf{x}_{\text{event}}^-, \mathbf{u}_{\text{event}})$, switch mode to contact

**Step 6**: Continue integration from $\mathbf{x}_{\text{event}}^+$ at time $t_{\text{event}}$ until $t_{n+1}$.

---

### Complete Integration Algorithm

```
Initialize: x = x_0, q = contact, t = 0

For n = 0, 1, 2, ..., N_steps:
    # Standard RK4 step
    x_trial = RK4_step(x, u, q, t, h)
    t_trial = t + h

    # Check for events
    events_detected = check_events(x, x_trial, u, t, t_trial, q)

    If events_detected is not empty:
        # Find first event
        t_event, event_type = find_first_event(x, u, q, t, h, events_detected)

        # Integrate to event time
        x_before = integrate_to(x, u, q, t, t_event)

        # Handle event
        If event_type == LIFTOFF:
            q = free_flight
            x_after = x_before  # No state reset

        Else if event_type == COLLISION:
            q = contact
            x_after = reset_map(x_before, u_event)

        # Continue integration from event to end of step
        x = integrate_from(x_after, u, q, t_event, t_trial)
        t = t_trial

    Else:
        # No events, accept trial step
        x = x_trial
        t = t_trial

    # Store solution
    X[n+1] = x
    T[n+1] = t

End For
```

---

### Time Step Selection

Choose time step $h$ based on:

1. **Dynamics timescale**: Fastest dynamics in the system
   - For ball on table: $\omega_{\max} \approx \sqrt{g/r}$ → $T_{\min} \approx 2\pi\sqrt{r/g}$
   - Typical: $r = 0.02$ m → $T_{\min} \approx 0.09$ s
   - Use $h \approx T_{\min}/100 = 0.001$ s (1 kHz)

2. **Event accuracy**: Smaller $h$ improves event detection accuracy

3. **Real-time constraints**: For real-time control, $h$ must match controller sample time

**Adaptive time stepping**: Use smaller $h$ near events, larger $h$ during smooth motion.

---

### Implementation Tips

1. **Event tolerance**: Use $\epsilon_{\text{event}} \approx 10^{-6}$ for event function zero crossing

2. **Multiple simultaneous events**: If multiple events detected, handle the earliest one first

3. **Chattering prevention**: If ball repeatedly bounces (chatter), use:
   - Energy dissipation check: Stop bouncing when kinetic energy < threshold
   - Velocity threshold: Treat $|\dot{z}_{\text{rel}}| < v_{\min}$ as permanent contact

4. **Constraint stabilization**: In contact mode, enforce constraint:
   $$
   z_b \leftarrow z_t + r + x_b\theta - y_b\varphi
   $$
   after each step to prevent drift

5. **Library support**:
   - **MATLAB**: `ode45` with `Events` option
   - **Python**: `scipy.integrate.solve_ivp` with `events` parameter
   - **C++**: Custom RK4 + event detection or use libraries like Boost.Numeric.Odeint

---

### Example: C++ Implementation

```cpp
#include <vector>
#include <array>
#include <functional>
#include <cmath>
#include <algorithm>

// Constants
constexpr double g = 9.81;      // Gravity [m/s²]
constexpr double r = 0.02;      // Ball radius [m]
constexpr double m = 0.1;       // Ball mass [kg]
constexpr double b = 0.01;      // Viscous friction coefficient [N·s/m]
constexpr double e = 0.8;       // Coefficient of restitution

// State vector: [x_b, y_b, z_b, vx_b, vy_b, vz_b]
using State = std::array<double, 6>;

// Input vector: [φ, θ, z_t, φ̇, θ̇, ż_t, φ̈, θ̈, z̈_t]
using Input = std::array<double, 9>;

enum class Mode { CONTACT, FREE_FLIGHT };

// Input function (user-defined)
using InputFunction = std::function<Input(double)>;

//-----------------------------------------------------------------------------
// Dynamics Functions
//-----------------------------------------------------------------------------

double compute_normal_force(const State& x, const Input& u) {
    // Simplified normal force (assuming small lateral velocities)
    double x_b = x[0], y_b = x[1], z_b = x[2];
    double phi = u[0], theta = u[1], z_t = u[2];
    double phi_dot = u[3], theta_dot = u[4];
    double phi_ddot = u[6], theta_ddot = u[7], z_t_ddot = u[8];

    double N = m * (g + z_t_ddot +
                    2*x_b*theta_ddot - 2*y_b*phi_ddot +
                    phi_dot*phi_dot*(z_t + r) + theta_dot*theta_dot*(z_t + r));

    return N;
}

State contact_dynamics(const State& x, const Input& u) {
    State dx;

    // Extract state
    double x_b = x[0], y_b = x[1], z_b = x[2];
    double vx_b = x[3], vy_b = x[4], vz_b = x[5];

    // Extract input
    double phi = u[0], theta = u[1], z_t = u[2];
    double phi_dot = u[3], theta_dot = u[4];
    double phi_ddot = u[6], theta_ddot = u[7];

    // Compute normal force
    double N = compute_normal_force(x, u);

    // Relative velocities
    double v_rel_x = vx_b - theta_dot*(z_b - z_t);
    double v_rel_y = vy_b + phi_dot*(z_b - z_t);

    // State derivative: [vx, vy, vz, ax, ay, az]
    dx[0] = vx_b;
    dx[1] = vy_b;
    dx[2] = vz_b;

    // Accelerations (with viscous friction)
    dx[3] = -g*theta - theta_dot*theta_dot*x_b - theta_ddot*z_b - (b/m)*v_rel_x;
    dx[4] = g*phi - phi_dot*phi_dot*y_b - phi_ddot*z_b - (b/m)*v_rel_y;
    dx[5] = -g + N/m + phi_ddot*y_b - theta_ddot*x_b -
            phi_dot*phi_dot*z_b - theta_dot*theta_dot*z_b;

    return dx;
}

State free_flight_dynamics(const State& x, const Input& u) {
    State dx;

    // State derivative: [vx, vy, vz, 0, 0, -g]
    dx[0] = x[3];  // vx
    dx[1] = x[4];  // vy
    dx[2] = x[5];  // vz
    dx[3] = 0.0;   // ax = 0
    dx[4] = 0.0;   // ay = 0
    dx[5] = -g;    // az = -g

    return dx;
}

//-----------------------------------------------------------------------------
// RK4 Integration Step
//-----------------------------------------------------------------------------

State rk4_step(const State& x, const Input& u_start, const Input& u_mid,
               const Input& u_end, double h, Mode mode) {
    State k1, k2, k3, k4;
    State x_temp;

    // k1 = f(x_n, u_n)
    k1 = (mode == Mode::CONTACT) ? contact_dynamics(x, u_start)
                                  : free_flight_dynamics(x, u_start);

    // k2 = f(x_n + h/2*k1, u_{n+1/2})
    for (int i = 0; i < 6; ++i) x_temp[i] = x[i] + 0.5*h*k1[i];
    k2 = (mode == Mode::CONTACT) ? contact_dynamics(x_temp, u_mid)
                                  : free_flight_dynamics(x_temp, u_mid);

    // k3 = f(x_n + h/2*k2, u_{n+1/2})
    for (int i = 0; i < 6; ++i) x_temp[i] = x[i] + 0.5*h*k2[i];
    k3 = (mode == Mode::CONTACT) ? contact_dynamics(x_temp, u_mid)
                                  : free_flight_dynamics(x_temp, u_mid);

    // k4 = f(x_n + h*k3, u_{n+1})
    for (int i = 0; i < 6; ++i) x_temp[i] = x[i] + h*k3[i];
    k4 = (mode == Mode::CONTACT) ? contact_dynamics(x_temp, u_end)
                                  : free_flight_dynamics(x_temp, u_end);

    // x_{n+1} = x_n + h/6*(k1 + 2*k2 + 2*k3 + k4)
    State x_next;
    for (int i = 0; i < 6; ++i) {
        x_next[i] = x[i] + (h/6.0)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }

    return x_next;
}

//-----------------------------------------------------------------------------
// Event Detection
//-----------------------------------------------------------------------------

enum class EventType { NONE, LIFTOFF, COLLISION };

struct Event {
    EventType type;
    double time;
    State state;
};

double liftoff_event_function(const State& x, const Input& u, Mode mode) {
    if (mode != Mode::CONTACT) return 1.0;
    return compute_normal_force(x, u);
}

double collision_event_function(const State& x, const Input& u, Mode mode) {
    if (mode != Mode::FREE_FLIGHT) return 1.0;
    double z_table = u[2] + r + x[0]*u[1] - x[1]*u[0];
    return x[2] - z_table;
}

// Bisection method to refine event time
Event refine_event(const State& x_start, const Input& u_start,
                   double t_start, double t_end, Mode mode,
                   EventType event_type, const InputFunction& u_func,
                   double tolerance = 1e-6) {

    double t_left = t_start;
    double t_right = t_end;
    State x_event;

    // Bisection loop
    while (t_right - t_left > tolerance) {
        double t_mid = 0.5 * (t_left + t_right);
        double h_left = t_mid - t_start;

        // Integrate from t_start to t_mid
        Input u_mid = u_func(t_mid);
        Input u_mid_half = u_func(t_start + h_left/2);
        x_event = rk4_step(x_start, u_start, u_mid_half, u_mid, h_left, mode);

        // Evaluate event function
        double g_mid = (event_type == EventType::LIFTOFF)
                       ? liftoff_event_function(x_event, u_mid, mode)
                       : collision_event_function(x_event, u_mid, mode);

        double g_start = (event_type == EventType::LIFTOFF)
                         ? liftoff_event_function(x_start, u_start, mode)
                         : collision_event_function(x_start, u_start, mode);

        // Check which side the zero crossing is on
        if (g_start * g_mid < 0) {
            t_right = t_mid;
        } else {
            t_left = t_mid;
        }
    }

    return {event_type, 0.5*(t_left + t_right), x_event};
}

// Apply collision reset map
State apply_collision_reset(const State& x_before, const Input& u) {
    State x_after = x_before;

    double x_b = x_before[0], y_b = x_before[1];
    double vx_b = x_before[3], vy_b = x_before[4], vz_b = x_before[5];

    double phi = u[0], theta = u[1], z_t = u[2];
    double phi_dot = u[3], theta_dot = u[4], z_t_dot = u[5];

    // Enforce position constraint
    x_after[2] = z_t + r + x_b*theta - y_b*phi;

    // Table contact point velocity
    double vz_table = z_t_dot + vx_b*theta + x_b*theta_dot -
                      vy_b*phi - y_b*phi_dot;

    // Apply coefficient of restitution
    x_after[5] = (1 + e)*vz_table - e*vz_b;

    // Tangential velocities unchanged
    x_after[3] = vx_b;
    x_after[4] = vy_b;

    return x_after;
}

//-----------------------------------------------------------------------------
// Main Simulation Loop
//-----------------------------------------------------------------------------

struct SimulationResult {
    std::vector<double> time;
    std::vector<State> states;
    std::vector<Mode> modes;
};

SimulationResult simulate(const State& x_0, Mode initial_mode,
                          const InputFunction& u_func,
                          double t_final, double h) {

    SimulationResult result;

    State x = x_0;
    Mode mode = initial_mode;
    double t = 0.0;

    // Store initial condition
    result.time.push_back(t);
    result.states.push_back(x);
    result.modes.push_back(mode);

    while (t < t_final) {
        // Get inputs
        Input u_start = u_func(t);
        Input u_mid = u_func(t + h/2);
        Input u_end = u_func(std::min(t + h, t_final));

        // Take RK4 step
        State x_trial = rk4_step(x, u_start, u_mid, u_end, h, mode);
        double t_trial = std::min(t + h, t_final);

        // Check for events
        double g_start_liftoff = liftoff_event_function(x, u_start, mode);
        double g_end_liftoff = liftoff_event_function(x_trial, u_end, mode);

        double g_start_collision = collision_event_function(x, u_start, mode);
        double g_end_collision = collision_event_function(x_trial, u_end, mode);

        bool liftoff_detected = (g_start_liftoff * g_end_liftoff < 0) &&
                                (mode == Mode::CONTACT);
        bool collision_detected = (g_start_collision * g_end_collision < 0) &&
                                  (mode == Mode::FREE_FLIGHT) &&
                                  (x_trial[5] < 0); // Moving downward

        if (liftoff_detected) {
            // Refine liftoff event
            Event event = refine_event(x, u_start, t, t_trial, mode,
                                      EventType::LIFTOFF, u_func);

            // Update state and mode
            x = event.state;
            t = event.time;
            mode = Mode::FREE_FLIGHT;

            // Store
            result.time.push_back(t);
            result.states.push_back(x);
            result.modes.push_back(mode);

        } else if (collision_detected) {
            // Refine collision event
            Event event = refine_event(x, u_start, t, t_trial, mode,
                                      EventType::COLLISION, u_func);

            // Apply collision reset
            Input u_event = u_func(event.time);
            x = apply_collision_reset(event.state, u_event);
            t = event.time;
            mode = Mode::CONTACT;

            // Store
            result.time.push_back(t);
            result.states.push_back(x);
            result.modes.push_back(mode);

        } else {
            // No event, accept step
            x = x_trial;
            t = t_trial;

            // Store
            result.time.push_back(t);
            result.states.push_back(x);
            result.modes.push_back(mode);
        }
    }

    return result;
}

//-----------------------------------------------------------------------------
// Example Usage
//-----------------------------------------------------------------------------

int main() {
    // Initial state: ball at origin, on table
    State x_0 = {0.0, 0.0, r, 0.0, 0.0, 0.0};

    // Define input function (example: sinusoidal pitch)
    auto u_func = [](double t) -> Input {
        double phi = 0.0;
        double theta = 0.1 * std::sin(2*M_PI*t);  // 1 Hz oscillation
        double z_t = 0.0;
        double phi_dot = 0.0;
        double theta_dot = 0.1 * 2*M_PI * std::cos(2*M_PI*t);
        double z_t_dot = 0.0;
        double phi_ddot = 0.0;
        double theta_ddot = -0.1 * (2*M_PI)*(2*M_PI) * std::sin(2*M_PI*t);
        double z_t_ddot = 0.0;

        return {phi, theta, z_t, phi_dot, theta_dot, z_t_dot,
                phi_ddot, theta_ddot, z_t_ddot};
    };

    // Run simulation
    double t_final = 5.0;  // 5 seconds
    double h = 0.001;      // 1 ms time step

    SimulationResult result = simulate(x_0, Mode::CONTACT, u_func, t_final, h);

    // Results are now in result.time, result.states, result.modes
    // Can be saved to file or plotted

    return 0;
}
```

---

## Notes

1. **Small angle approximation**: The equations above use small angle approximations. For large angles, use the full trigonometric expressions.

2. **Rolling vs. sliding**: This model assumes sliding friction. For rolling motion, additional constraints and rotational dynamics of the ball must be included.

3. **Validation**: Compare simulation results with physical experiments or high-fidelity simulations to validate the model.

---

## References

- Classical Mechanics (Goldstein)
- Dynamics and Control of Mechanical Systems (Block et al.)
- Ball and Beam Control System Design
