![[screenshot-2026-02-20_23-41-07.png]]
![[screenshot-2026-02-20_23-47-16.png]]
![[screenshot-2026-02-22_18-25-57.png]]

---

## System Description

The table is a **3-arm parallel mechanism** (similar in concept to a delta robot) with three identical arms that actuate the table's pose. The system has 3 degrees of freedom matching exactly the table's 3 DOFs from the ball dynamics document:

- $\varphi(t)$ = roll angle (rotation about x-axis)
- $\theta(t)$ = pitch angle (rotation about y-axis)
- $z_t(t)$ = vertical position of table center

Each arm consists of:
- **Lower link** (length $L_1$): from the ground mounting point to the revolute joint (elbow)
- **Upper link** (length $L_2$): from the elbow to the table attachment point

The lower link angle $\alpha_i$ (measured from horizontal) is driven by a servo motor and is the **actuated variable**. The upper link angle follows passively from the geometry.

---

## Coordinate Frames and Geometry

### Inertial Frame $\{I\}$

Fixed frame with origin at the center of the ground mounting circle:
- $X$ axis: pointing toward arm 1's ground point
- $Y$ axis: $90°$ counterclockwise from $X$ in the horizontal plane
- $Z$ axis: pointing upward

### Mounting Geometry

**Ground attachment points** lie at radius $R_g$ from the center, equally spaced at $120°$:
$$
\mathbf{G}_i = \begin{bmatrix}
R_g \cos(\psi_i) \\
R_g \sin(\psi_i) \\
0
\end{bmatrix}, \quad \psi_i = \frac{2\pi(i-1)}{3}, \quad i = 1, 2, 3
$$

Explicitly:
$$
\mathbf{G}_1 = \begin{bmatrix} R_g \\ 0 \\ 0 \end{bmatrix}, \quad
\mathbf{G}_2 = \begin{bmatrix} -R_g/2 \\ R_g\sqrt{3}/2 \\ 0 \end{bmatrix}, \quad
\mathbf{G}_3 = \begin{bmatrix} -R_g/2 \\ -R_g\sqrt{3}/2 \\ 0 \end{bmatrix}
$$

**Table attachment points** lie at radius $R_t$ from the table center, at the same angular positions $\psi_i$, but expressed in the **table frame** $\{T\}$:
$$
\mathbf{P}_i^T = \begin{bmatrix}
R_t \cos(\psi_i) \\
R_t \sin(\psi_i) \\
0
\end{bmatrix}
$$

### Table Frame $\{T\}$

The table frame is attached to the table center. When the table has pose $(\varphi, \theta, z_t)$, the rotation from table frame to inertial frame is (consistent with the ball dynamics document):

$$
R = R_y(\theta) \cdot R_x(\varphi) = \begin{bmatrix}
\cos\theta & 0 & \sin\theta \\
\sin\theta\sin\varphi & \cos\varphi & -\cos\theta\sin\varphi \\
-\sin\theta\cos\varphi & \sin\varphi & \cos\theta\cos\varphi
\end{bmatrix}
$$

The table center position in the inertial frame:
$$
\mathbf{c} = \begin{bmatrix} 0 \\ 0 \\ z_t \end{bmatrix}
$$

**Note**: We assume the table center translates only vertically (no lateral translation), which is enforced by the symmetry of the 3-arm arrangement.

---

## Position of Table Attachment Points in Inertial Frame

The position of table attachment point $i$ in the inertial frame is:

$$
\mathbf{T}_i = \mathbf{c} + R \cdot \mathbf{P}_i^T
$$

Expanding:

$$
\mathbf{T}_i = \begin{bmatrix} 0 \\ 0 \\ z_t \end{bmatrix} + R \begin{bmatrix}
R_t \cos\psi_i \\
R_t \sin\psi_i \\
0
\end{bmatrix}
$$

$$
= \begin{bmatrix} 0 \\ 0 \\ z_t \end{bmatrix} + R_t \begin{bmatrix}
\cos\theta\cos\psi_i + 0 \\
\sin\theta\sin\varphi\cos\psi_i + \cos\varphi\sin\psi_i \\
-\sin\theta\cos\varphi\cos\psi_i + \sin\varphi\sin\psi_i
\end{bmatrix}
$$

So the full expression for each table attachment point in the inertial frame is:

$$
\boxed{
\mathbf{T}_i = \begin{bmatrix}
R_t \cos\theta \cos\psi_i \\
R_t (\sin\theta \sin\varphi \cos\psi_i + \cos\varphi \sin\psi_i) \\
z_t + R_t (-\sin\theta \cos\varphi \cos\psi_i + \sin\varphi \sin\psi_i)
\end{bmatrix}
}
$$
---

## Single-Arm Kinematics

Each arm lies in the **vertical plane** that contains the arm's radial direction $\hat{\mathbf{r}}_i$. The unit radial direction for arm $i$:

$$
\hat{\mathbf{r}}_i = \begin{bmatrix} \cos\psi_i \\ \sin\psi_i \\ 0 \end{bmatrix}
$$

### Elbow Position
The servo motor rotates the lower link about an axis tangent to the mounting circle (i.e., perpendicular to $\hat{\mathbf{r}}_i$ and horizontal). The angle $\alpha_i$ is measured from the horizontal plane, with $\alpha_i > 0$ meaning the elbow is above the ground mounting point.

In the arm's own 2D radial plane, the lower link tip (elbow) sits at:
- Radial distance from $\mathbf{G}_i$: $L_1 \cos\alpha_i$ (outward along $\hat{\mathbf{r}}_i$)
- Height above $\mathbf{G}_i$: $L_1 \sin\alpha_i$ (upward along $\hat{\mathbf{z}}$)

Converting back to 3D inertial coordinates, the radial displacement $L_1\cos\alpha_i$ along $\hat{\mathbf{r}}_i = [\cos\psi_i,\ \sin\psi_i,\ 0]^\top$ gives:
$$
\mathbf{E}_i = \mathbf{G}_i + L_1 \underbrace{\begin{bmatrix}
\cos\alpha_i \cos\psi_i \\
\cos\alpha_i \sin\psi_i \\
\sin\alpha_i
\end{bmatrix}}_{\text{lower link vector in } \{I\}}
$$

Expanding with $\mathbf{G}_i = [R_g\cos\psi_i,\ R_g\sin\psi_i,\ 0]^\top$:
$$
\mathbf{E}_i = \begin{bmatrix}
(R_g + L_1\cos\alpha_i)\cos\psi_i \\
(R_g + L_1\cos\alpha_i)\sin\psi_i \\
L_1\sin\alpha_i
\end{bmatrix}
$$

**Physical interpretation**: The elbow always lies on the circle of radius $(R_g + L_1\cos\alpha_i)$ in the horizontal plane, and at height $L_1\sin\alpha_i$ above the ground. When $\alpha_i = 0$ (link horizontal), the elbow is at radius $R_g + L_1$ at height $0$. When $\alpha_i = 90°$ (link vertical), the elbow is directly above $\mathbf{G}_i$ at height $L_1$.

### Upper Link Constraint

The upper link must connect the elbow $\mathbf{E}_i$ to the table attachment point $\mathbf{T}_i$, and its length is fixed at $L_2$:
$$
\|\mathbf{T}_i - \mathbf{E}_i\| = L_2
$$

This is the **fundamental constraint** of each arm. It relates the servo angle $\alpha_i$ to the table pose $(\varphi, \theta, z_t)$.

---

## Inverse Kinematics — Solving for Servo Angles

**Problem**: Given the desired table pose $(\varphi, \theta, z_t)$, find the three servo angles $\alpha_1, \alpha_2, \alpha_3$.

### Step 1: Compute Target Points

For each arm $i$, compute the target table attachment point $\mathbf{T}_i$ using the formula derived above, and remember that the ground points are $\mathbf{G}_i$ :
$$
\mathbf{T}_i = \begin{bmatrix}
R_t \cos\theta \cos\psi_i \\
R_t (\sin\theta \sin\varphi \cos\psi_i + \cos\varphi \sin\psi_i) \\
z_t + R_t (-\sin\theta \cos\varphi \cos\psi_i + \sin\varphi \sin\psi_i)
\end{bmatrix} \quad , \quad \mathbf{G}_i = \begin{bmatrix}
R_g \cos(\psi_i) \\
R_g \sin(\psi_i) \\
0
\end{bmatrix}
$$
### Step 2: Project onto the Arm's Radial Plane

Since each arm moves purely in the vertical plane spanned by $\hat{\mathbf{r}}_i$ and $\hat{\mathbf{z}}$, we need to reduce the 3D problem to a 2D one. We do this by decomposing the vector from the ground point to the target:

$$
\mathbf{d}_i = \mathbf{T}_i - \mathbf{G}_i
$$

#### The Radial and Tangential Directions

The key to understanding this step is to look at the horizontal plane from above. Each ground attachment point $\mathbf{G}_i$ sits on a circle of radius $R_g$ at angle $\psi_i$. For arm $i$, we define two horizontal directions:

- $\hat{\mathbf{r}}_i$: points **away from the center**, along the line from origin to $\mathbf{G}_i$ (radial)
- $\hat{\mathbf{t}}_i$: points **sideways**, tangent to the circle at $\mathbf{G}_i$, $90°$ counterclockwise from $\hat{\mathbf{r}}_i$ (tangential)

![[diagrams/arm-ground-geometry-top-view.svg]]

For arm 1 ($\psi_1 = 0°$): $\hat{\mathbf{r}}_1 = [1, 0, 0]^\top$, $\hat{\mathbf{t}}_1 = [0, -1, 0]^\top$ (wait — counterclockwise gives $[0, +1, 0]^\top$... let's be precise):

$$
\hat{\mathbf{r}}_i = \begin{bmatrix} \cos\psi_i \\ \sin\psi_i \\ 0 \end{bmatrix} \quad , \quad \hat{\mathbf{t}}_i = \begin{bmatrix} -\sin\psi_i \\ \cos\psi_i \\ 0 \end{bmatrix}
$$

For arm 1: $\hat{\mathbf{r}}_1 = [1,0,0]^\top$, $\hat{\mathbf{t}}_1 = [0,1,0]^\top$. These two vectors together with $\hat{\mathbf{z}}$ form a right-handed local coordinate system at each ground point.

The arm itself — both the lower link and upper link — can only move within the **vertical plane** containing $\hat{\mathbf{r}}_i$ and $\hat{\mathbf{z}}$. This is because the servo rotates the lower link about the tangential axis $\hat{\mathbf{t}}_i$; the motion is always in the radial-vertical plane. Think of it like a door hinge: the hinge axis runs tangentially, so the door (lower link) swings radially.

![[diagrams/arm-hinge-top-view.svg]]

![[diagrams/arm-side-view-radial-plane.svg]]

This is the 2D picture we want to solve. The entire IK reduces to geometry in this plane.

#### Why $\mathbf{T}_i$ is Generally Not in the Radial Plane

When the table is **flat** ($\varphi = \theta = 0$), the table attachment point $\mathbf{T}_i$ lies directly above $\mathbf{G}_i$ — offset radially by $R_t - R_g$ and vertically by $z_t$. It sits perfectly in arm $i$'s radial plane.

When the table **tilts**, $\mathbf{T}_i$ moves. Crucially, it can drift **sideways** — in the $\hat{\mathbf{t}}_i$ direction — because tilting rotates the table attachment point out of the radial plane. This sideways offset is $d_{t,i}$.

![[diagrams/arm-3d-offset-view.svg]]

In general, $\mathbf{d}_i = \mathbf{T}_i - \mathbf{G}_i$ has three components:

$$
d_{r,i} = \mathbf{d}_i \cdot \hat{\mathbf{r}}_i = (T_{i,x} - G_{i,x})\cos\psi_i + (T_{i,y} - G_{i,y})\sin\psi_i
$$
$$
d_{t,i} = \mathbf{d}_i \cdot \hat{\mathbf{t}}_i = -(T_{i,x} - G_{i,x})\sin\psi_i + (T_{i,y} - G_{i,y})\cos\psi_i
$$
$$
d_{z,i} = T_{i,z}
$$

#### Why $d_{t,i}$ Does Not Constrain the Servo

The lower link is locked to the radial plane by its servo hinge — it **cannot** move tangentially. So how does the arm cope with $d_{t,i} \neq 0$?

The answer is the **ball-and-socket joints**. Both the elbow-to-upper-link joint and the upper-link-to-table joint are spherical — they allow rotation in any direction. This means the upper link is not forced to stay in the radial plane. It can swing sideways to wherever $\mathbf{T}_i$ actually is.

The upper link stretches from the elbow (in the radial plane) to $\mathbf{T}_i$ (which may be slightly off to the side). The ball-and-socket at the elbow lets it point in this off-axis direction freely, without transmitting any force or constraint into the lower link's rotation axis.

**Consequence for the IK**: The servo only needs to position the elbow at the correct $(d_{r,i}, d_{z,i})$ coordinates in the radial plane. The upper link then bridges the remaining 3D gap including $d_{t,i}$ on its own. The IK is therefore genuinely 2D for each arm.

**Small-angle validity**: When $\varphi$ and $\theta$ are small, $d_{t,i}$ is small (it is second-order in the tilt angles). The upper link deviates only slightly from the radial plane, so the approximation of using full $L_2$ as in-plane reach is accurate. At large tilt angles, $d_{t,i}$ becomes non-negligible and slightly reduces the effective in-plane reach of the upper link — this sets a practical limit on the tilt workspace.

The true 3D distance from ground to target is:
$$
\|\mathbf{d}_i\|^2 = d_{r,i}^2 + d_{t,i}^2 + d_{z,i}^2
$$

The in-plane distance the arm must cover (used in the IK):
$$
D_i = \sqrt{d_{r,i}^2 + d_{z,i}^2}
$$

The exact constraint (if we wanted to account for $d_{t,i}$) would be that the upper link length projected onto the radial plane satisfies $L_{2,\text{eff}} = \sqrt{L_2^2 - d_{t,i}^2}$. Since $d_{t,i} \approx 0$ for small angles, we use $L_{2,\text{eff}} \approx L_2$.

Substituting the expressions for $\mathbf{T}_i$ and $\mathbf{G}_i$:

$$
\begin{align}
T_{i,x} - G_{i,x} &= R_t\cos\theta\cos\psi_i - R_g\cos\psi_i = (R_t\cos\theta - R_g)\cos\psi_i \\
T_{i,y} - G_{i,y} &= R_t(\sin\theta\sin\varphi\cos\psi_i + \cos\varphi\sin\psi_i) - R_g\sin\psi_i
\end{align}
$$

Therefore, the radial component:

$$
\begin{align}
d_{r,i} &= (T_{i,x} - G_{i,x})\cos\psi_i + (T_{i,y} - G_{i,y})\sin\psi_i \\
&= (R_t\cos\theta - R_g)\cos^2\psi_i + R_t(\sin\theta\sin\varphi\cos\psi_i + \cos\varphi\sin\psi_i)\sin\psi_i - R_g\sin^2\psi_i
\end{align}
$$

Grouping $-R_g(\cos^2\psi_i + \sin^2\psi_i) = -R_g$:

$$
\boxed{d_{r,i} = R_t\cos\theta\cos^2\psi_i + R_t(\sin\theta\sin\varphi\cos\psi_i + \cos\varphi\sin\psi_i)\sin\psi_i - R_g}
$$

And the vertical component (the table attachment point's z-coordinate, since the ground points are at $z=0$):

$$
\boxed{d_{z,i} = z_t + R_t(-\sin\theta\cos\varphi\cos\psi_i + \sin\varphi\sin\psi_i)}
$$

The effective 2D problem in the arm's radial plane is now:
- Ground point at origin $(0,\ 0)$
- Elbow at $(L_1\cos\alpha_i,\ L_1\sin\alpha_i)$
- Target at $(d_{r,i},\ d_{z,i})$
- Lower link length $L_1$, upper link length $L_2$

### Step 3: Solve the 2-Link Planar IK

In the arm's 2D radial plane, the elbow is at position $(L_1\cos\alpha_i,\ L_1\sin\alpha_i)$ and the target is at $(d_{r,i},\ d_{z,i})$. The upper link must bridge between them with length exactly $L_2$, so:

$$
\|\text{target} - \text{elbow}\|^2 = L_2^2
$$
$$
(d_{r,i} - L_1\cos\alpha_i)^2 + (d_{z,i} - L_1\sin\alpha_i)^2 = L_2^2
$$

Expanding the squares:

$$
d_{r,i}^2 - 2d_{r,i}L_1\cos\alpha_i + L_1^2\cos^2\alpha_i + d_{z,i}^2 - 2d_{z,i}L_1\sin\alpha_i + L_1^2\sin^2\alpha_i = L_2^2
$$

The two $L_1^2$ terms combine using $\cos^2\alpha_i + \sin^2\alpha_i = 1$:

$$
d_{r,i}^2 + d_{z,i}^2 + L_1^2 - 2L_1(d_{r,i}\cos\alpha_i + d_{z,i}\sin\alpha_i) = L_2^2
$$

Moving the known terms to the right-hand side:

$$
2L_1(d_{r,i}\cos\alpha_i + d_{z,i}\sin\alpha_i) = d_{r,i}^2 + d_{z,i}^2 + L_1^2 - L_2^2
$$
$$
d_{r,i}\cos\alpha_i + d_{z,i}\sin\alpha_i = \frac{d_{r,i}^2 + d_{z,i}^2 + L_1^2 - L_2^2}{2L_1}
$$

**Geometric interpretation of the right-hand side**: By the law of cosines applied to the triangle formed by the ground point, elbow, and target, the angle at the elbow satisfies:

$$
L_2^2 = L_1^2 + D_i^2 - 2L_1 D_i \cos\beta_i
$$

where $D_i = \sqrt{d_{r,i}^2 + d_{z,i}^2}$ is the ground-to-target distance and $\beta_i$ is the angle at the elbow between the lower link and the line from ground to target. This is geometrically equivalent to what we derived algebraically.

Defining shorthand:
$$
A_i = d_{r,i}, \quad B_i = d_{z,i}, \quad C_i = \frac{d_{r,i}^2 + d_{z,i}^2 + L_1^2 - L_2^2}{2L_1}
$$

the constraint reduces to:
$$
A_i \cos\alpha_i + B_i \sin\alpha_i = C_i
$$

### Step 4: Solve $A\cos\alpha + B\sin\alpha = C$

This is a standard trigonometric equation. The key idea is to recognise that $A\cos\alpha + B\sin\alpha$ is a sinusoid in $\alpha$ with amplitude $\sqrt{A^2+B^2}$ — we can write it as a single cosine with a phase shift.

#### Auxiliary Angle Method

Any expression of the form $A\cos\alpha + B\sin\alpha$ can be written as:
$$
A\cos\alpha + B\sin\alpha = \sqrt{A^2+B^2}\cos(\alpha - \phi)
$$

To verify, expand the right side:
$$
\sqrt{A^2+B^2}(\cos\alpha\cos\phi + \sin\alpha\sin\phi) = A\cos\alpha + B\sin\alpha
$$

which matches when we set:
$$
\cos\phi = \frac{A}{\sqrt{A^2+B^2}}, \quad \sin\phi = \frac{B}{\sqrt{A^2+B^2}} \quad \Rightarrow \quad \phi = \text{atan2}(B, A)
$$

So $\phi_i = \text{atan2}(d_{z,i},\ d_{r,i})$ is simply the **angle of the line from the ground point to the target**, measured from the horizontal. This has a clear geometric meaning: it is the direction you would need to point the lower link if it reached all the way to the target in a straight line.

#### Solving for $\alpha_i$

Substituting into the constraint:
$$
\sqrt{A_i^2 + B_i^2}\,\cos(\alpha_i - \phi_i) = C_i
$$
$$
\cos(\alpha_i - \phi_i) = \frac{C_i}{\sqrt{A_i^2 + B_i^2}} = \frac{C_i}{D_i}
$$

where $D_i = \sqrt{d_{r,i}^2 + d_{z,i}^2}$ is the straight-line distance from the ground point to the target.

**Reachability condition**: A solution exists if and only if the argument of $\arccos$ is in $[-1, 1]$:
$$
\left|\frac{C_i}{D_i}\right| \leq 1
$$

This is equivalent to the triangle inequality $|L_1 - L_2| \leq D_i \leq L_1 + L_2$ — the arm can reach the target only if the target is neither too close nor too far away.

Since $\cos$ is an even function, there are exactly **two solutions**:
$$
\alpha_i - \phi_i = \pm\arccos\!\left(\frac{C_i}{D_i}\right)
$$

$$
\boxed{
\alpha_i = \phi_i \pm \arccos\!\left(\frac{C_i}{D_i}\right), \quad \phi_i = \text{atan2}(d_{z,i},\, d_{r,i}), \quad D_i = \sqrt{d_{r,i}^2 + d_{z,i}^2}
}
$$

#### Geometric Interpretation of the Two Solutions

The angle $\delta_i = \arccos(C_i/D_i)$ is the angular offset the lower link must make from the ground-to-target direction. The two solutions $\alpha_i = \phi_i \pm \delta_i$ correspond to two physically distinct arm configurations:

- **$\alpha_i = \phi_i + \delta_i$** (elbow-down): The lower link points above the ground-to-target line. The elbow is positioned below the target, and the upper link points upward to reach it.
- **$\alpha_i = \phi_i - \delta_i$** (elbow-up): The lower link points below the ground-to-target line. The elbow is above the target, and the upper link points downward.

For a table-balancing application where the table sits above the arm mounting points, the **elbow-down** branch ($+$ sign) is the physically natural configuration. The correct branch must be consistent across all three arms and fixed for the entire operating range to avoid singularities.

### Summary: Inverse Kinematics

Given $(\varphi, \theta, z_t)$, for each arm $i \in \{1, 2, 3\}$:

$$
\boxed{
\begin{aligned}
&1.\ \text{Compute } \mathbf{T}_i = \mathbf{c} + R(\varphi,\theta)\cdot R_t\hat{\mathbf{p}}_i \\
&2.\ d_{r,i} = (\mathbf{T}_i - \mathbf{G}_i)\cdot\hat{\mathbf{r}}_i, \quad d_{z,i} = T_{i,z} \\
&3.\ C_i = \frac{d_{r,i}^2 + d_{z,i}^2 + L_1^2 - L_2^2}{2L_1} \\
&4.\ \phi_i = \text{atan2}(d_{z,i},\ d_{r,i}) \\
&5.\ \alpha_i = \phi_i \pm \arccos\!\left(\frac{C_i}{\sqrt{d_{r,i}^2 + d_{z,i}^2}}\right)
\end{aligned}
}
$$
---

## Forward Kinematics — Solving for Table Pose

**Problem**: Given the three servo angles $(\alpha_1, \alpha_2, \alpha_3)$, find the table pose $(\varphi, \theta, z_t)$.

Forward kinematics for parallel mechanisms generally has **no closed-form solution** and is solved numerically. The approach is to find $(\varphi, \theta, z_t)$ such that all three arm constraints are simultaneously satisfied.

### Constraint Equations

For each arm $i$, the elbow position is known directly from the servo angle:
$$
\mathbf{E}_i(\alpha_i) = \mathbf{G}_i + L_1 \begin{bmatrix}
\cos\alpha_i \cos\psi_i \\
\cos\alpha_i \sin\psi_i \\
\sin\alpha_i
\end{bmatrix}
$$

The table attachment point $\mathbf{T}_i(\varphi, \theta, z_t)$ is defined by the pose (as derived earlier). The constraint for each arm is:

$$
f_i(\varphi, \theta, z_t) = \|\mathbf{T}_i(\varphi, \theta, z_t) - \mathbf{E}_i(\alpha_i)\|^2 - L_2^2 = 0
$$

This gives the system of three nonlinear equations:

$$
\begin{cases}
f_1(\varphi, \theta, z_t) = 0 \\
f_2(\varphi, \theta, z_t) = 0 \\
f_3(\varphi, \theta, z_t) = 0
\end{cases}
$$

### Numerical Solution: Newton-Raphson

Define the residual vector:
$$
\mathbf{f}(\mathbf{q}) = \begin{bmatrix} f_1 \\ f_2 \\ f_3 \end{bmatrix}, \quad \mathbf{q} = \begin{bmatrix} \varphi \\ \theta \\ z_t \end{bmatrix}
$$

The Newton-Raphson iteration:
$$
\mathbf{q}^{(k+1)} = \mathbf{q}^{(k)} - J^{-1}(\mathbf{q}^{(k)}) \cdot \mathbf{f}(\mathbf{q}^{(k)})
$$

where $J$ is the $3\times 3$ Jacobian matrix:

$$
J = \frac{\partial \mathbf{f}}{\partial \mathbf{q}} = \begin{bmatrix}
\partial f_1/\partial\varphi & \partial f_1/\partial\theta & \partial f_1/\partial z_t \\
\partial f_2/\partial\varphi & \partial f_2/\partial\theta & \partial f_2/\partial z_t \\
\partial f_3/\partial\varphi & \partial f_3/\partial\theta & \partial f_3/\partial z_t
\end{bmatrix}
$$

Each partial derivative:

$$
\frac{\partial f_i}{\partial q_j} = 2(\mathbf{T}_i - \mathbf{E}_i) \cdot \frac{\partial \mathbf{T}_i}{\partial q_j}
$$

The partial derivatives of $\mathbf{T}_i$ with respect to the pose variables are derived from $\mathbf{T}_i = \mathbf{c} + R(\varphi,\theta)\cdot\mathbf{P}_i^T$:

$$
\frac{\partial \mathbf{T}_i}{\partial z_t} = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
$$

$$
\frac{\partial \mathbf{T}_i}{\partial \varphi} = \frac{\partial R}{\partial \varphi} \cdot \mathbf{P}_i^T, \quad
\frac{\partial \mathbf{T}_i}{\partial \theta} = \frac{\partial R}{\partial \theta} \cdot \mathbf{P}_i^T
$$

where the rotation matrix partial derivatives are:
$$
R_x(\varphi) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos(\varphi) & -\sin(\varphi) \\
0 & \sin(\varphi) & \cos(\varphi)
\end{bmatrix} \quad , \quad R_y(\theta) = \begin{bmatrix}
\cos(\theta) & 0 & \sin(\theta) \\
0 & 1 & 0 \\
-\sin(\theta) & 0 & \cos(\theta)
\end{bmatrix}
$$
$$
\frac{\partial R_x(\varphi)}{\partial \varphi} = \begin{bmatrix}
0 & 0 & 0 \\
0 & -\sin(\varphi) & -\cos(\varphi) \\
0 & \cos(\varphi) & -\sin(\varphi)
\end{bmatrix} \quad , \quad \frac{\partial R_y(\theta)}{\partial \theta} = \begin{bmatrix}
-\sin(\theta) & 0 & \cos(\theta) \\
0 & 0 & 0 \\
-\cos(\theta) & 0 & -\sin(\theta)
\end{bmatrix}
$$

$$
\frac{\partial R}{\partial \varphi} = R_y(\theta) \cdot \frac{\partial R_x(\varphi)}{\partial \varphi} = \begin{bmatrix}
0 & 0 & 0 \\
\sin\theta\cos\varphi & -\sin\varphi & \cos\theta\cos\varphi \\
\sin\theta\sin\varphi & \cos\varphi & -\cos\theta\sin\varphi
\end{bmatrix}
$$

$$
\frac{\partial R}{\partial \theta} = \frac{\partial R_y(\theta)}{\partial \theta} \cdot R_x(\varphi) = \begin{bmatrix}
-\sin\theta & 0 & \cos\theta \\
\cos\theta\sin\varphi & 0 & \sin\theta\sin\varphi \\
-\cos\theta\cos\varphi & 0 & -\sin\theta\cos\varphi
\end{bmatrix}
$$

**Initial guess** for the Newton-Raphson iteration: use the previous timestep's solution (warm start), or use the neutral pose $(\varphi, \theta, z_t) = (0, 0, z_{\text{nom}})$ where $z_{\text{nom}}$ is the nominal table height.

---

## Workspace and Reachability

The table pose $(\varphi, \theta, z_t)$ is reachable if and only if all three arms can simultaneously reach their target attachment points. For each arm $i$:

$$
|d_{r,i}^2 + d_{z,i}^2 + L_1^2 - L_2^2| \leq 2L_1\sqrt{d_{r,i}^2 + d_{z,i}^2}
$$

This simplifies to the triangle inequality: the three lengths $L_1$, $L_2$, and $\sqrt{d_{r,i}^2 + d_{z,i}^2}$ must form a valid triangle:

$$
|L_1 - L_2| \leq \sqrt{d_{r,i}^2 + d_{z,i}^2} \leq L_1 + L_2
$$

The **workspace** is the set of all $(\varphi, \theta, z_t)$ satisfying this condition for all three arms simultaneously.

---

## Small Angle Approximation

For small tilt angles ($\varphi \ll 1$, $\theta \ll 1$), the table attachment points simplify to:

$$
\mathbf{T}_i \approx \begin{bmatrix}
R_t \cos\psi_i \\
R_t \sin\psi_i \\
z_t - R_t(\theta\cos\psi_i - \varphi\sin\psi_i)
\end{bmatrix}
$$

This uses $\cos\theta \approx 1$, $\sin\theta \approx \theta$, $\cos\varphi \approx 1$, $\sin\varphi \approx \varphi$, and neglects second-order terms $\theta\varphi$.

The radial and vertical displacements become:

$$
d_{r,i} \approx R_t - R_g \quad \text{(constant, independent of pose!)}
$$

$$
d_{z,i} \approx z_t - R_t(\theta\cos\psi_i - \varphi\sin\psi_i)
$$

**Physical insight**: For small angles, the radial position of each table attachment point barely changes — only the height $d_{z,i}$ varies with pose. The three heights are:

$$
\begin{aligned}
d_{z,1} &\approx z_t - R_t\theta \\
d_{z,2} &\approx z_t + R_t\left(\frac{\theta}{2} + \frac{\sqrt{3}}{2}\varphi\right) \\
d_{z,3} &\approx z_t + R_t\left(\frac{\theta}{2} - \frac{\sqrt{3}}{2}\varphi\right)
\end{aligned}
$$

These can be inverted to express the table pose directly from the three attachment heights:

$$
\boxed{
\begin{aligned}
z_t &= \frac{d_{z,1} + d_{z,2} + d_{z,3}}{3} \\
\theta &\approx \frac{d_{z,2} + d_{z,3} - 2d_{z,1}}{3R_t} \\
\varphi &\approx \frac{d_{z,2} - d_{z,3}}{\sqrt{3}\,R_t}
\end{aligned}
}
$$

This linear relationship is very useful for **control design**: the table pose is approximately a linear transformation of the three arm heights.

---

## Linearized Servo Angle to Height Mapping

Under the small-angle approximation, with $d_{r,i} \approx \Delta R = R_t - R_g$ (constant), the arm constraint simplifies to a 2-link planar problem with fixed horizontal reach. Define $h_i = d_{z,i}$ as the height arm $i$ must achieve.

The servo angle required:

$$
\alpha_i = \phi_i \pm \arccos\!\left(\frac{\Delta R^2 + h_i^2 + L_1^2 - L_2^2}{2L_1\sqrt{\Delta R^2 + h_i^2}}\right)
$$

where $\phi_i = \text{atan2}(h_i, \Delta R)$.

For small deviations around a nominal height $h_0$ (i.e., the flat table position), we can linearize:

$$
\alpha_i \approx \alpha_0 + \frac{d\alpha}{dh}\bigg|_{h=h_0} \cdot (h_i - h_0)
$$

The linearization coefficient:

$$
\frac{d\alpha}{dh}\bigg|_{h=h_0} = \frac{1}{\sqrt{1 - \left(\frac{C_0}{D_0}\right)^2}} \cdot \frac{d}{dh}\left(\frac{C(h)}{D(h)}\right)\bigg|_{h=h_0}
$$

where $D(h) = \sqrt{\Delta R^2 + h^2}$ and $C(h) = (\Delta R^2 + h^2 + L_1^2 - L_2^2)/(2L_1)$.

This gives a locally **linear mapping** from pose deviations $(\delta\varphi, \delta\theta, \delta z_t)$ to servo angle deviations $(\delta\alpha_1, \delta\alpha_2, \delta\alpha_3)$, which is suitable for implementing a linear controller around the operating point.

---

## Summary of Key Equations

### Geometry

$$
\boxed{
\psi_i = \frac{2\pi(i-1)}{3}, \quad i = 1, 2, 3
}
$$

$$
\boxed{
\mathbf{G}_i = R_g\begin{bmatrix}\cos\psi_i \\ \sin\psi_i \\ 0\end{bmatrix}, \quad
\mathbf{T}_i = \begin{bmatrix}0\\0\\z_t\end{bmatrix} + R(\varphi,\theta)\cdot R_t\begin{bmatrix}\cos\psi_i\\\sin\psi_i\\0\end{bmatrix}
}
$$

### Inverse Kinematics (exact)

$$
\boxed{
\alpha_i = \text{atan2}(d_{z,i},\, d_{r,i}) \pm \arccos\!\left(\frac{d_{r,i}^2 + d_{z,i}^2 + L_1^2 - L_2^2}{2L_1\sqrt{d_{r,i}^2 + d_{z,i}^2}}\right)
}
$$

where:
$$
d_{r,i} = (\mathbf{T}_i - \mathbf{G}_i)\cdot\hat{\mathbf{r}}_i, \quad d_{z,i} = T_{i,z}
$$

### Forward Kinematics (numerical)

Solve $\mathbf{f}(\mathbf{q}) = \mathbf{0}$ via Newton-Raphson:

$$
\boxed{
\mathbf{q}^{(k+1)} = \mathbf{q}^{(k)} - J^{-1}\mathbf{f}(\mathbf{q}^{(k)}), \quad \mathbf{q} = \begin{bmatrix}\varphi\\\theta\\z_t\end{bmatrix}
}
$$

where $f_i = \|\mathbf{T}_i(\mathbf{q}) - \mathbf{E}_i(\alpha_i)\|^2 - L_2^2$.

### Small-Angle Pose from Attachment Heights

$$
\boxed{
z_t = \frac{d_{z,1} + d_{z,2} + d_{z,3}}{3}, \quad
\theta \approx \frac{d_{z,2} + d_{z,3} - 2d_{z,1}}{3R_t}, \quad
\varphi \approx \frac{d_{z,2} - d_{z,3}}{\sqrt{3}\,R_t}
}
$$

### Reachability Condition (per arm)

$$
\boxed{
|L_1 - L_2| \leq \sqrt{d_{r,i}^2 + d_{z,i}^2} \leq L_1 + L_2
}
$$

---

## Alternative Formulation: Normal-Vector / Height-Based Model

This section documents an alternative kinematic model for the same 3-arm mechanism, as presented in a YouTube derivation. It uses a different parameterisation of the table orientation and a different coordinate convention per arm. The two models are equivalent — they describe the same mechanism — but the normal-vector model yields a **closed-form forward kinematics** at the cost of a less intuitive representation of table orientation.

### Notation Differences from This Document

The YouTube model uses a **local arm frame** for each arm rather than the global inertial frame. Each arm is treated independently in its own XZ-plane, with:

- **$L_3$**: horizontal distance from the arm's coordinate origin (motor pivot) to the reference point along the x-axis (analogous to $R_g$ in our notation — the radial offset to the ground attachment)
- **$L_1$**: upper link length from the motor pivot to the ball joint (analogous to our $L_2$)
- **$L_2$** (in our notation): lower link from ground to motor pivot (analogous to our $L_1$, but not directly represented — the model works from the motor pivot outward)
- **$\theta$**: servo angle measured from horizontal at the motor pivot (analogous to our $\alpha_i$)
- **$\mathbf{n} = [\alpha, \beta, \gamma]^T$**: normal vector to the table plane (replaces our Euler angles $\varphi$, $\theta$)
- **$h$**: signed height of the table plane at the coordinate origin

> **Correspondence to our notation**: YouTube's $L_3 \leftrightarrow R_g$, $L_1 \leftrightarrow L_2$, $\theta \leftrightarrow \alpha_i$.

---

### Arm Coordinate Frame and Forward Position

![[diagrams/youtube-model-single-arm-xz.svg]]

Each arm is analysed in its own **XZ-plane** with:
- Origin at the motor pivot
- $x$ pointing **outward** (away from the table centre, i.e. in the $-\hat{\mathbf{r}}_i$ direction of our notation)
- $z$ pointing **upward**

In this 2D local frame the ball joint position $P = (x_P, 0, z_P)$ is given by forward kinematics of the single upper link:

$$
\boxed{
x_P = L_3 + L_1 \cos\theta, \quad z_P = L_1 \sin\theta
}
$$

where $L_3$ is the horizontal offset from the motor pivot to the table attachment reference. Inverting to recover the servo angle:

$$
\cos\theta = \frac{x_P - L_3}{L_1}, \quad \sin\theta = \frac{z_P}{L_1} \quad \Rightarrow \quad \boxed{\theta = \text{atan2}(z_P,\; x_P - L_3)}
$$

---

### Table Plane Representation

Instead of Euler angles $(\varphi, \theta, z_t)$, the table is described by its **plane equation**:

$$
\alpha x + \beta y + \gamma(z - h) = 0
$$

where $\mathbf{n} = [\alpha, \beta, \gamma]^T$ is the (unnormalised) normal to the table plane and $h$ is the height of the plane at $x = y = 0$. The normal is characterised so that $\mathbf{n}^T \cdot \mathbf{P}_i = 0$ for any point $\mathbf{P}_i$ on the table plane.

**Plane constraint for a single ball joint** (in the arm's local XZ-plane with $y = 0$):

$$
\alpha x + \gamma(z - h) = 0 \quad \Rightarrow \quad z = h - \frac{\alpha}{\gamma}\,x
$$

Combined with the link-length constraint $L_1^2 = x^2 + (h - z)^2$ (Pythagorean theorem in the XZ-plane), eliminating $z$:

$$
x^2 + (h - z)^2 = L_1^2 \quad \text{and} \quad x = \sqrt{L_1^2 - (h-z)^2} \quad (\text{since } x > 0)
$$

Substituting the plane constraint $\sqrt{L_1^2 - (h-z)^2} = -\dfrac{\gamma}{\alpha}(z - h)$ and squaring:

$$
L_1^2 - (h-z)^2 = \frac{\gamma^2}{\alpha^2}(z-h)^2 \quad \Rightarrow \quad \frac{\alpha^2 + \gamma^2}{\alpha^2}(z-h)^2 = L_1^2
$$

$$
\boxed{
z = h - \frac{\alpha L_1}{\sqrt{\alpha^2 + \gamma^2}}, \qquad
x = \sqrt{L_1^2 - (h - z)^2} = \frac{\gamma L_1}{\sqrt{\alpha^2 + \gamma^2}}
}
$$

(choosing the physically meaningful sign: $x > 0$ since the ball joint lies outward from the pivot.)

---

### Full 3-Arm Forward Kinematics (Closed Form)

For the full 3-arm mechanism the three ball joint positions in a **shared** coordinate system satisfy an additional geometric constraint: successive ball joints lie at a fixed angular spacing ($120°$), so their positions are related by:

$$
y_2 = \sqrt{3}\,x_2, \qquad y_3 = -\sqrt{3}\,x_3
$$

(arms 2 and 3 being $\pm 120°$ from arm 1). Combining the three arm constraints with the table-plane constraint and setting $y_1 = 0$ (arm 1 lies in the XZ-plane) yields the coupled system:

$$
\begin{cases}
(x - a)^2 + (y - b)^2 + (z - c)^2 = L_1^2 \\[4pt]
(x - L_3)^2 + y^2 + z^2 = L_2^2 \\[4pt]
y = 0
\end{cases}
$$

where $[a, b, c]$ is the elbow/intermediate joint position and $L_2$ is the lower link length (ground to motor pivot).

Subtracting the second equation from the first (eliminating $x^2$, $y^2$, $z^2$ terms with $y = 0$):

$$
-2ax + a^2 + L_3^2 - 2L_3 x - 2cz + c^2 = L_1^2 - L_2^2
$$

Solving for $z$ from the second equation: $z = \pm\sqrt{L_2^2 - (x - L_3)^2}$, substituting, and collecting terms gives:

$$
\sqrt{L_2^2 - (x - L_3)^2} = Ax - B, \quad \text{where} \quad A = \frac{a - L_3}{c}, \quad B = \frac{a^2 + c^2 + L_2^2 - L_1^2 - L_3^2}{2c}
$$

Squaring both sides:

$$
L_2^2 - (x - L_3)^2 = (Ax - B)^2 \quad \Rightarrow \quad (A^2 + 1)x^2 + 2(AB + L_3)x + (B^2 - L_2^2 - L_3^2) = 0
$$

Defining:

$$
D = A^2 + 1, \qquad E = 2(AB + L_3), \qquad F = B^2 - L_2^2 - L_3^2
$$

the ball joint x-coordinate follows from the quadratic formula:

$$
\boxed{x = \frac{-E + \sqrt{E^2 - 4DF}}{2D}}
$$

(taking the $+$ root to ensure the physically meaningful $x > 0$ solution). The remaining coordinates:

$$
\boxed{z = \sqrt{L_2^2 - (x - L_3)^2}}
$$

For the full three-arm system (all three ball joints simultaneously), the three-arm version replaces the single-arm coefficients with:

$$
A = -\frac{\alpha + \sqrt{3}\,\beta + 2L_3}{\gamma}, \qquad
B = \frac{a^2 + b^2 + c^2 + L_2^2 - L_1^2 - L_3^2}{2\gamma}
$$

$$
C = A^2 + 4, \qquad D = 2AB + 4L_3, \qquad E = B^2 + L_3^2 - L_2^2
$$

$$
\boxed{
x = \frac{-D - \sqrt{D^2 - 4CE}}{2C}, \qquad
y = \sqrt{3}\,x, \qquad
z = \sqrt{L_2^2 - 4x^2 - 4L_3 x - L_3^2}
}
$$

The servo angle for each arm is then recovered from:

$$
\boxed{\theta = 90° - \text{atan2}\!\left(\sqrt{x^2 + y^2} - L_3,\; z\right)}
$$

---

### Comparison with the Euler-Angle Model in This Document

| Aspect | This document (Euler angles) | YouTube model (normal vector) |
|---|---|---|
| Table orientation | Roll/pitch $(\varphi, \theta)$ | Normal vector $[\alpha, \beta, \gamma]$ |
| IK input | Desired pose $(\varphi, \theta, z_t)$ | Desired $(\alpha, \beta, \gamma, h)$ |
| IK method | Geometric, arm-by-arm in radial plane | Algebraic constraint intersection |
| FK method | Newton-Raphson (numerical) | Closed-form quadratic formula |
| Control integration | Direct — $(\varphi, \theta)$ appear in ball EOM | Requires conversion to Euler angles first |
| Small-angle linearisation | Derived explicitly | Not present |
| Singularity analysis | Explicit reachability condition | Implicit in quadratic discriminant |

**Key trade-off**: The normal-vector model gives an elegant **closed-form FK** (no Newton-Raphson), which is advantageous for embedded real-time computation. The Euler-angle model is preferable for control design because $(\varphi, \theta)$ couple directly to the ball equations of motion ($\ddot{x}_b \approx -g\theta$, $\ddot{y}_b \approx g\varphi$) and the small-angle linearisation yields a clean linear state-space model.

In practice the two approaches can be combined: use the closed-form quadratic FK for real-time state estimation and fall back to Newton-Raphson near singularities.

---

## C++ Implementation — `TableKinematics`

This section documents the complete C++ implementation for the project at `/home/nds/Projects/ball-balancer/`. The class lives in:

```
include/ball_balancer/physics/table_kinematics.hpp
src/physics/table_kinematics.cpp
```

### Design Overview

The `TableKinematics` class is **stateless** — it contains only the mechanism parameters and provides pure functions. Callers pass pose or servo angles in and receive the result back. This mirrors the existing `BallDynamics` design pattern in the project.

Two FK implementations are provided, selectable at call time via a `FKMethod` enum:

| Enum value | Algorithm | Notes |
|---|---|---|
| `FKMethod::NewtonRaphson` | 3×3 Newton-Raphson (our model) | Robust, warm-start from previous timestep |
| `FKMethod::YouTubeClosedForm` | Quadratic formula (YouTube model) | No iteration; may be near-singular at large tilt |

For IK there is a single implementation (exact, per-arm, both models reduce to identical IK once $\mathbf{T}_i$ and $\mathbf{G}_i$ are in the inertial frame).

---

### Mechanism Parameters Added to `SystemParameters`

The following fields must be added to `SystemParameters` in `types.hpp` to use `TableKinematics`. They have no effect on the existing ball dynamics.

```cpp
// Mechanism geometry (table arm kinematics)
double arm_L1{0.08};          // Lower link length (servo to elbow), m
double arm_L2{0.08};          // Upper link length (elbow to table point), m
double arm_Rg{0.10};          // Ground mounting radius (centre to arm pivot), m
double arm_Rt{0.07};          // Table mounting radius (centre to table attach), m
double arm_z_nominal{0.12};   // Nominal table height (flat pose, for FK warm-start), m
```

---

### Header: `table_kinematics.hpp`

```cpp
#pragma once

#include <ball_balancer/core/types.hpp>
#include <array>
#include <cmath>
#include <optional>

namespace ball_balancer {

/**
 * @brief Servo angle triple for the three arms.
 *
 * alpha[0..2] — servo angles α₁, α₂, α₃ in radians, measured from
 * horizontal (positive = elbow above ground mounting point).
 */
struct ServoAngles {
    std::array<double, 3> alpha{};
};

/**
 * @brief Selects which forward-kinematics algorithm to use.
 */
enum class FKMethod {
    NewtonRaphson,       ///< Iterative 3×3 Newton-Raphson (robust, exact)
    YouTubeClosedForm,   ///< Closed-form quadratic formula (fast, no iteration)
};

/**
 * @brief Kinematics of the 3-arm parallel table mechanism.
 *
 * Converts between servo angles (α₁, α₂, α₃) and table pose (φ, θ, z_t).
 *
 * Two modes of forward kinematics are available:
 *  - NewtonRaphson      — solves the 3×3 nonlinear constraint system iteratively
 *  - YouTubeClosedForm  — closed-form via quadratic formula (no iteration)
 *
 * Inverse kinematics is exact and identical regardless of FK model:
 *   αᵢ = atan2(d_z,i, d_r,i) ± arccos((d_r² + d_z² + L₁² − L₂²) / (2L₁D))
 * where D = sqrt(d_r² + d_z²).
 *
 * Arm indexing and coordinate convention:
 *   Arm 1: ψ₁ = 0°     → ground point on +X axis
 *   Arm 2: ψ₂ = 120°
 *   Arm 3: ψ₃ = 240°
 *   Z up, inertial origin at centre of ground mounting circle.
 *
 * @see Table Dynamics Equations - Ball Balancing System.md
 */
class TableKinematics {
public:
    /// Maximum Newton-Raphson iterations before giving up.
    static constexpr int  NR_MAX_ITER   = 20;
    /// Newton-Raphson convergence threshold (residual norm).
    static constexpr double NR_TOL      = 1e-10;
    /// Number of arms.
    static constexpr int  N_ARMS        = 3;

    /**
     * @brief Construct from system parameters.
     * @param params Must have arm_L1, arm_L2, arm_Rg, arm_Rt, arm_z_nominal set.
     */
    explicit TableKinematics(const SystemParameters& params);

    // -------------------------------------------------------------------------
    // Inverse Kinematics
    // -------------------------------------------------------------------------

    /**
     * @brief Compute servo angles from a desired table pose.
     *
     * Exact closed-form solution per arm:
     *   αᵢ = atan2(d_z,i, d_r,i) + arccos(Cᵢ / Dᵢ)   [elbow-down branch]
     *
     * Returns std::nullopt if any arm is out of reach (triangle inequality
     * violated: |L₁ − L₂| > Dᵢ or Dᵢ > L₁ + L₂).
     *
     * @param phi    Roll angle φ (rad)
     * @param theta  Pitch angle θ (rad)
     * @param z_t    Table height z_t (m)
     * @return Servo angles, or nullopt if the pose is unreachable.
     */
    std::optional<ServoAngles> inverseKinematics(
        double phi, double theta, double z_t) const;

    // -------------------------------------------------------------------------
    // Forward Kinematics
    // -------------------------------------------------------------------------

    /**
     * @brief Compute table pose from servo angles.
     *
     * @param servos  Three servo angles (rad)
     * @param method  Which FK algorithm to use (default: NewtonRaphson)
     * @param phi0    Warm-start roll (rad)  — ignored for YouTubeClosedForm
     * @param theta0  Warm-start pitch (rad) — ignored for YouTubeClosedForm
     * @param z0      Warm-start height (m)  — ignored for YouTubeClosedForm
     * @return { phi, theta, z_t }, or nullopt on failure.
     */
    std::optional<std::array<double, 3>> forwardKinematics(
        const ServoAngles& servos,
        FKMethod method    = FKMethod::NewtonRaphson,
        double phi0        = 0.0,
        double theta0      = 0.0,
        double z0          = -1.0   // negative → use arm_z_nominal
    ) const;

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    double L1()  const { return L1_; }
    double L2()  const { return L2_; }
    double Rg()  const { return Rg_; }
    double Rt()  const { return Rt_; }

private:
    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    /**
     * @brief Compute table attachment point Tᵢ in the inertial frame.
     * @param i     Arm index 0,1,2
     * @param phi   Roll angle (rad)
     * @param theta Pitch angle (rad)
     * @param z_t   Table height (m)
     * @return [x, y, z] of table attachment point
     */
    std::array<double, 3> tableAttachPoint(
        int i, double phi, double theta, double z_t) const;

    /**
     * @brief Compute elbow position Eᵢ in the inertial frame.
     * @param i     Arm index 0,1,2
     * @param alpha Servo angle αᵢ (rad)
     * @return [x, y, z] of elbow
     */
    std::array<double, 3> elbowPosition(int i, double alpha) const;

    /**
     * @brief Per-arm IK: compute αᵢ given d_r and d_z.
     * Returns nullopt if unreachable.
     */
    std::optional<double> solveArmIK(double d_r, double d_z) const;

    /**
     * @brief FK via Newton-Raphson.
     * @param servos   Servo angles
     * @param phi0     Initial guess φ
     * @param theta0   Initial guess θ
     * @param z0       Initial guess z_t
     */
    std::optional<std::array<double, 3>> fkNewtonRaphson(
        const ServoAngles& servos,
        double phi0, double theta0, double z0) const;

    /**
     * @brief FK via closed-form quadratic formula (YouTube model).
     * @param servos  Servo angles
     */
    std::optional<std::array<double, 3>> fkYouTube(
        const ServoAngles& servos) const;

    /**
     * @brief Evaluate constraint residuals fᵢ = ||Tᵢ - Eᵢ||² − L₂².
     */
    std::array<double, 3> residuals(
        const ServoAngles& servos,
        double phi, double theta, double z_t) const;

    /**
     * @brief Evaluate the 3×3 Jacobian J = ∂f/∂(φ, θ, z_t).
     * Each entry: ∂fᵢ/∂qⱼ = 2 (Tᵢ − Eᵢ) · ∂Tᵢ/∂qⱼ
     */
    std::array<std::array<double, 3>, 3> jacobian(
        const ServoAngles& servos,
        double phi, double theta, double z_t) const;

    /**
     * @brief Solve a dense 3×3 linear system Ax = b (Cramer's rule).
     * Returns nullopt if the matrix is singular (|det| < 1e-14).
     */
    static std::optional<std::array<double, 3>> solve3x3(
        const std::array<std::array<double, 3>, 3>& A,
        const std::array<double, 3>& b);

    // Mechanism parameters
    double L1_;        ///< Lower link length (m)
    double L2_;        ///< Upper link length (m)
    double Rg_;        ///< Ground mounting radius (m)
    double Rt_;        ///< Table mounting radius (m)
    double zNominal_;  ///< Nominal table height for FK warm-start (m)

    // Precomputed arm angles ψᵢ = 2π(i-1)/3
    static constexpr double PSI[3] = {
        0.0,
        2.0 * M_PI / 3.0,
        4.0 * M_PI / 3.0
    };
};

} // namespace ball_balancer
```

---

### Implementation: `table_kinematics.cpp`

Key implementation notes for every method:

#### Constructor
Reads `arm_L1`, `arm_L2`, `arm_Rg`, `arm_Rt`, `arm_z_nominal` from `SystemParameters`. Falls back to `arm_z_nominal = 0.12` m if negative.

#### `tableAttachPoint(i, φ, θ, z_t)`
Direct evaluation of the boxed result from the document:
```cpp
// R = Ry(theta) * Rx(phi)
const double cp = cos(phi), sp = sin(phi);
const double ct = cos(theta), st = sin(theta);
const double ci = cos(PSI[i]), si = sin(PSI[i]);

T[0] = Rt_ * ct * ci;
T[1] = Rt_ * (st*sp*ci + cp*si);
T[2] = z_t + Rt_ * (-st*cp*ci + sp*si);
```

#### `elbowPosition(i, αᵢ)`
```cpp
const double ca = cos(alpha), sa = sin(alpha);
const double ci = cos(PSI[i]), si = sin(PSI[i]);
E[0] = (Rg_ + L1_*ca) * ci;
E[1] = (Rg_ + L1_*ca) * si;
E[2] = L1_ * sa;
```

#### `solveArmIK(d_r, d_z)` — per-arm IK
```cpp
const double D2 = d_r*d_r + d_z*d_z;
const double D  = sqrt(D2);
const double C  = (D2 + L1_*L1_ - L2_*L2_) / (2.0*L1_);

// Reachability check
if (std::abs(C) > D + 1e-9) return std::nullopt;  // triangle inequality

const double phi_dir = atan2(d_z, d_r);       // direction to target
const double delta   = acos(std::clamp(C/D, -1.0, 1.0));
return phi_dir + delta;                        // elbow-down branch (+)
```

#### `inverseKinematics(φ, θ, z_t)`
For each arm $i$:
1. Call `tableAttachPoint(i, φ, θ, z_t)` → $\mathbf{T}_i$
2. Compute $\mathbf{d}_i = \mathbf{T}_i - \mathbf{G}_i$
3. Project onto radial/vertical plane: $d_{r,i} = \mathbf{d}_i \cdot \hat{\mathbf{r}}_i$, $d_{z,i} = T_{i,z}$
4. Call `solveArmIK(d_r, d_z)` → $\alpha_i$

Return `nullopt` if any arm is out of reach.

#### `fkNewtonRaphson(servos, φ₀, θ₀, z₀)` — FK via our model
Iterate up to `NR_MAX_ITER`:
```
q ← q₀
loop:
    f ← residuals(servos, q)
    if ||f|| < NR_TOL: return q
    J ← jacobian(servos, q)
    Δq ← solve3x3(J, -f)
    q ← q + Δq
return nullopt  // did not converge
```

The `jacobian()` method uses the analytical partial derivatives $\partial R/\partial\varphi$ and $\partial R/\partial\theta$ derived in the Forward Kinematics section:
```cpp
// dR/dphi:
dRdphi = {{ {0,0,0},
             {st*cp, -sp,  ct*cp},
             {st*sp,  cp, -ct*sp} }};

// dR/dtheta:
dRdtheta = {{ {-st, 0,  ct},
               {ct*sp, 0, st*sp},
               {-ct*cp, 0, -st*cp} }};

// dTi/dqj = dR/dqj * Pt_i   (for phi, theta)
// dTi/dz_t = [0, 0, 1]^T
// dfi/dqj = 2 * dot(Ti - Ei, dTi/dqj)
```

#### `fkYouTube(servos)` — closed-form FK

This method maps the YouTube model's derivation onto the project's existing geometry. Each arm's elbow position is computed from the servo angle (`elbowPosition(i, alpha_i)`). The ball-joint position for arm 1 (the XZ-plane arm) is then recovered from the quadratic constraint intersection between the two sphere equations:

$$
\text{Sphere 1: } \|\mathbf{P} - \mathbf{E}_1\|^2 = L_2^2 \quad\text{(upper link from elbow)}
$$
$$
\text{Sphere 2: } \|\mathbf{P} - \mathbf{G}_1\|^2 = (R_g + L_1 + L_2)^2 \quad\text{(geometric reach)}
$$

With $y = 0$ (arm 1 is in the XZ-plane), the intersection reduces to a quadratic in $x$ exactly as in the YouTube derivation. The resulting $x$, $z$ coordinates of arm 1's table attachment are then used together with arms 2 and 3 (related by the 120° symmetry) to recover $(φ, θ, z_t)$ using the small-angle inverse:

$$
z_t = \frac{z_1 + z_2 + z_3}{3}, \quad
\theta \approx \frac{z_2 + z_3 - 2z_1}{3R_t}, \quad
\varphi \approx \frac{z_2 - z_3}{\sqrt{3}\,R_t}
$$

> **Note**: This last step re-uses the small-angle pose formula from the Small-Angle Approximation section. For large tilt angles the YouTube model must fall back to Newton-Raphson for this final inversion.

#### `solve3x3(A, b)` — 3×3 linear solve
Uses Cramer's rule (no dependencies, fast for fixed 3×3):
```cpp
const double det = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
                 - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
                 + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
if (std::abs(det) < 1e-14) return std::nullopt;
// ... Cramer's rule for each component
```

---

### Integration with the Simulation Loop

The `TableKinematics` class plugs into the existing `Simulator` in two places:

**1. After each `step()` call — FK to get pose for rendering**

The simulator's state already carries $(φ, θ, z_t)$ directly (from the servo dynamics). `TableKinematics::forwardKinematics` is therefore not needed inside the simulation loop itself — but it can be used to:
- Verify that the servo angles fed to a real robot match what the simulator assumes
- Drive the 3D OpenGL rendering of the arm mechanism

**2. Before generating servo commands — IK for the controller output**

The controller produces a desired table pose $(φ^*, θ^*, z_t^*)$ (from the PID/LQR output). This is converted to servo commands via IK:

```cpp
// In control loop, after controller_.compute():
auto servos = table_kin_.inverseKinematics(phi_cmd, theta_cmd, z_cmd);
if (servos) {
    // Send servos->alpha[0..2] to hardware
}
```

The `ControlVector` in the existing code already stores `(φ_cmd, θ_cmd, z_cmd)` — the IK adds the final mapping to physical servo angles without changing any existing interface.

---

### Usage Example

```cpp
#include <ball_balancer/physics/table_kinematics.hpp>

// Setup
SystemParameters params;
params.arm_L1 = 0.08;
params.arm_L2 = 0.08;
params.arm_Rg = 0.10;
params.arm_Rt = 0.07;
params.arm_z_nominal = 0.12;
TableKinematics kin(params);

// Inverse kinematics: desired pose → servo angles
auto servos = kin.inverseKinematics(0.05, -0.03, 0.12);
if (servos) {
    // servos->alpha[0], [1], [2] are the three servo commands in rad
}

// Forward kinematics — Newton-Raphson (with warm-start from previous step)
ServoAngles measured = { {1.1, 0.9, 1.0} };
auto pose_nr = kin.forwardKinematics(measured,
                   FKMethod::NewtonRaphson,
                   prev_phi, prev_theta, prev_z);

// Forward kinematics — YouTube closed-form (no iteration)
auto pose_yt = kin.forwardKinematics(measured, FKMethod::YouTubeClosedForm);
```

---

## Connection to Ball Dynamics

In the ball dynamics document, the table pose $(\varphi, \theta, z_t)$ and its derivatives $(\dot\varphi, \dot\theta, \dot z_t, \ddot\varphi, \ddot\theta, \ddot z_t)$ appear as **inputs** to the ball dynamics. The table dynamics document closes the loop:

$$
\underbrace{(\alpha_1, \alpha_2, \alpha_3)}_{\text{servo commands}} \xrightarrow{\text{forward kinematics}} \underbrace{(\varphi, \theta, z_t)}_{\text{table pose}} \xrightarrow{\text{ball dynamics}} \underbrace{(x_b, y_b, z_b)}_{\text{ball position}}
$$

For the **controller**, the inverse direction is used:

$$
\underbrace{(x_b, y_b)}_{\text{measured}} \xrightarrow{\text{control law}} \underbrace{(\varphi^*, \theta^*, z_t^*)}_{\text{desired pose}} \xrightarrow{\text{inverse kinematics}} \underbrace{(\alpha_1^*, \alpha_2^*, \alpha_3^*)}_{\text{servo commands}}
$$

The time derivatives needed by the ball dynamics ($\dot\varphi, \dot\theta$, etc.) can be obtained by **differentiating the inverse kinematics** with respect to time, using the mechanism Jacobian.
