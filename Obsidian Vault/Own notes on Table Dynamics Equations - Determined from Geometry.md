We know for certain the following!
The ground points are stationary as:
$$
\mathbf{G}_i = \begin{bmatrix}
R_g \cos(\psi_i) \\
R_g \sin(\psi_i) \\
0
\end{bmatrix}, \quad \psi_i = \frac{2\pi(i-1)}{3}, \quad i = 1, 2, 3
$$
$$
\mathbf{G}_1 = \begin{bmatrix} R_g \\ 0 \\ 0 \end{bmatrix}, \quad
\mathbf{G}_2 = \begin{bmatrix} -R_g/2 \\ R_g\sqrt{3}/2 \\ 0 \end{bmatrix}, \quad
\mathbf{G}_3 = \begin{bmatrix} -R_g/2 \\ -R_g\sqrt{3}/2 \\ 0 \end{bmatrix}
$$
Where $\mathbf{R}_g$ is the radius of the ground disc.

And the elbow points are also directly given to us, as we can measure and command the angle between the ground points, $\mathbf{G}_i$, and the elbow points, $\mathbf{E}_i$, namely the angles $\mathbf{\alpha}_i$.

The angles $\mathbf{\alpha}_i$ are determined from the commanded angles $\mathbf{\alpha}_{i,cmd}$ modeled as a first order transfer function with time constant, $\mathbf{\tau}$.
$$\mathbf{\alpha}_i = \mathbf{\alpha}_i+\frac{\Delta t}{\tau}(\mathbf{\alpha}_{i,cmd} - \mathbf{\alpha}_i)$$
We also have knowledge of the length of the two arms in each leg, $\mathbf{L}_1$ and $\mathbf{L}_2$ and we also know the radius of the table disc, $\mathbf{R}_t$.
Thus, the elbow points can be easily calculated as ground points plus the change in $\mathbf{\alpha}_i$:
$$\mathbf{E}_i = \mathbf{G}_i + \begin{bmatrix}
\cos(\psi_i) \\
\sin(\psi_i) \\
0
\end{bmatrix} \cdot(L_1\cos(\alpha_i)) + \begin{bmatrix}
0 \\
0 \\
L_1\sin(\alpha_i)
\end{bmatrix} $$
$$\mathbf{E}_1 = \begin{bmatrix}
R_g+L_1\cos(\alpha_1) \\
0 \\
L_1\sin(\alpha_1)
\end{bmatrix} $$$$ \mathbf{E}_2 = \begin{bmatrix}
R_g \cos(\psi_2)+\cos(\psi_2)\cdot L_1\cos(\alpha_2) \\
R_g \sin(\psi_2)+\sin(\psi_2)\cdot L_1\cos(\alpha_2) \\
L_1\sin(\alpha_2)
\end{bmatrix}  $$
$$\mathbf{E}_3 = \begin{bmatrix}
R_g \cos(\psi_3)+\cos(\psi_3)\cdot L_1\cos(\alpha_3) \\
R_g \sin(\psi_3)+\sin(\psi_3)\cdot L_1\cos(\alpha_3) \\
L_1\sin(\alpha_3)
\end{bmatrix}$$
And following that the table points can be found similarly:
$$\mathbf{T}_i = \mathbf{E}_i + \begin{bmatrix}
\cos(\psi_i) \\
\sin(\psi_i) \\
0
\end{bmatrix} \cdot(-L_2\cos(\beta_i)) + \begin{bmatrix}
0 \\
0 \\
L_2\sin(\beta_i)
\end{bmatrix} $$
$$\mathbf{T}_1 = \begin{bmatrix}
R_g+L_1\cos(\alpha_1)-L_2\cos(\beta_1) \\
0 \\
L_1\sin(\alpha_1) + L_2\sin(\beta_1)
\end{bmatrix} $$
$$\mathbf{T}_2 = \begin{bmatrix}
R_g \cos(\psi_2)+\cos(\psi_2)\cdot L_1\cos(\alpha_2)-\cos(\psi_2)\cdot L_2\cos(\beta_2) \\
R_g \sin(\psi_2)+\sin(\psi_2)\cdot L_1\cos(\alpha_2)-\sin(\psi_2)\cdot L_2\cos(\beta_2) \\
L_1\sin(\alpha_2) + L_2\sin(\beta_2)
\end{bmatrix}  $$
$$\mathbf{T}_3 = \begin{bmatrix}
R_g \cos(\psi_3)+\cos(\psi_3)\cdot L_1\cos(\alpha_3)-\cos(\psi_3)\cdot L_2\cos(\beta_3) \\
R_g \sin(\psi_3)+\sin(\psi_3)\cdot L_1\cos(\alpha_3)-\sin(\psi_3)\cdot L_2\cos(\beta_3) \\
L_1\sin(\alpha_3)+ L_2\sin(\beta_3)
\end{bmatrix}$$
We also know that the table points are bounded by the disc shape and in case the table was flat on the ground, the points can be computed as:
$$
\mathbf{T}_i = \begin{bmatrix}
R_t \cos(\psi_i) \\
R_t \sin(\psi_i) \\
0
\end{bmatrix}, \quad \psi_i = \frac{2\pi(i-1)}{3}, \quad i = 1, 2, 3
$$
$$
\mathbf{T}_1 = \begin{bmatrix} R_t \\ 0 \\ 0 \end{bmatrix}, \quad
\mathbf{T}_2 = \begin{bmatrix} -R_t/2 \\ R_t\sqrt{3}/2 \\ 0 \end{bmatrix}, \quad
\mathbf{T}_3 = \begin{bmatrix} -R_t/2 \\ -R_t\sqrt{3}/2 \\ 0 \end{bmatrix}
$$
Then the distance between two table points, $D_t$, when the table is flat on the ground can be calculated as:
$$D_t=\|T_1-T_2\|=\sqrt{(R_t-(-R_t/2))²+(0-R_t\sqrt{3}/2)²}$$
It can also be calculated as $|T_1-T_3|$ and $|T_2-T_3|$, but it will give he same result.
Then, we can use the other definitions for the table points in distance constraint giving us three equations with three unknowns
$$D_t=\|T_1-T_2\| = \sqrt{(T_{1,x}-T_{2,x})²+(T_{1,y}-T_{2,y})²+(T_{1,z}-T_{2,z})²}$$
$$D_t=\|T_1-T_3\| = \sqrt{(T_{1,x}-T_{3,x})²+(T_{1,y}-T_{3,y})²+(T_{1,z}-T_{3,z})²}$$
$$D_t=\|T_2-T_3\| = \sqrt{(T_{2,x}-T_{3,x})²+(T_{2,y}-T_{3,y})²+(T_{2,z}-T_{3,z})²}$$
With this, we can solve for the three elbow angles: $\beta_1,\beta_2,\beta_3$ .


---

## Step 1 — Compact notation for the constraint equations

Introduce the **net radial reach** of arm $i$ (the horizontal distance from the center to $\mathbf{T}_i$, measured along $\hat{\mathbf{r}}_i$):

$$\rho_i = R_g + L_1\cos\alpha_i - L_2\cos\beta_i$$

With this, the table attachment points take the tidy form:

$$\mathbf{T}_i = \begin{bmatrix} \rho_i\cos\psi_i \\ \rho_i\sin\psi_i \\ L_1\sin\alpha_i + L_2\sin\beta_i \end{bmatrix}$$

Also define the **height** of each table point:

$$h_i = L_1\sin\alpha_i + L_2\sin\beta_i$$

so $\mathbf{T}_i = [\rho_i\cos\psi_i,\; \rho_i\sin\psi_i,\; h_i]^\top$.

Note that $\alpha_i$ is **known** (commanded servo angle), so $\rho_i$ and $h_i$ each contain exactly one unknown: $\beta_i$.

---

## Step 2 — Distance squared between two table points

Compute $\|\mathbf{T}_i - \mathbf{T}_j\|^2$ for any pair $(i,j)$:

$$
\|\mathbf{T}_i - \mathbf{T}_j\|^2
= (\rho_i\cos\psi_i - \rho_j\cos\psi_j)^2
+ (\rho_i\sin\psi_i - \rho_j\sin\psi_j)^2
+ (h_i - h_j)^2
$$

Expand the first two terms:

$$
= \rho_i^2\cos^2\psi_i - 2\rho_i\rho_j\cos\psi_i\cos\psi_j + \rho_j^2\cos^2\psi_j
$$$$+ \rho_i^2\sin^2\psi_i - 2\rho_i\rho_j\sin\psi_i\sin\psi_j + \rho_j^2\sin^2\psi_j
+ (h_i - h_j)^2
$$

Using $\cos^2\psi + \sin^2\psi = 1$ and $\cos\psi_i\cos\psi_j + \sin\psi_i\sin\psi_j = \cos(\psi_i - \psi_j)$:

$$
\boxed{
\|\mathbf{T}_i - \mathbf{T}_j\|^2 = \rho_i^2 + \rho_j^2 - 2\rho_i\rho_j\cos(\psi_i - \psi_j) + (h_i - h_j)^2
}
$$

**Key simplification**: for any pair of arms, the angular separation is $|\psi_i - \psi_j| = 2\pi/3$, so:
$$\cos(\psi_i - \psi_j) = \cos\!\left(\frac{2\pi}{3}\right) = -\frac{1}{2}$$
Therefore, for **all three pairs**:
$$\|\mathbf{T}_i - \mathbf{T}_j\|^2 = \rho_i^2 + \rho_j^2 + \rho_i\rho_j + (h_i - h_j)^2$$
---

## Step 3 — Reference chord length $D_t$

From the flat-ground formula (derived earlier):

$$
D_t = \sqrt{\left(R_t + \frac{R_t}{2}\right)^2 + \left(\frac{R_t\sqrt{3}}{2}\right)^2}
= \sqrt{\frac{9R_t^2}{4} + \frac{3R_t^2}{4}}
= \sqrt{3R_t^2}
= \sqrt{3}\,R_t
$$

$$\boxed{D_t = \sqrt{3}\,R_t, \qquad D_t^2 = 3R_t^2}$$

This is constant regardless of the table pose — it is a property of the rigid table disc, not of its orientation.

---

## Step 4 — The three constraint equations

Substituting into the distance constraint $\|\mathbf{T}_i - \mathbf{T}_j\|^2 = D_t^2 = 3R_t^2$ for each pair:

$$
\rho_1^2 + \rho_2^2 + \rho_1\rho_2 + (h_1 - h_2)^2 = 3R_t^2
$$

$$
\rho_1^2 + \rho_3^2 + \rho_1\rho_3 + (h_1 - h_3)^2 = 3R_t^2
$$

$$
\rho_2^2 + \rho_3^2 + \rho_2\rho_3 + (h_2 - h_3)^2 = 3R_t^2
$$

Substituting back the definitions ($\rho_i = R_g + L_1\cos\alpha_i - L_2\cos\beta_i$, $h_i = L_1\sin\alpha_i + L_2\sin\beta_i$) and defining known constants for each arm:

$$A_i = R_g + L_1\cos\alpha_i \quad \text{(known radial offset)}, \qquad B_i = L_1\sin\alpha_i \quad \text{(known height offset)}$$

so that $\rho_i = A_i - L_2\cos\beta_i$ and $h_i = B_i + L_2\sin\beta_i$. The three equations become:

$$
(A_1 - L_2 c_1)^2 + (A_2 - L_2 c_2)^2 + (A_1 - L_2 c_1)(A_2 - L_2 c_2) + (B_1 + L_2 s_1 - B_2 - L_2 s_2)^2 = 3R_t^2
$$

$$
(A_1 - L_2 c_1)^2 + (A_3 - L_2 c_3)^2 + (A_1 - L_2 c_1)(A_3 - L_2 c_3) + (B_1 + L_2 s_1 - B_3 - L_2 s_3)^2 = 3R_t^2
$$

$$
(A_2 - L_2 c_2)^2 + (A_3 - L_2 c_3)^2 + (A_2 - L_2 c_2)(A_3 - L_2 c_3) + (B_2 + L_2 s_2 - B_3 - L_2 s_3)^2 = 3R_t^2
$$

where the shorthand $c_i = \cos\beta_i$, $s_i = \sin\beta_i$ is used for brevity.

These three equations have exactly three unknowns: $\beta_1, \beta_2, \beta_3$. They are nonlinear and coupled, so they must be solved numerically (e.g. Newton-Raphson).

### Residual form for numerical solving

Define the residual vector $\mathbf{f}(\boldsymbol{\beta}) : \mathbb{R}^3 \to \mathbb{R}^3$ where $\boldsymbol{\beta} = [\beta_1, \beta_2, \beta_3]^\top$:

$$
\boxed{
\mathbf{f}(\boldsymbol{\beta}) = \begin{bmatrix}
\rho_1^2 + \rho_2^2 + \rho_1\rho_2 + (h_1 - h_2)^2 - 3R_t^2 \\
\rho_1^2 + \rho_3^2 + \rho_1\rho_3 + (h_1 - h_3)^2 - 3R_t^2 \\
\rho_2^2 + \rho_3^2 + \rho_2\rho_3 + (h_2 - h_3)^2 - 3R_t^2
\end{bmatrix} = \mathbf{0}
}
$$

Newton-Raphson iteration: given an initial guess $\boldsymbol{\beta}^{(0)}$ (e.g., from the previous timestep), iterate:

$$\boldsymbol{\beta}^{(k+1)} = \boldsymbol{\beta}^{(k)} - J^{-1}\!\left(\boldsymbol{\beta}^{(k)}\right)\,\mathbf{f}\!\left(\boldsymbol{\beta}^{(k)}\right)$$

where $J_{ij} = \partial f_i / \partial \beta_j$ is the Jacobian. The partial derivatives are:

$$
\frac{\partial \rho_i}{\partial \beta_i} = L_2\sin\beta_i, \qquad
\frac{\partial h_i}{\partial \beta_i} = L_2\cos\beta_i
$$

**Limiting case check**: if $\alpha_1 = \alpha_2 = \alpha_3 = \alpha$ (all servos equal), by symmetry $\beta_1 = \beta_2 = \beta_3 = \beta$ is the solution. Then $\rho_i = \rho$ and $h_i = h$ for all $i$, and each equation reduces to $3\rho^2 = 3R_t^2$, i.e. $\rho = R_t$, i.e. $A - L_2\cos\beta = R_t$ — a single equation in one unknown $\beta$, as expected.

---

## Step 5 — Extracting table pose $(\varphi, \theta, z_t)$ from the three table points

Once $\beta_1, \beta_2, \beta_3$ are known, the three table attachment points $\mathbf{T}_1, \mathbf{T}_2, \mathbf{T}_3$ are fully determined. The table pose is recovered by fitting these three points to the rigid-body parameterisation.
### Table center

The table center $\mathbf{c} = [0, 0, z_t]^\top$ (we assume no lateral translation by symmetry) is the **centroid of the three table points expressed in the table frame** mapped back through the rotation. A simpler route: the $z$-coordinate of the table center equals the average of the three $z$-components, **only when tilt is small** (see approximation below). For the exact case we need the full pose.

#### Exact approach — solve $\mathbf{T}_i = \mathbf{c} + R\,\mathbf{P}_i^T$

From the main document, the table attachment point in the inertial frame is:
$$
\mathbf{T}_i = \begin{bmatrix} 0 \\ 0 \\ z_t \end{bmatrix} + R \begin{bmatrix} R_t\cos\psi_i \\ R_t\sin\psi_i \\ 0 \end{bmatrix}
$$

where $R = R_y(\theta)\,R_x(\varphi)$. This gives, for the $z$-component of each point:

$$
T_{i,z} = z_t + R_t\bigl(-\sin\theta\cos\varphi\cos\psi_i + \sin\varphi\sin\psi_i\bigr)
$$

Writing out the three arms ($\psi_1 = 0$, $\psi_2 = 2\pi/3$, $\psi_3 = 4\pi/3$):

$$
h_1 = z_t - R_t\sin\theta\cos\varphi
$$
$$
h_2 = z_t + R_t\left(\frac{1}{2}\sin\theta\cos\varphi + \frac{\sqrt{3}}{2}\sin\varphi\right)
$$
$$
h_3 = z_t + R_t\left(\frac{1}{2}\sin\theta\cos\varphi - \frac{\sqrt{3}}{2}\sin\varphi\right)
$$
This is a linear system in three unknowns $\{z_t,\; \sin\theta\cos\varphi,\; \sin\varphi\}$. In matrix form:
$$
\begin{bmatrix} 1 & -R_t & 0 \\ 1 & R_t/2 & R_t\sqrt{3}/2 \\ 1 & R_t/2 & -R_t\sqrt{3}/2 \end{bmatrix}
\begin{bmatrix} z_t \\ \sin\theta\cos\varphi \\ \sin\varphi \end{bmatrix}
=
\begin{bmatrix} h_1 \\ h_2 \\ h_3 \end{bmatrix}
$$

#### Solving the linear system

Call the matrix $M$. Its determinant:
$$\det(M) = 1\cdot\bigl((R_t/2)(-R_t\sqrt{3}/2) - (R_t\sqrt{3}/2)(R_t/2)\bigr) - (-R_t)\bigl((-R_t\sqrt{3}/2) - (R_t\sqrt{3}/2)\bigr) + 0$$

$$= -\frac{R_t^2\sqrt{3}}{4} - \frac{R_t^2\sqrt{3}}{4} - R_t\cdot R_t\sqrt{3} = -\frac{R_t^2\sqrt{3}}{2} - R_t^2\sqrt{3} = -\frac{3R_t^2\sqrt{3}}{2}$$

Solving by Cramer's rule (or by direct row reduction — summing all three rows gives $3z_t = h_1+h_2+h_3$):
$$
\boxed{z_t = \frac{h_1 + h_2 + h_3}{3}}
$$

Subtracting the first equation from the average of the last two:
$$
\frac{h_2+h_3}{2} - h_1 = \frac{3R_t}{2}\sin\theta\cos\varphi
\implies
\sin\theta\cos\varphi = \frac{h_2+h_3-2h_1}{3R_t}
$$

Subtracting the third equation from the second:

$$
h_2 - h_3 = R_t\sqrt{3}\sin\varphi
\implies
\sin\varphi = \frac{h_2 - h_3}{\sqrt{3}\,R_t}
$$

From $\sin\varphi$ we recover $\varphi = \arcsin\!\left(\dfrac{h_2-h_3}{\sqrt{3}\,R_t}\right)$, then:

$$\sin\theta = \frac{\sin\theta\cos\varphi}{\cos\varphi} = \frac{h_2+h_3-2h_1}{3R_t\cos\varphi}$$

$$\theta = \arcsin\!\left(\frac{h_2+h_3-2h_1}{3R_t\cos\varphi}\right)$$

#### Exact boxed results

$$
\boxed{
z_t = \frac{h_1 + h_2 + h_3}{3}
}
$$

$$
\boxed{
\varphi = \arcsin\!\left(\frac{h_2 - h_3}{\sqrt{3}\,R_t}\right)
}
$$

$$
\boxed{
\theta = \arcsin\!\left(\frac{h_2+h_3-2h_1}{3R_t\cos\varphi}\right)
}
$$

where $h_i = L_1\sin\alpha_i + L_2\sin\beta_i$ is the height of table point $i$.

**Physical interpretation**:
- $z_t$ is simply the **mean height** of the three table points — the table centre rides at the average.
- $\varphi$ (roll) is driven by the **difference** between arms 2 and 3, which are symmetric about the $xz$-plane.
- $\theta$ (pitch) is driven by how much arm 1 deviates from the mean of arms 2 and 3; the $\cos\varphi$ factor corrects for the fact that a pure pitch looks slightly different when the table is already rolled.

**Limiting case**: if $h_1 = h_2 = h_3 = h$ (all arms equal height), then $\varphi = 0$, $\theta = 0$, $z_t = h$ — flat table at height $h$. ✓

---

### Small-angle approximation

For small $\varphi, \theta$: $\cos\varphi \approx 1$, $\arcsin(x) \approx x$, giving:

$$
\boxed{
z_t \approx \frac{h_1+h_2+h_3}{3}, \qquad
\theta \approx \frac{h_2+h_3-2h_1}{3R_t}, \qquad
\varphi \approx \frac{h_2-h_3}{\sqrt{3}\,R_t}
}
$$

These match the small-angle results in the main table dynamics document exactly.

---

## Summary of the forward kinematics chain (geometry approach)

```
Known:  α₁, α₂, α₃  (servo angles)
        L₁, L₂, Rg, Rt  (geometry)

Step 1: Compute Aᵢ = Rg + L₁cosαᵢ ,  Bᵢ = L₁sinαᵢ
        (known per-arm offsets)

Step 2: Solve f(β) = 0  numerically for β₁, β₂, β₃
        using Newton-Raphson with fᵢ as defined above.
        Initial guess: βᵢ⁽⁰⁾ from previous timestep.

Step 3: Compute table point heights
        hᵢ = Bᵢ + L₂sinβᵢ

Step 4: Extract pose
        z_t = (h₁+h₂+h₃)/3
        φ   = arcsin((h₂-h₃) / (√3 Rt))
        θ   = arcsin((h₂+h₃-2h₁) / (3 Rt cosφ))
```
