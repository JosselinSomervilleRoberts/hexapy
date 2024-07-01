# Hexapod Maths

## Notations

Let's define a few characteristics points. We will have $O_F$ the center of the floor (fixed), $O_B$ the center of the body and then $A_i, B_i, C_i$ and $D_i$ the respective points of each actuator $i$ for $i \in {1,..,6}$ with $D_i$ the extremity of the leg.

We will use generalized coordinated meaning that each point $P$ will be represented by $P^A = [x_P, y_P, z_P, 1]_A^T$ where $A$ is the base $A$.
To go frome one base to another, we define the transformation matrices $T$ such that $P^A = T^A_B P^B$. We will define the bases $F$ the floor, $B$ the body, and then $j$ for $j \in \{1,2,3\}$ the referential of each moving palanx (it should technically be indexed by $i$ but we will omit this for simplicity.)


## Forward Kinematics

$$T^F_B = \begin{bmatrix}
c\phi . c\psi - s\psi . s\xi . s\psi & -s\phi . c\xi & c\phi . s\psi + s\phi . s\xi . c\psi & x_B \\
s\phi . c\psi + c\phi * s\xi . s\psi & c\phi . c\xi & s\phi . s\psi - c\phi . s\xi . c\psi & y_B \\
-c\xi . s\psi & s\xi & c\xi . c\psi & z_B \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$T^B_{1_i} = \begin{bmatrix}
c\alpha & s\alpha & 0 & x_i \\
-s\alpha & c\alpha & 0 & y_i \\
0 & 0 & 1 & z_i \\
0 & 0 & 0 & 1
\end{bmatrix}

\text{, }

T^1_2 = \begin{bmatrix}
c\beta & 0 & s\beta & d_1 \\
0 & 1 & 0 & 0 \\
-s\beta & 0 & c\beta & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}

\text{, }

T^2_3 = \begin{bmatrix}
c\gamma & 0 & s\gamma & d_2 \\
0 & 1 & 0 & 0 \\
-s\gamma & 0 & c\gamma & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}

\text{, }

T^3_4 = \begin{bmatrix}
1 & 0 & 0 & d_3 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$


## Jacobian

Let's note $ J = \begin{bmatrix}J_v \\ J_\omega \end{bmatrix}$ such that $\begin{bmatrix}v \\ \omega \end{bmatrix} = J \dot{q}$

Let's recall that $v = \begin{bmatrix}\dot{x} \\ \dot{y} \\ \dot{z} \end{bmatrix} = \dot{P} = \frac{\partial P}{\partial q_1}.\dot{q_1} + \frac{\partial P}{\partial q_2}.\dot{q_2} + \frac{\partial P}{\partial q_3}.\dot{q_3}$ so $J_v = \begin{bmatrix}\frac{\partial P}{\partial q_1} & \frac{\partial P}{\partial q_2} & \frac{\partial P}{\partial q_3}\end{bmatrix}$

We will continue in the base $B$ as $T^F_B$ does not depend on $q = \begin{bmatrix} \alpha & \beta & \gamma \end{bmatrix}^T$.

We will need to compute $T^B_{j_i}$ for $j \in \{1, ..., 4\}$.

$$T^B_{2_i} = T^B_{1_i} . T^1_2 = 
\begin{bmatrix}
c\alpha & s\alpha & 0 & x_i \\
-s\alpha & c\alpha & 0 & y_i \\
0 & 0 & 1 & z_i \\
0 & 0 & 0 & 1
\end{bmatrix}
.
\begin{bmatrix}
c\beta & 0 & s\beta & d_1 \\
0 & 1 & 0 & 0 \\
-s\beta & 0 & c\beta & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
c\alpha.c\beta & s\alpha & c\alpha.s\beta & x_i + d_1.c\alpha \\
-s\alpha.c\beta & c\alpha & -s\alpha.s\beta & y_i - d_1.s\alpha \\
-s\beta & 0 & c\beta & z_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$T^B_{3_i} = T^B_{2_i} . T^2_3 = 
\begin{bmatrix}
c\alpha.c\beta & s\alpha & c\alpha.s\beta & x_i + d_1.c\alpha \\
-s\alpha.c\beta & c\alpha & -s\alpha.s\beta & y_i - d_1.s\alpha \\
-s\beta & 0 & c\beta & z_i \\
0 & 0 & 0 & 1
\end{bmatrix}
.
\begin{bmatrix}
c\gamma & 0 & s\gamma & d_2 \\
0 & 1 & 0 & 0 \\
-s\gamma & 0 & c\gamma & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
c\alpha.c\beta.c\gamma - c\alpha.s\beta.s\gamma & s\alpha & c\alpha.c\beta.s\gamma + c\alpha.s\beta.s\gamma & x_i + d_1.c\alpha + d_2.c\alpha.c\beta\\
-s\alpha.c\beta.c\gamma + s\alpha.s\beta.s\gamma & c\alpha & -s\alpha.c\beta.s\gamma - s\alpha.s\beta.c\gamma & y_i -d_1.s\alpha - d_2.s\alpha.c\beta\\
-s\beta.c\gamma - c\beta.s\gamma & 0 & -s\beta.s\gamma + c\beta.c\gamma & z_i - d_2.s\beta\\
0 & 0 & 0 & 1
\end{bmatrix}
$$
which can be simplified as:
$T^B_{3_i} = \begin{bmatrix}
c\alpha.c(\beta+\gamma) & s\alpha & c\alpha.s(\beta+\gamma) & x_i + d_1.c\alpha + d_2.c\alpha.c\beta\\
-s\alpha.c(\beta+\gamma) & c\alpha & -s\alpha.s(\beta+\gamma) & y_i -d_1.s\alpha - d_2.s\alpha.c\beta\\
-s(\beta+\gamma) & 0 & c(\beta + \gamma) & z_i - d_2.s\beta\\
0 & 0 & 0 & 1
\end{bmatrix}$

$$T^B_{4_i} = T^B_{3_i}.T^3_4 = 
\begin{bmatrix}
c\alpha.c(\beta+\gamma) & s\alpha & c\alpha.s(\beta+\gamma) & x_i + d_1.c\alpha + d_2.c\alpha.c\beta\\
-s\alpha.c(\beta+\gamma) & c\alpha & -s\alpha.s(\beta+\gamma) & y_i -d_1.s\alpha - d_2.s\alpha.c\beta\\
-s(\beta+\gamma) & 0 & c(\beta + \gamma) & z_i - d_2.s\beta\\
0 & 0 & 0 & 1
\end{bmatrix}
.
\begin{bmatrix}
1 & 0 & 0 & d_3 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
c\alpha.c(\beta+\gamma) & s\alpha & c\alpha.s(\beta+\gamma) & x_i + d_1.c\alpha + d_2.c\alpha.c\beta + d_3.c\alpha.c(\beta+\gamma)\\
-s\alpha.c(\beta+\gamma) & c\alpha & -s\alpha.s(\beta+\gamma) & y_i -d_1.s\alpha - d_2.s\alpha.c\beta - d_3.s\alpha.c(\beta+\gamma)\\
-s(\beta+\gamma) & 0 & c(\beta + \gamma) & z_i - d_2.s\beta - d_3.s(\beta+\gamma)\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Since we compute the Jacobian of the end of the leg in the base $B$, the point $P$ in the jacobian is none other than $D^B$, i.e. the 4-th colum of $T^B_{4_i}$.

Therefore,
$$
J_v^B = \begin{bmatrix}
-d_1.s\alpha - d_2.s\alpha.c\beta - d_3.s\alpha.c(\beta+\gamma) & -d_2.c\alpha.s\beta - d_3.c\alpha.s(\beta+\gamma) & -d_3.c\alpha.s(\beta+\gamma)\\
-d_1.c\alpha - d_2.c\alpha.c\beta - d_3.c\alpha.c(\beta+\gamma) & d_2.s\alpha.s\beta + d_3.s\alpha.s(\beta+\gamma) & d_3.s\alpha.s(\beta+\gamma)\\
0 & -d_2.c\beta - d_3.c(\beta+\gamma) & - d_3.c(\beta+\gamma)
\end{bmatrix}
$$
and
$$
J_\omega^B = \begin{bmatrix}Z_1^B & Y_2^B & Y_3^B
\end{bmatrix}
=
\begin{bmatrix}
0 & s\alpha & s\alpha\\
0 & c\alpha & c\alpha\\
1 & 0 & 0
\end{bmatrix}
$$


## Dynamics

Let's remember that the generalized mass matrix $M$ is defined as $M = \sum_{i=1}^n \left(m_i.J_{v_i}^T.J_{v_i} + J_{\omega_i}^T.I_{C_i}.J_{\omega_i}\right)$, with $m_i$ the mass of part $i$ and $J_{v_i}$ the linear Jacobian at the center of mass. We will make a few asumptions regarding the distribution of masses. For each part $i$, there is one ponctual mass $m_{S_{i+1}}$ at distance $d_i$ corresponding to the motor for the next joint and the mass of the link $M_i$ at the center of the part $d_i/2$. The only exception is for join $i=3$, there is no actuator so only the mass of the part is used.

Therefore $m_{G_i} = m_i + m_{S_{i+1}}$ and $d_{G_i} = d_i\frac{m_i/2 + m_{S_{i+1}}}{m_i + m_{S_{i+1}}}$ for $i \in \{1,2\}$ and $m_{G_3} = m_3$ with $d_{G_3} = d_3/2$

Let's compute this in the referential of the base $B$:
$$J_{v_B}^B = 
\begin{bmatrix}
-d_1.s\alpha & 0 & 0\\
-d_1.c\alpha & 0 & 0\\
0 & 0 & 0
\end{bmatrix}
\text{ so }
(J_{v_B}^B)^T.J_{v_B}^B = \begin{bmatrix}
\left(d_1\frac{m_1/2 + m_{S_2}}{m_1 + m_{S_2}}\right)^2 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & 0\\
\end{bmatrix}
$$

$$J_{v_C}^B = 
\begin{bmatrix}
-d_1.s\alpha - d_2.s\alpha.c\beta & -d_2.c\alpha.s\beta & 0\\
-d_1.c\alpha -d_2.c\alpha.c\beta & d_2.s\alpha.s\beta & 0\\
0 & - d_2.c\beta & 0
\end{bmatrix}
\text{ so }
(J_{v_C}^B)^T.J_{v_C}^B = \begin{bmatrix}
(d_1 + d_2.c\beta\frac{m_2/2 + m_{S_3}}{m_2 + m_{S_3}})^2 & 0 & 0\\
0 & \left(d_2\frac{m_2/2 + m_{S_3}}{m_2 + m_{S_3}}\right)^2 & 0\\
0 & 0 & 0
\end{bmatrix}
$$

$$J_{v_D}^B = J_V^B
\text{ so }
(J_{v_D}^B)^T.J_{v_D}^B = \begin{bmatrix}
(d_1 + d_2.c\beta + \frac{d_3}{2}.c(\beta+\gamma))^2 & 0 & 0\\
0 & (d_2 + \frac{d_3}{2}.c\gamma)^2 + (\frac{d_3}{2}.s\gamma)^2 & \frac{d_2.d_3}{2}.s(\beta+\gamma)(c\beta + s\beta) + \frac{d_3^2}{4} \\
0 & \frac{d_2.d_3}{2}.s(\beta+\gamma)(c\beta + s\beta) + \frac{d_3^2}{4} & \frac{d_3^2}{4}
\end{bmatrix}
$$

and the rotational jacobian are:
$$J_{\omega_1}^B = \begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
1 & 0 & 0\\
\end{bmatrix}
\text{ so }
(J_{\omega_1}^B)^T.I_{C_1}.J_{\omega_1}^B = 
\begin{bmatrix}
0 & 0 & 1\\
0 & 0 & 0\\
0 & 0 & 0\\
\end{bmatrix}
\begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & I_{S_1}\\
\end{bmatrix}
\begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
1 & 0 & 0\\
\end{bmatrix}
=
\begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & I_{S_1}\\
\end{bmatrix}
$$

$$J_{\omega_2}^B = \begin{bmatrix}
0 & s\alpha & 0\\
0 & c\alpha & 0\\
1 & 0 & 0\\
\end{bmatrix}
\text{ so }
(J_{\omega_2}^B)^T.I_{C_2}.J_{\omega_2}^B = 
\begin{bmatrix}
0 & 0 & 1\\
s\alpha & c\alpha & 0\\
0 & 0 & 0\\
\end{bmatrix}
\begin{bmatrix}
0 & 0 & 0\\
0 & I_{S_2} & 0\\
0 & 0 & 0\\
\end{bmatrix}
\begin{bmatrix}
0 & s\alpha & 0\\
0 & c\alpha & 0\\
1 & 0 & 0\\
\end{bmatrix}
=
\begin{bmatrix}
0 & 0 & 0\\
0 & I_{s_2} & 0\\
0 & 0 & 0\\
\end{bmatrix}
$$

$$J_{\omega_3}^B = \begin{bmatrix}
0 & s\alpha & s\alpha\\
0 & c\alpha & c\alpha\\
1 & 0 & 0\\
\end{bmatrix}
\text{ so }
(J_{\omega_3}^B)^T.I_{C_3}.J_{\omega_3}^B = 
\begin{bmatrix}
0 & 0 & 1\\
s\alpha & c\alpha & 0\\
s\alpha & c\alpha & 0\\
\end{bmatrix}
\begin{bmatrix}
0 & 0 & 0\\
0 & I_{S_3} & 0\\
0 & 0 & 0\\
\end{bmatrix}
\begin{bmatrix}
0 & s\alpha & s\alpha\\
0 & c\alpha & c\alpha\\
1 & 0 & 0\\
\end{bmatrix}
=
\begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & I_{S_3}\\
\end{bmatrix}
$$

This gives us:
$$
M = 
\begin{bmatrix}
I_{S_1} + m_{G_1}.d_{G_1}^2 + m_{G_2}\left(d_1 + d_{G_2}.c\beta\right)^2 + m_{G_3}\left(d_1 + d_2.c\beta + d_{G_3}.c(\beta+\gamma)\right)^2 & 0 & 0\\
0 & I_{S_2} + m_{G_2}.d_{G_2}^2 + m_{G_3}\left(d_2 + d_{G_3}.c\gamma\right)^2 & m_{G_3}\left(d_{G_3}^2 + d_{G_2}.d_{G_3}.s(\beta+\gamma)(c\beta + s\beta)\right)\\
0 & m_{G_3}\left(d_{G_3}^2 + d_{G_2}.d_{G_3}.s(\beta+\gamma)(c\beta + s\beta)\right) & I_{S_3} + m_{G_3}.d_{G_3}^2
\end{bmatrix}