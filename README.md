# Flatness-Based Mixed-Integer Obstacle Avoidance MPC (OA-MPC)

A framework for **Flatness-Based Mixed-Integer Obstacle Avoidance Model Predictive Control**, based on the methodology presented by Gratzer et al. (CoDIT 2023):

> **A. L. Gratzer, M. M. Broger, A. Schirrer, and S. Jakubek**, *"Flatness-Based Mixed-Integer Obstacle Avoidance MPC for Collision-Safe Automated Urban Driving,"* in 2023 9th International Conference on Control, Decision and Information Technologies (CoDIT), Rome, Italy, 2023, pp. 1844–1849. DOI: `10.1109/CODIT58514.2023.10284415`

---

## Overview

Autonomous driving in complex multi-agent urban environments (such as intersections and multi-lane curved roads) requires trajectory planning and control algorithms that guarantee collision safety while remaining computationally tractable. 

This repository implements a modular **Obstacle Avoidance MPC (OA-MPC)** strategy that exploits the **differential flatness** property of the kinematic single-track (bicycle) model. By transforming the nonlinear vehicle dynamics into an exact **Linear Time-Invariant (LTI)** system in flat coordinates—either Global Cartesian $(X, Y)$ or Frenet path coordinates $(s, l)$—the non-convex collision avoidance problem is formulated as a **Mixed-Integer Quadratic/Linear Program (MIQP/MILP)** using the **Big-M method**. This guarantees globally optimal avoidance maneuvers across multi-agent traffic scenarios.

### Key Features
* **Coordinate-Agnostic LTI-MPC Core:** Decouples the coordinate transformation layer from the core optimization solver, supporting both **Global Cartesian** (unstructured maneuvers) and **Frenet Frame** (structured road/lane following and overtaking) formulations.
* **Exact Feedback Linearization via Flatness:** Represents the nonlinear kinematic single-track vehicle as two decoupled double integrators in flat space.
* **Curvature-Continuous ($C^3$) Frenet Mapping:** Utilizes cubic/quintic spline parameterization with continuous curvature $\kappa(s)$ and curvature derivative $\kappa'(s)$ to prevent steering angle jumps when mapping flat trajectories back to physical actuator commands.
* **Convex Hull Obstacle Approximation in Frenet Space:** Accounts for geometric distortion of bounding boxes along curved reference paths by projecting obstacle vertices into Frenet space and computing their convex hull.
* **Iterative Collision Detection Loop:** Solves an unconstrained/convex problem first and incrementally adds only active colliding obstacles to the mixed-integer formulation, significantly reducing binary decision variables and solver runtime.

---

## Mathematical Formulation

### 1. Kinematic Single-Track Vehicle Model
Under the zero lateral slip assumption ($v_y \equiv 0$), the nonlinear vehicle dynamics with state $\mathbf{x} = [X, Y, \psi, v_x]^T$ and physical control input $\mathbf{u} = [\delta, a_x]^T$ are given by:

$$\dot{\mathbf{x}} = \begin{bmatrix} \dot{X}; \\ \dot{Y}; \\ \dot{\psi}; \\ \dot{v}_x \end{bmatrix} = \begin{bmatrix} v_x \cos\psi; \\ v_x \sin\psi; \\ \frac{v_x}{L_{\text{wb}}} \tan\delta; \\ a_x \end{bmatrix}$$

where $(X, Y)$ is the rear-axle global position, $\psi$ is the heading angle, $v_x \ge 0$ is the longitudinal velocity, $\delta$ is the front steering angle, $a_x$ is the longitudinal acceleration, and $L_{\text{wb}}$ is the wheelbase.

### 2. Differential Flatness Transformation
Selecting flat outputs $\mathbf{y} = [y_1, y_2]^T$ transforms the nonlinear system into two decoupled double integrators in discrete time (Zero-Order Hold with sampling period $T_s$):

$$\mathbf{z}_{k+1} = \mathbf{A}_d \mathbf{z}_k + \mathbf{B}_d \boldsymbol{\nu}_k = \begin{bmatrix} 1 & T_s & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & T_s \\ 0 & 0 & 0 & 1 \end{bmatrix} \mathbf{z}_k + \begin{bmatrix} \frac{1}{2}T_s^2 & 0 \\ T_s & 0 \\ 0 & \frac{1}{2}T_s^2 \\ 0 & T_s \end{bmatrix} \boldsymbol{\nu}_k$$

where $\mathbf{z} = [y_1, \dot{y}_1, y_2, \dot{y}_2]^T$ is the flat state vector and $\boldsymbol{\nu} = [\ddot{y}_1, \ddot{y}_2]^T$ is the virtual control input.

* **Cartesian Formulation ($\mathbf{y} = [X, Y]^T$):**
  $$\mathbf{z} = \begin{bmatrix} X \\ v_x \cos\psi \\ Y \\ v_x \sin\psi \end{bmatrix}, \qquad \begin{bmatrix} \delta \\ a_x \end{bmatrix} = \begin{bmatrix} \arctan\left(\frac{L_{\text{wb}}}{v_x^2 + \epsilon}(-\nu_1 \sin\psi + \nu_2 \cos\psi)\right) \\ \nu_1 \cos\psi + \nu_2 \sin\psi \end{bmatrix}$$
  *(where $\epsilon > 0$ avoids division-by-zero singularity at $v_x = 0$).*

* **Frenet Formulation ($\mathbf{y} = [s, l]^T$):**
  $$\mathbf{z} = [s, \dot{s}, l, \dot{l}]^T, \qquad \boldsymbol{\nu} = [\ddot{s}, \ddot{l}]^T$$
  where $s$ is the arc length along the reference path and $l$ is the orthogonal lateral deviation. Physical inputs $(\delta, a_x)$ are recovered via exact Frenet-to-Cartesian curvature transformations.

### 3. Mixed-Integer Optimal Control Problem (OA-MPC)
At each time step $k$, the controller solves the finite-horizon optimization problem over prediction horizon $N_p$:

$$\min_{\mathcal{V}, \mathbf{Z}, s_{\text{slack}}} \sum_{j=0}^{N_p-1} \left( \|\mathbf{e}_{k+j+1}\|_{\mathbf{Q}} + \|\boldsymbol{\nu}_{k+j}\|_{\mathbf{R}} \right) + r_s s_{\text{slack}}$$

subject to:
1. **Flat LTI Dynamics:** $\mathbf{z}_{k+j+1} = \mathbf{A}_d \mathbf{z}_{k+j} + \mathbf{B}_d \boldsymbol{\nu}_{k+j}$
2. **Mapped Actuator Constraints:** $\tilde{\mathbf{u}}_{\min} \le \mathbf{M}(\mathbf{x}_{k+j})\boldsymbol{\nu}_{k+j} \le \tilde{\mathbf{u}}_{\max}$ (mapping physical bounds $\delta \in [-\delta_{\max}, \delta_{\max}]$ and $a_x \in [a_{\min}, a_{\max}]$ into virtual input space).
3. **Road / Lane Boundaries (Frenet):** $l_{\min} \le l_{k+j+1} \le l_{\max}$ and $\dot{s}_{k+j+1} \ge 0$.
4. **Big-M Polytopic Obstacle Avoidance:** For each active obstacle $\mathcal{Q}_i = \{ \mathbf{y} : \mathbf{G}\mathbf{y} \le \mathbf{f} \}$ with $n_e$ edges and safety buffer $\gamma$:
   $$\begin{aligned}
   \mathbf{f} - \mathbf{G}\begin{bmatrix} y_{1, k+j+1} \\ y_{2, k+j+1} \end{bmatrix} + (\gamma - s_{\text{slack}})\mathbf{1} &\le M(\mathbf{1} - \boldsymbol{\delta}_{\text{bin}}) \\
   \mathbf{1}^T \boldsymbol{\delta}_{\text{bin}} &\ge 1, \quad \boldsymbol{\delta}_{\text{bin}} \in \{0, 1\}^{n_e}, \quad s_{\text{slack}} \ge 0
   \end{aligned}$$

---

## Control Architecture

The closed-loop control pipeline executes the following five stages at every sampling step $k$:

```text
┌───────────────────┐      ┌────────────────────┐     ┌──────────────────────────────────┐
│  x_k, Y_ref, O_k  │────▶│  Transform to Flat │────▶│   Map Physical Input Constraints │
└───────────────────┘      │  Coordinates (z_k) │     │     (U -> W via x_{k-1} / x_k)   │
                           └────────────────────┘     └─────────────────┬────────────────┘
                                                                        │
┌───────────────────┐      ┌────────────────────┐                       ▼
│ Apply u_k* =      │◀────│ Inverse Flatness   │      ┌──────────────────────────────────┐
│ [delta_k*, a_x*]  │      │ Mapping (V_k*->U*) │◀────│  Iterative Collision Detection:  │
└───────────────────┘      └────────────────────┘     │  1. Solve LTI-MPC (O_col = empty)│
                                                      │  2. Check G*y <= f + gamma       │
                                                      │  3. Add colliding obs -> O_col   │
                                                      │  4. Re-solve MIQP until safe     │
                                                      └──────────────────────────────────┘
```

---

## Installation & Usage

### Prerequisites
Ensure you have Python 3.8+ installed along with the required scientific computing packages:

```bash
pip install numpy scipy matplotlib pillow
```

### Running the Simulation
Run the complete S-curved multi-agent overtaking simulation and generate the animation:

```bash
python flatness_oa_mpc.py
```

### Simulation Scenario Description
The default benchmark simulates a two-lane S-curved road (`150 m` length) with four interacting vehicles:
* **Ego Vehicle (OA-MPC):** Starts in the right lane ($l = 0\text{ m}$) at $39.6\text{ km/h}$ ($11.0\text{ m/s}$) with a target cruising speed of $48.6\text{ km/h}$ ($13.5\text{ m/s}$).
* **Obstacle 1 & 2 (Slow Traffic):** Two vehicles traveling in the right lane at $5.5\text{ m/s}$ and $6.5\text{ m/s}$, requiring lane-change overtaking maneuvers.
* **Obstacle 3 (Oncoming Traffic):** An oncoming vehicle traveling in the left lane ($l = 3.6\text{ m}$) at $8.0\text{ m/s}$.

The controller demonstrates globally optimal decision-making: it overtakes **Obstacle 1** on the first bend, yields and merges back to the right lane to let **Obstacle 3** pass safely, and subsequently overtakes **Obstacle 2** on the second bend.

---

## Configuration Parameters

Key controller and simulation parameters (aligned with Table I of the paper) can be tuned in the `params` dictionary:

| Parameter | Symbol | Default Value | Description |
| :--- | :---: | :---: | :--- |
| `Ts` | $T_s$ | `0.05 s` | Discrete sampling period |
| `Np` | $N_p$ | `24` (`60` in paper) | Prediction horizon steps |
| `L_wb` | $L_{\text{wb}}$ | `2.7 m` | Vehicle wheelbase |
| `delta_max` | $\delta_{\max}$ | `35.0 deg` | Maximum steering angle |
| `a_max` / `a_min` | $a_{\max}, a_{\min}$ | `4.0 / -6.0 m/s²` | Longitudinal acceleration / deceleration limits |
| `qs`, `ql` | $q_s, q_l$ | `1.0, 0.35` | Longitudinal and lateral tracking weights |
| `r1`, `r2` | $r_1, r_2$ | `0.2, 0.2` | Virtual control input weights |
| `gamma` | $\gamma$ | `1.65 m` | Safety buffer distance (ego radius + margin) |
| `M` | $M$ | `150.0` | Big-M exclusion constant |
| `rs` | $r_s$ | `1e6` | Slack variable penalty weight for feasibility |

---

## Implementation Notes & Extensions
* **Solver Backend:** This implementation uses `scipy.optimize.milp` (powered by the open-source HiGHS solver) via an exact epigraph reformulation so no commercial licenses are required. For pure quadratic cost terms ($L_2$-norm) as written in Eq. (14) of the paper, the `solve_subproblem` method can be swapped to `cvxpy` with `GUROBI`, `MOSEK`, or `SCIP`.
* **Real-Time Deployment:** As noted by Gratzer et al., solving full-horizon MIQPs ($N_p = 60$) can exceed hard real-time budgets during dense multi-obstacle interactions. For real-time embedded execution, consider reducing $N_p$ or deploying a hierarchical two-layer architecture (low-frequency MIQP tactical planner + high-frequency QP trajectory tracker).
