# LQR vs Constrained MPC — Mass–Damper Position Tracking

**A focused MATLAB control project about actuator limits and tracking performance.**

> **Research question:** Can model predictive control follow a position step in about two seconds **without exceeding a force limit of ±1**, when the original integral-action LQR demands more force?

This project reconstructs one coherent problem from *Digital Control Technologies and Architectures* coursework at Politecnico di Torino. Rather than covering every assignment, it compares three controller configurations on the **same plant and reference**:

1. **Integral LQR (original)** — fast tracking, no explicit input constraint.
2. **Integral LQR, R = 100 (new tuning)** — a simple input-penalty alternative, not a constrained LQR.
3. **Constrained MPC (original tracking settings)** — predictive control with explicit force and predicted-output bounds.

The entire MATLAB implementation has **two files**: [`run_project.m`](run_project.m), which walks through the experiment, and [`solve_mpc.m`](solve_mpc.m), which solves one quadratic program. The sections and comments follow the straightforward teaching style of the course's MATLAB examples.

## The model

The original coursework gives the normalized mass–damper equations:

$$
\dot{x}=\begin{bmatrix}0&1\\0&-1\end{bmatrix}x+
\begin{bmatrix}0\\1\end{bmatrix}u,\qquad
 y=\begin{bmatrix}1&0\end{bmatrix}x.
$$

Here $x_1$ is position, $x_2$ is velocity, and $u$ is applied force. The controller is sampled every $T_s=0.05$ s; MATLAB uses `ss` and `c2d(...,'zoh')` to obtain the discrete-time system.

The experiment uses a **unit-step position reference**, starting at rest, and compares all controllers over the same 15-second simulation. The longer simulation (beyond the original five-second MPC exercise) allows us to check whether the response remains inside the 1% tolerance band.

## The controllers

**Integral LQR.** An error integrator is added to the discrete states:

$$q_{k+1}=q_k+T_s(r-y_k),\qquad u_k=-K[q_k\quad x_k^T]^T.$$

The original controller uses $Q=\operatorname{diag}(850,150,1)$ and $R=1$. A second run changes only $R$ to 100 to explore the cost of reducing force demand. It **does not** enforce a hard force bound.

**Constrained MPC.** At each sample, MPC predicts 25 future positions and optimizes 25 changes in force $\Delta u$. Its original tracking cost is

$$
J=\sum_{i=1}^{25}120000(y_{k+i|k}-1)^2
+\sum_{i=0}^{24}120(\Delta u_{k+i|k})^2,
$$

subject to the predicted **sampled-time** constraints

$$-1\leq u_{k+i|k}\leq1,\qquad y_{k+i|k}\leq1.0099.$$

The optimizer is MATLAB's `quadprog`. The code applies only the first optimized force change before solving again at the next sample. **This controller does not have a hard input-rate bound**; the increment is penalized in the cost. The input-rate-constrained regulation problem was excluded to keep this repository focused.

## Results

The figures and table below were generated with an **independently executed Python/SciPy + CasADi reconstruction** of the same specified model and controller designs, *not* with MATLAB or the original Simulink files. Running the MATLAB script produces separately named MATLAB outputs.

### Position response

![Position response comparing three controllers](figures/tracking_independent.png)

### Applied force

![Applied force comparing three controllers](figures/force_independent.png)

| Controller | 1% settling time | Peak \|force\| | Input energy $T_s\sum u_k^2$ |
|---|---:|---:|---:|
| Integral LQR, original | 2.014 s | **3.121** | 3.885 |
| Integral LQR, $R=100$ (new tuning) | 5.165 s | **0.866** | 0.796 |
| Constrained MPC | **1.991 s** | **1.000** | 2.145 |

**Interpretation:** The original integral LQR reaches the tracking target but would exceed a ±1 actuator limit. Increasing its input penalty reduces the demanded force below the limit *in this nominal simulation*, but its settling time becomes much longer. The configured MPC gives similar nominal settling to the original LQR while explicitly respecting the ±1 force bound.

This does **not** mean MPC always outperforms LQR, or that this is the optimal LQR tuning. The original LQR and MPC also minimize different cost functions. The experiment isolates a practical constraint-handling trade-off for these chosen designs.

**Sampled-data detail:** The MPC meets its upper output bound at the prediction/sampling instants. The independent reconstruction found a small **intersample exceedance** of approximately $5.37\times10^{-5}$ above 1.0099, so we do not claim a continuous-time output guarantee. The force is held constant between samples, so its sampled magnitude limit also applies between samples.

## Run in MATLAB

**Requirements:** MATLAB with **Control System Toolbox** (`ss`, `c2d`, `dlqr`, `ctrb`) and **Optimization Toolbox** (`quadprog`). MATLAB R2018b or newer is recommended for `yline`, along with `optimoptions`. No Simulink model or external MPCtools package is needed.

1. Download and extract the repository.
2. Set MATLAB's Current Folder to the extracted folder.
3. Run:

```matlab
run_project
```

The command prints a comparison table and generates exactly three output files:

- `figures/tracking_matlab.png` — position tracking.
- `figures/force_matlab.png` — applied-force comparison.
- `results/matlab_comparison.csv` — computed performance metrics.

The MATLAB and independent figures are intentionally saved under **different names** to make the provenance unambiguous. The MATLAB script includes a controllability check, a closed-loop LQR stability check, and QP constraint-residual checks. It uses the exact held-force solution between samples when measuring tracking time.

## Read the code

Read [`run_project.m`](run_project.m) from top to bottom:

1. Define the plant and discretize it.
2. Design the LQR controllers using `dlqr`.
3. Build simple prediction matrices for MPC.
4. Simulate all three controllers on the same plant.
5. Compute metrics and create two figures.

The other file, [`solve_mpc.m`](solve_mpc.m), turns the prediction matrices into one `quadprog` problem. It is kept separate because that QP repeats at every sample; the main script remains readable.

## Verification and scope

- Original MATLAB/Simulink execution: **not yet performed** in this environment. The MATLAB code is newly reconstructed and still requires a local MATLAB run.
- Independent Python verification: **performed** for the same plant, prediction matrices, cost weights, and constraints. The first MPC QP was also compared across independent QP formulations and solvers in the earlier validation workflow.
- A new check confirmed that the **simplified** augmented prediction matrices agree with the previously verified implementation to numerical precision.
- The sample figure/CSV values were copied from that earlier executed independent validation. They are **not claimed as generated by the new MATLAB code**.
- This is a deterministic, full-state, nominal linear model with no measurement noise or external disturbances. It is **not** a vehicle model and offers no formal robustness or real-time feasibility guarantees.

See [verification notes](VALIDATION.md) for details.

## Coursework attribution

Original plant, reference-tracking requirements, and controller configurations are from *Digital Control Technologies and Architectures*, Politecnico di Torino (course materials by M. Canale): the discrete-LQR/integral-action and predictive tracking exercises. For teaching style and derivations, this refactor also consulted `DiCTAr_L06_v07_example_1.m.pdf` (straightforward MATLAB examples), `DiCTAr_L07_v07.pdf` (LQ control), and `DiCTAr_L08_v09_examples (1).pdf` (finite-horizon quadratic optimization and receding-horizon control) supplied in `Lectures.zip`.

The two MATLAB implementation files here are **newly written reconstructions**, not copies of the university scripts. The original PDFs, Simulink models, and MPCtools source are not redistributed. The original coursework values are clearly distinguished from the new $R=100$ comparison and extended simulation duration.
