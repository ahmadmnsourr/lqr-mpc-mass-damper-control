# Verification and known limits

## What was checked

- The original continuous model was discretized with exact zero-order hold at 0.05 seconds.
- The discrete plant has controllability rank 2.
- The augmented integral-LQR design uses the original state/input weights, and the gain is stable in the discrete-time model.
- In an independently executed Python numerical workflow, LQR gains were calculated with SciPy's discrete algebraic Riccati equation and compared against the design equations.
- MPC's stacked predictions were checked against direct state propagation. The simplified augmented predictor in this focused repository agrees with the previously verified predictor to roughly 1e-16 in double precision.
- The independently executed MPC used CasADi/qpOASES for its quadratic programs; the first QP was cross-checked against OSQP and against an absolute-input formulation in earlier validation.
- The independent simulation uses exact held-force continuous dynamics, with 1 ms output-resolution for settling-time checks.
- The sampled input bound ±1 and predicted sampled upper output bound 1.0099 are enforced. There is a small continuous intersample output exceedance; the README reports it.

## Independent measurements

See `results/independent_comparison.csv` and the two images in `figures/`.

These measurements were generated from an earlier, larger Python verification package; the focused repository intentionally omits that package to keep only the MATLAB implementation a reader needs to understand. To reproduce equivalent measurements locally, run `run_project.m`; its results will be saved as new MATLAB-named files rather than overwriting the independently verified originals.

## What was NOT checked

- The focused `run_project.m` and `solve_mpc.m` have not been executed in MATLAB. MATLAB toolboxes were unavailable here. The independently checked model mathematics and prediction matrices do not replace this runtime validation.
- The original course Simulink S-functions have not been re-executed and are not bundled.
- No code-generation, hardware, robustness, formal closed-loop MPC stability, or real-time computational performance is claimed.
- One retuned LQR design (`R=100`) is **not** proof that no other LQR tuning could meet the force and tracking objectives simultaneously.

## Design choices beyond the original exercise

- The unit-step tracking simulation runs for 15 seconds, rather than the original five-second MPC exercise, to assess sustained settling.
- A second integral-LQR run uses a larger input cost `R=100`. This is a new illustrative tuning comparison.
- The use of an explicit incremental-input QP in `solve_mpc.m` is a new readable reconstruction of the original control formulation; it does not redistribute the older MPCtools library.
