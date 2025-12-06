# Simulation and Controller Evaluation

This folder contains the MATLAB scripts, Simulink models, and precomputed
data used to implement and evaluate the discrete-time controllers for the
skid-steer mobile manipulator. The focus is on trajectory tracking using a
whole-body discrete-time Computed Torque Controller (CTC) and a discrete-time
Proportional–Derivative (PD) controller, both tuned by Particle Swarm
Optimization (PSO) and compared in terms of tracking accuracy and control
energy.

The simulations implemented here correspond to the discrete-time control
design and performance assessment described in **Sections 5–6** of the
manuscript, including the four test trajectories (square and helix paths in
the XY and YZ planes) and the metrics reported in **Table 2** and illustrated
in **Figure 7**.

> For installation, global requirements, and basic repository usage, refer to
> the top-level `README.md` in the repository root.

---

## Contents

- `Discrete_CTC.slx`  
  Simulink implementation of the discrete-time Computed Torque Controller
  for the coupled mobile manipulator. This model integrates:
  - the discrete-time state-space model derived from the coupled dynamics,
  - the CTC law implemented directly in discrete time, and
  - actuator-level mapping for the skid-steer base and 3-DOF arm.  
  It is used to generate the CTC simulation results (simulation portion of
  **Table 2** and the blue curves in **Figure 7**).

- `Discrete_PD.slx`  
  Simulink implementation of the discrete-time PD controller used as a
  reference baseline. The plant dynamics and trajectory inputs are identical
  to those of `Discrete_CTC.slx`; only the control law differs, being a
  discrete-time PD scheme designed from the linearized, discretized model and
  tuned via PSO.

- `run_trajectory_experiments.m`  
  Main script to run Monte Carlo simulations for both controllers and
  trajectories. It:
  - reads the PSO-optimized gains (`ctc.mat` or `pd.mat`),
  - provides a menu to select controller (CTC/PD) and trajectory,
  - configures sensor and actuator noise independently for each repetition,
  - runs the selected Simulink model for the appropriate simulation horizon,
  - saves raw performance data for post-processing.  
  The script produces one `.mat` file per controller–trajectory pair, ready to
  be consumed by higher-level analysis scripts (e.g., for regenerating
  **Table 2** and **Figure 7**).

- `ctc.mat`, `pd.mat`  
  PSO-optimized gain sets for the discrete-time CTC and PD controllers,
  respectively. Each file stores the global-best particle of the PSO search,
  obtained by minimizing a cost function that combines end-effector RMS
  tracking error and RMS power, as defined in Equations (17)–(19) of the
  manuscript.

- Trajectory files (`*.mat`)  
  Joint-space reference trajectories for the four test cases:
  - `square_xy.mat` – square path in the XY plane  
  - `square_yz.mat` – square path in the YZ plane  
  - `helix_xy.mat`   – helical path in the XY plane  
  - `helix_yz.mat`   – helical path in the YZ plane  

  Each file contains a matrix with the joint references used in the simulations, 
  consistent with the trajectories described in **Section 6**.

---

## Relation to the manuscript and other folders

- The simulation models in this folder use the coupled base–arm dynamic model
  and the discrete-time linearization/discretization derived in
  `Modelling_and_Discretization`, which implement the formulation of
  **Section 3** and the discretization procedure of **Section 5**.

- The recovered matrices \(M(q)\), \(C(q,\dot{q})\), and \(G(q)\) from the
  modeling folder correspond to the inverse dynamics equation in
  **Equation (6)** and are used in the definition of the CTC law in
  **Equations (7)–(8)**, which is then discretized and implemented here.

- The discrete-time plant model, sampling time, and equilibrium point used in
  the Simulink models are consistent with those employed for the discrete root
  locus analysis and controller tuning in **Section 5**.

- The four trajectories defined by the `*.mat` files match the test cases used
  to generate the simulation results summarized in **Table 2** and plotted in
  **Figure 7**.

---

## How to run simulations (from this folder)

> The following steps assume that MATLAB and Simulink are installed and
> configured as described in the top-level README.

1. **Ensure the repository is on the MATLAB path**  
   (see root `README.md` for the recommended `addpath` command).

2. **Navigate to the simulation folder in MATLAB**

```matlab
cd('<repo_root>/Controller_Implementation/Simulation');
```

3. **Run the experiment script**

```matlab
run_trajectory_experiments
```


The script will:

- Ask for the controller:
  - `Discrete_CTC` (CTC),
  - `Discrete_PD` (PD).
- Ask for the trajectory:
  - square XY, square YZ, helix XY, helix YZ, or all of them.
- Automatically:
  - load the corresponding PSO-optimized gains (`ctc.mat` or `pd.mat`),
  - configure random sensor and actuator noise for each repetition,
  - execute the selected Simulink model with the appropriate stop time,
  - save the results to `.mat` files.

4. **Inspect the output files**

By default, results are written to:

```matlab
<repo_root>/Experiment_Results/Simulation_Results/
```

If this folder does not exist (e.g., when running the script standalone),
results are saved to the current working directory.

Each file is named as:

<Controller>_<Trajectory>_results.mat
e.g., CTC_square_xy_results.mat, PD_helix_yz_results.mat

and contains:

- `J_tracking` – RMS tracking error per repetition and timestep  
- `J_energy` – RMS power per repetition and timestep  
- `t_vec` – time vector  
- `metadata` – controller, trajectory, stop time, and configuration info

These files are the inputs to the analysis/plotting scripts that reconstruct
the numerical results reported in the manuscript (e.g., **Table 2**).

---

## Notes

- The sampling time in the Simulink models is \(T_s = 10 \mathrm{ms}\),
consistent with the discrete-time design and the modeling scripts in
`Modelling_and_Discretization`.

- Sensor and actuator noise are included to emulate encoder quantization and
actuation disturbances, as described in the simulation setup of **Section 6**,
and are randomized across repetitions to enable Monte Carlo–style averages.

- Controller retuning (PSO) and model derivation (Featherstone-based
dynamics, linearization, and discretization) are handled in separate
directories (`PSO_Tuning` and `Modelling_and_Discretization`) and documented
in their respective READMEs.