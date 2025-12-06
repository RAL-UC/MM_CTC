# PSO-Based Gain Tuning for Discrete-Time Controllers

This directory implements the Particle Swarm Optimization (PSO)–based gain tuning procedure for the discrete-time PD and Computed Torque Control (CTC) schemes used in the skid-steer mobile manipulator trajectory tracking study. 

## 1. Overview

The mobile manipulator is controlled by two discrete-time whole-body controllers:

- A discrete-time Computed Torque Controller (CTC) operating on the coupled base–arm dynamics.
- A discrete-time PD controller used as a reference baseline.

Both controllers share a common 10-dimensional gain vector
$\left[K_{p1}, K_{p2}, K_{p3}, K_{p4}, K_{p5}, K_{v1}, K_{v2}, K_{v3}, K_{v4}, K_{v5}\right]$
where the first five entries correspond to proportional gains and the last five entries to derivative gains for the selected generalized coordinates.

The tuning procedure uses PSO to search over this gain space and minimize a performance index that jointly captures end-effector tracking performance and control-energy usage.

## 2. Tuning methodology

For a given gain vector $x \in \mathbb{R}^{10}$, the objective function proceeds as follows:

1. Sets the controller gains $K_{p_i}$, $K_{v_i}$ in the MATLAB base workspace.
2. Randomizes sensor and actuator noise seeds to reflect measurement and actuation uncertainties.
3. Simulates the selected discrete-time controller (CTC or PD) along predetermined reference trajectories using the corresponding Simulink model.
4. Computes:
   - The root-mean-square (RMS) end-effector position tracking error $J_{\text{traj}}$ in Cartesian space.
   - The RMS control power $J_{\text{ener}}$ from joint torque–velocity products.
5. Forms the scalar cost:
   $J = \alpha \ J_{\text{traj}} + \beta \ J_{\text{ener}}$,
   where $\alpha$ and $\beta$ are positive weights defined in the objective function to balance tracking accuracy and energy consumption.

The PSO algorithm iteratively updates a swarm of candidate solutions and converges to a gain vector that minimizes $J$ for each controller type.

## 3. File structure

The key files in this directory and its immediate dependencies are:

- `run_pso_tuning.m`  
  Main entry point. Orchestrates PSO-based tuning for both controllers, handles repository-aware path management, and writes the tuned gains to `.mat` files.

- `ObjectiveFunction_error_pos.m`  
  Simulation-based objective function. Given a gain vector and a controller identifier (`'CTC'` or `'PD'`), it configures the workspace, runs the appropriate Simulink model, and returns the scalar cost $J$.

- `PSO2.m`  
  PSO implementation used for gain optimization. Accepts the problem definition (bounds, dimension, objective function) and returns:
  - `GBEST` (global best solution, including `GBEST.X` with the optimal gains).
  - `cgCurve` (convergence history).
  - `Swarm_save` (per-iteration swarm data, including `Swarm_save{1,end}.GBEST.X`).

- `Discrete_CTC.slx`  
  Discrete-time Simulink model implementing the whole-body CTC for the mobile manipulator, including the sampled controller, plant, and measurement noise/actuator disturbances.

- `Discrete_PD.slx`  
  Discrete-time Simulink model implementing the PD controller for the same mobile manipulator and trajectories, sharing the same logging interfaces as `Discrete_CTC.slx`.

Downstream consumers (e.g., `run_trajectory_experiments.m`) expect to find tuned gains in `.mat` files with a `Swarm_save` cell array whose last element contains the best gain vector.

## 4. Controller models and interfaces

Both controller models (`Discrete_CTC.slx` and `Discrete_PD.slx`) expose a consistent set of logged outputs used by the objective function:

- `simout.xd(:,2:4)`  
  Reference end-effector positions ($x_{\text{ref}}$, $y_{\text{ref}}$, $z_{\text{ref}}$).

- `simout.xa(:,2:4)`  
  Measured end-effector positions ($x_{\text{meas}}$, $y_{\text{meas}}$, $z_{\text{meas}}$).

- `simout.energias(:,2:6)`  
  Joint-level power-related quantities for the five actuated coordinates, used to compute the RMS energy term. 

Controller parameters and noise seeds are passed through the MATLAB base workspace:

- Gains: `Kp1..Kp5`, `Kv1..Kv5`.
- Sensor noise seeds: `r1..r5`.
- Actuator noise seeds: `t1..t5`.

This shared interface allows the same objective function to be used for both controllers by simply switching the controller identifier (`'CTC'` or `'PD'`), while the underlying Simulink model remains modular and specific to each control law.

## 5. Usage

### 5.1. Prerequisites

- MATLAB and Simulink installed.
- All `.m` files (`run_pso_tuning.m`, `ObjectiveFunction_error_pos.m`, `PSO2.m`) on the MATLAB path.
- Simulink models:
  - `Discrete_CTC.slx`
  - `Discrete_PD.slx`  
  configured with the logging interfaces described above.

If the code is used within the full repository, the expected structure is:

- `<repo_root>/PSO_tuning/` – this directory (`run_pso_tuning.m`).
- `<repo_root>/Controller_implementation/Simulation/` – simulation and trajectory scripts (e.g., `run_trajectory_experiments.m`).

### 5.2. Running the tuner

1. Start MATLAB and change directory to the PSO tuning folder, typically:

```matlab
cd('<repo_root>/PSO_tuning');
```

2. Run the tuning script:

```matlab
run_pso_tuning
```

3. At the interactive prompt, select one of:
- `1` – Tune the discrete-time CTC controller.
- `2` – Tune the discrete-time PD controller.
- `3` – Tune both controllers sequentially.
- `0` – Cancel.

4. The script will:
- Configure the PSO problem (dimensionality, bounds, objective function).
- Execute PSO for the selected controller(s).
- Print the best cost value and the corresponding gain vector `GBEST.X`.
- Store the full swarm history in `Swarm_save`.

### 5.3. Output files

For each tuned controller, the script produces a gains file:

- `ctc.mat` for the CTC controller.
- `pd.mat` for the PD controller.

If the expected repository structure is detected, each gains file is written to:

- `<repo_root>/Controller_implementation/Simulation/<controller>.mat`

so that simulation scripts can load the tuned gains transparently.
If the repository structure is not detected (e.g., when the tuning code is used standalone), the gains files are written to the current working directory, ensuring that `ctc.mat` and `pd.mat` are always available to the user.

The files include at least:

- `Swarm_save` – PSO swarm history per iteration.
- `Swarm_save{1,end}.GBEST.X` – best gain vector.
- `GBEST` – global best solution structure.
- `cgCurve` – convergence curve of best cost vs. iteration.
- `metadata` – configuration and provenance information (controller type, swarm size, number of iterations, timestamps).
- `problem` – problem definition (bounds, dimension, objective function handle used).

## 6. Reproducibility and auditing

The PSO parameters (swarm size, number of iterations, random seed) are explicitly set within `run_pso_tuning.m`, and the cost computation is fully defined in `ObjectiveFunction_error_pos.m`, using logged signals from the discrete-time models.
This design makes it possible for reviewers and researchers to reproduce the tuning experiments, verify the cost definition, and inspect the intermediate swarm states stored in `Swarm_save` to confirm convergence behavior and robustness with respect to noise.