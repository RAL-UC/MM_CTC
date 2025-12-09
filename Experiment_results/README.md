# Experiment results

This directory contains the data files and MATLAB scripts used to reproduce the
experimental and simulation results reported in the paper.

The subfolders are organized to mirror the figures and tables in the paper and
to separate **laboratory** and **simulation** data. This enables reviewers to
reproduce each numerical result and plot in a traceable way.

---

## 1. Performance metrics

This section summarizes the performance indices used throughout the paper and
computed by the scripts in this directory.

### 1.1 RMS end-effector tracking error

For a given trajectory $i$ sampled at instants $k = 1, \dots, N$, the RMS
end-effector tracking error is defined as $J_{\vec{x}}^{i} = e_{\text{rms}}^{i} = \sqrt{\frac{1}{N} \sum_{k=1}^{N}\left(x_k^{\text{ref}^{i}} - x_k^{i}\right)^2 + \left(y_k^{\text{ref}^{i}} - y_k^{i}\right)^2 +\left(z_k^{\text{ref}^{i}} - z_k^{i}\right)^2}$ where $(x_k^{\text{ref}}, y_k^{\text{ref}}, z_k^{\text{ref}})$ and $(x_k, y_k, z_k)$ denote the reference and measured end-effector positions at sample $k$, respectively.

This metric is used in:
- Figure 7 (simulation trajectories).
- Figure 12 (laboratory tests).
- Tables 2 and 3 (average tracking error and associated statistics).

### 1.2 RMS control power

For a trajectory $i$, the RMS control power is defined as
$J_{\vec{u}}^{i} = P_{\text{rms}}^{i} =
\sqrt{
\frac{1}{N} \sum_{k=1}^{N}
\sum_{c=1}^{n_j}
\bigl(u_{c,k}\bigr)^2
}$
where $u_{c,k}$ denotes the instantaneous control “energy” term associated
with joint $c$ at time step $k$, computed as the product of joint velocity
and applied torque (i.e., joint power).

This metric is the basis for the power plots in Figure 12 and for the RMS power
values reported in Tables 2 and 3.

### 1.3 Closing error

The closing error for a trajectory is defined as the Euclidean norm of the
final end-effector position error:
$e_{\text{close}} =\left\|\vec{x}_N^{\text{ref}} - \vec{x}_N\right\|$
with $\vec{x}_N^{\text{ref}}$ and $\vec{x}_N$ the reference and measured
end-effector positions at the final sample $N$.

Closing error is one of the scalars reported in Tables 2 and 3.

### 1.4 Expected value, variance and confidence interval

To assess repeatability over $\Omega$ repetitions of a given trajectory, the
expected tracking error is estimated as
$J_{\Omega} = \mathbb{E}\left(J_{\vec{x}}\right) = \frac{1}{\Omega}\sum_{i=1}^{\Omega} J_{\vec{x}}^{i}$
with sample variance $V\left(J_{\Omega}\right) = \frac{1}{\Omega}\sum_{i=1}^{\Omega}\left(J_{\vec{x}}^{i} - J_{\Omega}\right)^2$ and an approximate 95% confidence interval
$J_{\Omega} \pm 1.96 \sqrt{V\left(J_{\Omega}\right)/\Omega}$. These quantities are reported in the “variance” and “confidence interval” columns of the tables.

### 1.5 ITAE indices

The ITAE indices used in this repository quantify the accumulated,
time-weighted absolute tracking error of selected generalized coordinates.
Rather than focusing only on instantaneous deviations, these indices penalize
errors that persist over time, so larger and longer-lasting deviations have a
greater impact on the final value.

In the context of this work, the scripts in the `Table 4` folders compute:
- An ITAE index for the base translational displacement, expressed in m·s.
- An ITAE index for the base yaw angle, expressed in rad·s.
- An aggregate ITAE index for the arm joints, expressed as a norm in rad·s.

These quantities are obtained by processing the complete experimental or
simulation time histories and are the ones reported and compared for the CTC
and PD controllers in Table 4 of the paper.

---

## 2. Laboratory_results

This branch contains **laboratory** data and MATLAB scripts. The discrete-time
CTC and PD controllers run on the physical mobile manipulator platform
(Pioneer P3-AT + Katana 6M), and the resulting logs are saved as `.mat` files.
These data are post-processed here to obtain the figures and tables reported in
the paper.

### 2.1 Figure 12 (laboratory)

`Laboratory_results/Figure 12/` reproduces the experimental plots in Figure 12:
RMS end-effector tracking error and RMS power versus time for two laboratory
tests (Test 1 and Test 2), comparing the discrete CTC and PD controllers.

#### 2.1.1 Contents

- `Test 1/`
  - Data files:
    - `cpcdata_lab.mat`, `pddata_lab.mat`  
      Measured end-effector trajectories for CTC and PD.
    - `cpc_simlab_int.mat`, `pd_simlab_int.mat`  
      Time-aligned / interpolated trajectories used to compute
      $e_{\text{rms}}$ and closing error.
    - `ene_cpc_lab.mat`, `ene_pd_lab.mat`  
      Instantaneous joint power signals used to compute $P_{\text{rms}}$.
  - Scripts:
    - `plot_xee_test1.m`  
      Computes and plots the RMS end-effector tracking error curves
      $e_{\text{rms}}(t)$ for CTC and PD (Figure 12, upper part).
    - `plot_Prms_test1.m`  
      Computes and plots the RMS power curves $P_{\text{rms}}(t)$ for CTC and
      PD (Figure 12, lower part).

- `Test 2/`
  - Scripts:
    - `plot_xee_test2.m`, `plot_Prms_test2.m`  
      Same purpose as for Test 1, but for the second laboratory scenario.

#### 2.1.2 How to run

1. In MATLAB, set the working directory to the desired test:

```matlab
cd path_to_repo/MM_CTC/Experiment_results/Laboratory_results/Figure 12/Test 1
```

2. Ensure that all `.mat` files listed in Section 2.1.1 are present.
3. Run:

```matlab
plot_xee_test1
plot_Prms_test1
```

4. The scripts generate the time histories of $e_{\text{rms}}$ and
\(P_{\text{rms}}\) used in Figure 12 for Test 1. Repeat in `Test 2/` using
`plot_xee_test2` and `plot_Prms_test2`.

To use new experimental datasets, replace the `.mat` files with your own logs
while preserving the variable names expected by the scripts.

---

### 2.2 Table 3 (laboratory statistics)

`Laboratory_results/Table 3/` contains the script and data used to obtain the
laboratory entries of Table 3, which summarize, for each controller and test:
RMS power, closing error, expected RMS tracking error, variance and confidence
interval.

#### 2.2.1 Contents

- Data:
- `cpcdata_lab.mat`, `pddata_lab.mat`
- `cpc_simlab_int.mat`, `pd_simlab_int.mat`  
Same structure as in Figure 12.

- Script:
- `calcula_datos_lab_2.m`  
 Loads the `.mat` files, evaluates $e_{\text{rms}}$, $P_{\text{rms}}$,
 closing error, and the associated statistics over multiple repetitions
 (expected value, variance and 95% confidence interval), and prints/saves
 the values reported in Table 3.

#### 2.2.2 How to run

1. Set the working directory:

```matlab
cd path_to_repo/MM_CTC/Experiment_results/Laboratory_results/Table 3
```

2. Ensure that the four `.mat` files are present.
3. Execute:

```matlab
calcula_datos_lab_2
```

4. The script outputs the scalar metrics per controller and test, matching the
laboratory entries in Table 3.

---

### 2.3 Table 4 (laboratory ITAE indices)

`Laboratory_results/Table 4/` computes the accumulated ITAE indices listed in
Table 4 for both tests and both controllers.

#### 2.3.1 Contents

- `Test 1/` and `Test 2/`:
- Data:
 - `cpcdata_lab.mat`, `pddata_lab.mat`
 - `cpc_simlab_int.mat`, `pd_simlab_int.mat`
- Script:
 - `plot_ITAEs.m`  
   Computes the ITAE indices for:
   - Base linear position (m·s).
   - Base rotation (rad·s).
   - Norm of the arm joint angles (rad·s).

#### 2.3.2 How to run

1. Set the working directory to the desired test:

4. The script outputs the scalar metrics per controller and test, matching the
laboratory entries in Table 3.

---

### 2.3 Table 4 (laboratory ITAE indices)

`Laboratory_results/Table 4/` computes the accumulated ITAE indices listed in
Table 4 for both tests and both controllers.

#### 2.3.1 Contents

- `Test 1/` and `Test 2/`:
- Data:
 - `cpcdata_lab.mat`, `pddata_lab.mat`
 - `cpc_simlab_int.mat`, `pd_simlab_int.mat`
- Script:
 - `plot_ITAEs.m`  
   Computes the ITAE indices for:
   - Base linear position (m·s).
   - Base rotation (rad·s).
   - Norm of the arm joint angles (rad·s).

#### 2.3.2 How to run

1. Set the working directory to the desired test:

```matlab
cd path_to_repo/MM_CTC/Experiment_results/Laboratory_results/Table 4/Test 1
```

2. Verify that the `.mat` files are present.
3. Run:

```matlab
plot_ITAEs
```

4. The script prints and/or saves the ITAE values that populate the
corresponding row of Table 4.

---

## 3. Simulation_results

This branch uses **simulation** data only. The CTC and PD controllers are
executed in Simulink using the discrete-time models described in the paper, and
the resulting trajectories are saved into `.mat` files. These are then used to
reproduce the simulation figures and tables.

The folder structure mirrors the laboratory branch:

- `Figure 7/`  – simulation plots for four trajectories.
- `Figure 12/` – simulation counterpart of the laboratory plots.
- `Table 2/`   – simulation statistics for the four trajectories.
- `Table 3/` and `Table 4/` – simulation-side statistics and ITAE indices.

### 3.1 Figure 7 (simulation trajectories)

`Simulation_results/Figure 7/` contains the `.mat` files and script to generate
the plots for the four reference trajectories (square/helix in XY and YZ
planes), comparing the discrete CTC and PD controllers.

#### 3.1.1 Contents

- Data:
- `CPC_cuadrado_xy_grafico.mat`, `CPC_cuadrado_yz_grafico.mat`
- `CPC_hélice_xy_grafico.mat`, `CPC_hélice_yz_grafico.mat`
- `PD_cuadrado_xy_grafico.mat`, `PD_cuadrado_yz_grafico.mat`
- `PD_hélice_xy_grafico.mat`, `PD_hélice_yz_grafico.mat`  
Each file contains the time histories required to compute
$e_{\text{rms}}(t)$ and $P_{\text{rms}}(t)$ for a specific trajectory and
controller.

- Script:
- `graficos2.m`  
 Generates the plots reported in Figure 7, including average trajectories
 and confidence bounds; it relies on the `boundedline` toolbox included in
 `boundedline-pkg-master/`.

- Toolbox:
- `boundedline-pkg-master/`  
 Third-party toolbox providing the `boundedline` function.

#### 3.1.2 How to run

1. Set the working directory:

```matlab
cd path_to_repo/MM_CTC/Experiment_results/Simulation_results/Figure 7
```

2. Ensure all `.mat` files listed above are present and that
`boundedline-pkg-master` resides in this directory.
3. In MATLAB:

```matlab
addpath('boundedline-pkg-master');
graficos2
```

4. The script generates the plots corresponding to Figure 7.

---

### 3.2 Table 2 (simulation statistics)

`Simulation_results/Table 2/` contains the Simulink models, trajectory data and
scripts used to compute the simulation-side entries of Table 2.

#### 3.2.1 Contents

- Simulink models:
- `CPC_discreto_ruido_vel.slx`  
 Discrete-time CTC controller with sensors and actuation noise.
- `PD_discreto_ruido_vel.slx`  
 Discrete-time PD controller under the same noise conditions.

- Data:
- `cuadrado_xy.mat`, `cuadrado_yz.mat`
- `helice_xy.mat`, `helice_yz.mat`
- `cpc.mat`, `pd.mat`

- Scripts:
- `cal_cindir.m`  
 Direct kinematics utility used to map joint trajectories to end-effector
 positions.
- `estadisticas.m`  
 Main script that runs (or loads) the simulations and computes
 $e_{\text{rms}}$, $P_{\text{rms}}$, closing error and their statistics
 for each trajectory and controller, reproducing the entries in Table 2.

#### 3.2.2 How to run

1. Set the working directory:

```matlab
cd path_to_repo/MM_CTC/Experiment_results/Simulation_results/Table 2
```

2. Ensure that both Simulink models and all `.mat` files are available.
3. Run:

```matlab
estadisticas
```

4. By default, the script uses the provided `.mat` files to avoid lengthy
simulations. Comments inside `estadisticas.m` indicate how to switch to
full re-simulation if desired.

---

### 3.3 Simulation-side Table 3, Table 4 and Figure 12

The folders:

- `Simulation_results/Figure 12/`
- `Simulation_results/Table 3/`
- `Simulation_results/Table 4/`

contain data and scripts that mirror those in `Laboratory_results/`, but
computed from simulation rather than experimental logs. The usage pattern is
analogous:

1. Change to the corresponding directory.
2. Ensure the `.mat` files are present.
3. Run the associated script (`plot_xee_test*`, `plot_Prms_test*`,
`calcula_datos_lab_2.m`, `plot_ITAEs.m`) to generate the simulation-side
plots and scalar metrics. 

---

## 4. Software requirements

- MATLAB (version used in the paper: please specify here, e.g. R2020b or older).
- Simulink (required for scripts that invoke `*.slx` models, such as those in
`Simulation_results/Table 2`).
- No additional MathWorks toolboxes are strictly required beyond the base
installation for running the scripts as provided.
- The `boundedline` toolbox is included locally in
`Simulation_results/Figure 7/boundedline-pkg-master` and is added to the path
by `graficos2.m` (or manually via `addpath` as shown above).

All scripts assume that the `.mat` files they use reside in the same directory
as the script itself, so no manual path changes are necessary in the default
configuration.
