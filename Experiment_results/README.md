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
with sample variance $V\left(J_{\Omega}\right) = \frac{1}{\Omega}\sum_{i=1}^{\Omega}\left(J_{\vec{x}}^{(i)} - J_{\Omega}\right)^2$ and an approximate 95% confidence interval
$J_{\Omega} \pm 1.96 \sqrt{V\left(J_{\Omega}\right)}/\sqrt{\Omega}$.These quantities are reported in the “variance” and “confidence interval” columns of the tables.

### 1.5 ITAE indices

For a given generalized coordinate $q_i$, the accumulated Integral
Time-weighted Absolute Error (ITAE) is defined as
$
\text{ITAE}_{q_i} =
\int_{0}^{\infty} t \,\bigl|\tilde{q}_i(t)\bigr| \, dt
$
where $\tilde{q}_i(t) = q_i^{\text{ref}}(t) - q_i(t)$ is the tracking error for
that coordinate.

In discrete time, this is approximated as
$
\text{ITAE}_{q_i} \approx
\sum_{k=1}^{N} t_k \bigl|\tilde{q}_i(k)\bigr| T_s
$
with sampling period $T_s$ and time instants $t_k = k T_s$.

The scripts in `Table 4` compute:
- ITAE for base translational position (m·s).
- ITAE for base rotation (rad·s).
- ITAE norm of the arm joint angles (rad·s).

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
