# Laboratory implementation

This directory contains the discrete-time controller implementation used on the
real mobile manipulator and the raw laboratory data collected for the
comparative evaluation between the discrete-time CTC and PD controllers.

The implementation is monolithic by design: a single main program configures
the robot, acquires camera data, and executes either the CTC or PD controller
based on a user selection at runtime. The subfolders below reflect this
structure and the raw logs obtained from it.

- `c_src_MM_CTC/` and `c_src_MM_PID/` – variants of the same monolithic
  laboratory program.
- `c_src_camera_red_target_segmentation/` – standalone example of the red
  target segmentation pipeline.
- `Raw data/` – unprocessed logs for CTC and PD experiments.

---

## 1. Monolithic controller code

### 1.1 Shared structure in `c_src_MM_CTC` and `c_src_MM_PID`

Both `c_src_MM_CTC/` and `c_src_MM_PID/` contain the same core source files and
represent two instances of the same monolithic laboratory program:

- `main.cpp`  
  Central entry point. It:
  - initializes communication with the Pioneer P3-AT base and Katana 6M arm,
  - initializes the camera and runs the red-target segmentation,
  - queries the user to select which controller to run (discrete CTC or PD),
  - enters the discrete-time control loop with a sampling period consistent
    with the design in the manuscript (10 ms).

- `base_control.cpp`, `base_control.h`  
  Routines to send velocity/torque commands to the mobile base and read back
  base state information. These functions are used by `main.cpp` in both CTC
  and PD modes.

- `robot_control.cpp`, `robot_control.h`  
  Routines to command the arm joints and read their state, encapsulating the
  low-level control of the Katana 6M arm. The higher-level control law (CTC or
  PD) is implemented in `main.cpp`, which calls these routines to apply the
  computed torques/commands.

- `katana6M180.cfg`  
  Configuration file for the Katana 6M arm (kinematic and communication
  parameters).

- `sched.h`  
  Auxiliary definitions used to implement the fixed-sampling control loop
  (scheduling primitives, timing utilities).

In practice, these two folders should be regarded as two variants of the same
laboratory program rather than completely independent implementations. They are
kept separate to preserve the original development structure and to simplify
building and archiving.

### 1.2 Camera segmentation example

The folder `c_src_camera_red_target_segmentation/` contains a standalone
example of the red-target segmentation procedure:

- `camara_segmenta_rojo.cpp`  
  Demonstrates how red pixels are thresholded and how the target centroid is
  extracted using OpenCV.

- `camera_red_target_segmentation.sln`, `*.ncb`, `*.suo`  
  Visual Studio solution and auxiliary files.

The `Debug/` subfolder contains precompiled binaries and the required OpenCV
runtime libraries for the original development environment.

In the actual experiments, equivalent camera processing logic is integrated
directly into `main.cpp` of the monolithic program described above. This folder
is preserved as a minimal, self-contained example.

---

## 2. Raw laboratory data

The `Raw data/` directory stores the unprocessed logs collected on the real
platform for both controllers. These files are converted into `.mat` files and
used by the analysis scripts in `Experiment_results/Laboratory_results/` to
compute the metrics reported in the manuscript.

### 2.1 CTC experiments (`Raw data/CTC/`)

- `datos_th0_*.txt`, `datos_th1_*.txt`, `datos_th2_*.txt`  
  Time series of base and arm signals (for different trials and test
  scenarios) when using the discrete-time CTC controller.

- `itae_base_*.txt`, `itae_base_th*.txt`  
  Intermediate ITAE-related accumulations obtained during the CTC experiments.

- `cpcdata_test1.mat`, `cpcdata_test2.mat`  
  Processed MATLAB data files that aggregate the relevant variables for Tests 1
  and 2 (e.g. end-effector trajectories, base pose, control signals).

- `CTC_data_processing.m`  
  MATLAB script that reads the text logs, performs basic processing (e.g.
  scaling, synchronization), and generates the `.mat` files required by the
  analysis scripts in `Experiment_results/Laboratory_results/`.

### 2.2 PD experiments (`Raw data/PD/`)

- `datos_th*_*.txt`, `itae_base*.txt`, `itae_base_th_*.txt`  
  Analogous logs for the discrete-time PD controller.

- `pddata_test1.mat`, `pddata_test2.mat`  
  Processed MATLAB data files summarizing the PD experiments.

- `pd_data_processing.m`  
  MATLAB script that converts the PD logs into `.mat` files compatible with the
  analysis pipeline used for the CTC data.

These raw logs provide full traceability from the monolithic C/C++ program
running on the hardware to the performance indices reported in the manuscript.

---

## 3. Usage guidelines

### 3.1 Building and running the laboratory program

To run the monolithic laboratory controller on the real robot:

1. Open the Visual Studio project associated with either `c_src_MM_CTC` or
   `c_src_MM_PID` (both contain the same main program with runtime controller
   selection).
2. Configure the include paths and libraries for:
   - Aria (Pioneer P3-AT interface),
   - Katana SDK,
   - OpenCV (for camera access and red-target segmentation).
3. Build the project (Debug or Release) and deploy the executable to the PC
   connected to the Pioneer P3-AT and Katana 6M arm.
4. Run the executable. On startup, the program:
   - initializes the base, arm and camera interfaces,
   - prompts the user to select the controller type (CTC or PD),
   - enters the discrete-time control loop and logs experiment data to the
     `Raw data/CTC` or `Raw data/PD` folders according to the selected mode.

The control loop frequency is configured to match the sampling period used in
the discrete-time design (10 ms).

### 3.2 Processing raw logs

To regenerate the `.mat` files from raw logs:

- For CTC:

```matlab
cd('<repo_root>/Controllers_implementation/Laboratory/Raw data/CTC');
CTC_data_processing
```

- For PD:

```matlab
cd('<repo_root>/Controllers_implementation/Laboratory/Raw data/PD');
pd_data_processing
```

The resulting `.mat` files are then used by the scripts in
`Experiment_results/Laboratory_results/` to compute RMS tracking error, RMS
power, closing error, variance, confidence intervals and ITAE-type indices for
each test and controller.

---

In summary, this directory consolidates the monolithic C/C++ implementation of
the discrete-time CTC and PD controllers, the integrated camera-based target
segmentation, and the raw laboratory data used to support the experimental
results and performance comparisons presented in the manuscript.