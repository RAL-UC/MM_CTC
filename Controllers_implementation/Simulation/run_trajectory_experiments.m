function run_trajectory_experiments()
%RUN_TRAJECTORY_EXPERIMENTS Monte Carlo simulation for trajectory tracking
%   Executes Simulink-based trajectory tracking experiments with randomized
%   sensor and actuator noise for discrete-time CTC and PD controllers.
%
%   This script generates .mat files containing raw RMS tracking error and
%   RMS power for each repetition and timestep. Post-processing (tables,
%   plots) is done in a separate analysis script.
%
%   Repository Structure (after cloning):
%       <repo_root>/
%       ├── Controller_Implementation/
%       │   ├── Simulation/          <- Run this script from here
%       │   └── Laboratory/
%       ├── PSO_Tuning/
%       ├── Modelling_and_Discretization/
%       └── Experiment_Results/
%           ├── Simulation_Results/  <- Preferred output directory
%           └── Laboratory_Results/
%
%   If the expected repository structure is not found, results are saved in
%   the current working directory.
%
%   Usage:
%       1. Clone the repository
%       2. Open MATLAB and navigate to Controller_Implementation/Simulation/
%       3. Run: >> run_trajectory_experiments()
%
%   Output files:
%       <Controller>_<Trajectory>_results.mat
%       e.g., CTC_square_xy_results.mat, PD_helix_yz_results.mat
%
%   Each file contains:
%       J_tracking   - RMS tracking error per repetition and timestep
%       J_energy     - RMS power per repetition and timestep
%       t_vec        - Time vector
%       metadata     - Struct with experiment configuration
%
%   Requirements:
%       - MATLAB 2020b or later
%       - Simulink models: Discrete_CTC.slx, Discrete_PD.slx
%       - Gains files: ctc.mat, pd.mat
%       - Trajectory files: cuadrado_xy.mat, cuadrado_yz.mat,
%                           helice_xy.mat, helice_yz.mat
%
%   Author: Patricio Galarce-Acevedo
%   Date: December 2025

%% ======================= PATH CONFIGURATION =======================
[script_path, ~, ~] = fileparts(mfilename('fullpath'));
if isempty(script_path)
    script_path = pwd;
end

repo_root       = fullfile(script_path, '..', '..');
expected_output = fullfile(repo_root, 'Experiment_Results', 'Simulation_Results');

if exist(fullfile(repo_root, 'Experiment_Results'), 'dir')
    output_path = expected_output;
    using_repo_structure = true;
else
    output_path = pwd;
    using_repo_structure = false;
end

PATHS = struct(...
    'output',       output_path, ...
    'models',       script_path, ...
    'gains',        script_path, ...
    'trajectories', script_path ...
);

%% ======================= CONFIGURATION =======================
CONFIG = struct(...
    'num_joints',  5,  ...   % 5 DOF used in trajectories
    'num_reps',    10, ...   % Monte Carlo repetitions
    'noise_scale', 100 ...   % Scaling factor for noise seeds
);

% Controllers: {model_name, gains_file, short_name, description}
CONTROLLERS = {
    'Discrete_CTC', 'ctc.mat', 'CTC', 'Computed Torque Controller';
    'Discrete_PD',  'pd.mat',  'PD',  'Proportional-Derivative Controller'
};

% Trajectories: {stop_time, traj_name, traj_file, description}
TRAJECTORIES = {
    119.99, 'square_xy', 'square_xy.mat',    'Square trajectory - XY plane';
    399.99, 'square_yz', 'square_yz.mat',    'Square trajectory - YZ plane';
    200.00, 'helix_xy',  'helix_xy.mat',      'Helix trajectory - XY plane';
    200.00, 'helix_yz',  'helix_yz.mat',      'Helix trajectory - YZ plane'
};

%% ======================= INITIALIZATION =======================
clc; close all; warning('off', 'all');
print_header();

if ~exist(PATHS.output, 'dir')
    mkdir(PATHS.output);
end

if using_repo_structure
    fprintf('[INFO] Repository structure detected.\n');
    fprintf('       Output path: %s\n\n', PATHS.output);
else
    fprintf('[WARNING] Repository structure not found.\n');
    fprintf('          Results will be saved in current directory:\n');
    fprintf('          %s\n\n', PATHS.output);
end

[ctrl_idx, traj_idx] = display_main_menu(CONTROLLERS, TRAJECTORIES);
if ctrl_idx == 0 || traj_idx == 0
    fprintf('\n[INFO] Operation cancelled by user.\n');
    return;
end

%% ======================= EXECUTION =======================
model_name = CONTROLLERS{ctrl_idx, 1};
gains_file = fullfile(PATHS.gains, CONTROLLERS{ctrl_idx, 2});
ctrl_short = CONTROLLERS{ctrl_idx, 3};
ctrl_desc  = CONTROLLERS{ctrl_idx, 4};

verify_required_files(model_name, gains_file);
load_controller_gains(gains_file, CONFIG.num_joints);

fprintf('\n[INFO] Controller: %s (%s)\n', ctrl_short, ctrl_desc);

output_files = {};

if traj_idx == 5
    for k = 1:size(TRAJECTORIES, 1)
        out_file = run_single_experiment(k, TRAJECTORIES, model_name, ...
                                         ctrl_short, CONFIG, PATHS);
        output_files{end+1} = out_file;
    end
else
    out_file = run_single_experiment(traj_idx, TRAJECTORIES, model_name, ...
                                     ctrl_short, CONFIG, PATHS);
    output_files{end+1} = out_file;
end

print_summary(output_files, PATHS.output);

end

%% ======================= DISPLAY FUNCTIONS =======================

function print_header()
    fprintf('\n');
    fprintf('╔═══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║     DISCRETE-TIME CONTROLLER SIMULATION FOR MOBILE MANIPULATORS   ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════════════╝\n\n');
end

function [ctrl_idx, traj_idx] = display_main_menu(controllers, trajectories)
    ctrl_idx = select_controller(controllers);
    if ctrl_idx == 0
        traj_idx = 0;
        return;
    end
    traj_idx = select_trajectory(trajectories);
end

function selection = select_controller(controllers)
    fprintf('╔═══════════════════════════════════════════════════════════╗\n');
    fprintf('║              STEP 1: SELECT CONTROLLER                    ║\n');
    fprintf('╠═══════════════════════════════════════════════════════════╣\n');
    for i = 1:size(controllers, 1)
        fprintf('║  [%d] %-50s   ║\n', i, controllers{i, 4});
    end
    fprintf('╠═══════════════════════════════════════════════════════════╣\n');
    fprintf('║  [0] Cancel                                               ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════╝\n\n');
    selection = get_valid_input('Select controller', 0, size(controllers, 1));
end

function selection = select_trajectory(trajectories)
    fprintf('\n');
    fprintf('╔═══════════════════════════════════════════════════════════╗\n');
    fprintf('║              STEP 2: SELECT TRAJECTORY                    ║\n');
    fprintf('╠═══════════════════════════════════════════════════════════╣\n');
    for i = 1:size(trajectories, 1)
        fprintf('║  [%d] %-40s        ║\n', i, trajectories{i, 4});
        fprintf('║      Duration: %7.2f s                                 ║\n', trajectories{i, 1});
    end
    fprintf('╠═══════════════════════════════════════════════════════════╣\n');
    fprintf('║  [5] Run ALL trajectories sequentially                    ║\n');
    fprintf('║  [0] Cancel                                               ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════╝\n\n');
    selection = get_valid_input('Select trajectory', 0, 5);
end

function value = get_valid_input(prompt, min_val, max_val)
    while true
        value = input(sprintf('%s (%d-%d): ', prompt, min_val, max_val));
        if isnumeric(value) && isscalar(value) && ...
           value >= min_val && value <= max_val && floor(value) == value
            return;
        end
        fprintf('[WARNING] Invalid input. Enter a number between %d and %d.\n', ...
                min_val, max_val);
    end
end

function print_summary(output_files, output_path)
    fprintf('\n');
    fprintf('╔═══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║                    SIMULATION COMPLETED                           ║\n');
    fprintf('╠═══════════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Output directory:                                                ║\n');
    fprintf('║  %s\n', output_path);
    fprintf('╠═══════════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Generated files (ready for analysis/plotting):                   ║\n');
    for i = 1:length(output_files)
        [~, fname, ext] = fileparts(output_files{i});
        fprintf('║    - %-57s  ║\n', [fname, ext]);
    end
    fprintf('╚═══════════════════════════════════════════════════════════════════╝\n\n');
end

%% ======================= CORE FUNCTIONS =======================

function verify_required_files(model_name, gains_file)
    model_file = [model_name, '.slx'];
    if ~exist(model_file, 'file')
        error('Simulink model not found: %s', model_file);
    end
    if ~exist(gains_file, 'file')
        error('Controller gains file not found: %s', gains_file);
    end
end

function load_controller_gains(gains_file, num_joints)
    data  = load(gains_file);
    gbest = data.Swarm_save{1, end}.GBEST.X;
    for j = 1:num_joints
        assignin('base', sprintf('Kp%d', j), gbest(j));
        assignin('base', sprintf('Kv%d', j), gbest(j + num_joints));
    end
    fprintf('[INFO] PSO-optimized gains loaded from: %s\n', gains_file);
end

function initialize_noise_seeds(num_joints, scale)
    for j = 1:num_joints
        rng('shuffle');
        assignin('base', sprintf('r%d', j), abs(rand) * scale);
        rng('shuffle');
        assignin('base', sprintf('t%d', j), abs(rand) * scale);
    end
end

function [J_tracking, J_energy] = compute_metrics(simOut)
    xd = simOut.xd(:, 2:4);
    xa = simOut.xa(:, 2:4);
    error_xyz  = xd - xa;
    J_tracking = sqrt(sum(error_xyz.^2, 2))';

    energy   = simOut.energias(:, 2:6);
    J_energy = sqrt(sum(energy.^2, 2))';
end

function output_file = run_single_experiment(traj_idx, trajectories, model_name, ...
                                             ctrl_short, config, paths)
%RUN_SINGLE_EXPERIMENT Execute Monte Carlo simulation for one trajectory

    stop_time = trajectories{traj_idx, 1};
    traj_name = trajectories{traj_idx, 2};
    traj_file = trajectories{traj_idx, 3};
    traj_desc = trajectories{traj_idx, 4};

    fprintf('\n');
    fprintf('┌───────────────────────────────────────────────────────────────┐\n');
    fprintf('│ Trajectory: %-49s │\n', traj_desc);
    fprintf('│ Controller: %-49s │\n', ctrl_short);
    fprintf('│ Stop time: %.2f s | Repetitions: %d                          │\n', ...
            stop_time, config.num_reps);
    fprintf('└───────────────────────────────────────────────────────────────┘\n');

    % Ensure model is closed before loading
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    % Load model
    load_system(model_name);
    fprintf('[INFO] Model loaded: %s.slx\n', model_name);

    % Update MATLAB Function block to load the selected trajectory
    set_trajectory_in_model(model_name, traj_file);

    J_tracking = [];
    J_energy   = [];

    for rep = 1:config.num_reps
        fprintf('  [%2d/%2d] Simulating...', rep, config.num_reps);

        initialize_noise_seeds(config.num_joints, config.noise_scale);

        simOut = sim(model_name, 'StopTime', num2str(stop_time));

        [J_traj_k, J_ener_k] = compute_metrics(simOut);
        J_tracking(rep, :) = J_traj_k;
        J_energy(rep, :)   = J_ener_k;

        fprintf(' Done\n');
    end

    % Close model to release compiled state
    close_system(model_name, 0);

    t_vec = simOut.xd(:, 1)';

    output_file = fullfile(paths.output, ...
        sprintf('%s_%s_results.mat', ctrl_short, traj_name));

    metadata = struct(...
        'controller',      ctrl_short, ...
        'trajectory',      traj_name, ...
        'trajectory_file', traj_file, ...
        'description',     traj_desc, ...
        'stop_time',       stop_time, ...
        'num_repetitions', config.num_reps, ...
        'created',         datestr(now, 'yyyy-mm-dd HH:MM:SS'));

    save(output_file, 'J_tracking', 'J_energy', 't_vec', 'metadata');

    fprintf('  >> Saved: %s\n', output_file);
end

function set_trajectory_in_model(model_name, traj_file)
%SET_TRAJECTORY_IN_MODEL Update MATLAB Function block to load given MAT-file
%   Edits the MATLAB Function block in the Simulink model so that the
%   load() call uses the specified trajectory MAT-file. Changing this line
%   forces Simulink to recompile the block and infer the size of q.q.
%
%   Inputs:
%       model_name - Simulink model name, e.g. 'Discrete_CTC'
%       traj_file  - MAT-file name, e.g. 'helice_yz.mat'

    % Relative path of the MATLAB Function block inside the model
    % Adjust if your block is in a different subsystem path.
    blockRelPath = 'References generator/MATLAB Function';
    blockPath    = [model_name '/' blockRelPath];

    % Use Stateflow API to access the MATLAB Function chart
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);

    if isempty(chart)
        error('Could not find MATLAB Function block at path: %s', blockPath);
    end

    % Get current script of the MATLAB Function block
    script = chart.Script;

    % Pattern to find the load() line:
    %   q = load('something.mat');
    pattern = 'q\s*=\s*load\(''.*''\);';

    % New load() line with desired trajectory file
    newLine = sprintf('q = load(''%s'');', traj_file);

    % Replace the pattern in the script
    newScript = regexprep(script, pattern, newLine);

    % Update the block code
    chart.Script = newScript;

    % Mark model as dirty so Simulink recompila en el próximo sim
    set_param(model_name, 'Dirty', 'on');

    fprintf('[INFO] MATLAB Function updated to load: %s\n', traj_file);
end
