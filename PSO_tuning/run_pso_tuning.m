function run_pso_tuning()
%RUN_PSO_TUNING PSO-based gain tuning for discrete-time controllers
%   This script runs the PSO2 algorithm to tune the proportional and
%   derivative gains Kp1..Kp5 and Kv1..Kv5 of the discrete-time
%   controllers (CTC and PD) for the skid-steer mobile manipulator.
%
%   The objective function:
%       ObjectiveFunction_error_pos(x, ctrl_id)
%   evaluates a scalar cost J that combines RMS end-effector tracking
%   error and RMS control power, where ctrl_id selects between:
%       'CTC' -> Discrete_CTC.slx
%       'PD'  -> Discrete_PD.slx.
%
%   For each controller, the optimized gains are stored in a Swarm_save
%   cell array whose final entry Swarm_save{1,end}.GBEST.X contains:
%       [Kp1 Kp2 Kp3 Kp4 Kp5 Kv1 Kv2 Kv3 Kv4 Kv5].
%   This format is compatible with downstream scripts such as
%   run_trajectory_experiments.m.
%
%   Output files (per controller):
%       - If the expected repository structure is detected:
%           <repo_root>/PSO_Results/ctc.mat or pd.mat
%           <repo_root>/Controller_Implementation/Simulation/ctc.mat or pd.mat
%       - Otherwise (no repository structure detected):
%           ./ctc.mat or ./pd.mat  (in the current working directory)
%
%   Requirements:
%       - PSO2.m with signature:
%             [GBEST, cgCurve, Swarm_save] = PSO2(...)
%       - ObjectiveFunction_error_pos.m in the MATLAB path.
%       - Simulink models:
%             Discrete_CTC.slx
%             Discrete_PD.slx
%
%   Usage:
%       1. Ensure PSO2.m and ObjectiveFunction_error_pos.m are on the path.
%       2. Navigate to this folder in MATLAB.
%       3. Run:
%              >> run_pso_tuning

%% ======================= PATH CONFIGURATION =======================
[script_path, ~, ~] = fileparts(mfilename('fullpath'));
if isempty(script_path)
    script_path = pwd;
end

% Try to infer the repository root assuming the structure:
%   <repo_root>/
%       Controller_Implementation/
%           PSO_Tuning/          <- this script
%           Simulation/          <- run_trajectory_experiments.m
repo_root_candidate   = fullfile(script_path, '..', '..');
controller_impl_dir   = fullfile(repo_root_candidate, 'Controller_Implementation');
simulation_dir        = fullfile(controller_impl_dir, 'Simulation');
pso_results_dir       = fullfile(repo_root_candidate, 'PSO_Results');

using_repo_structure = false;

if exist(controller_impl_dir, 'dir') && exist(simulation_dir, 'dir')
    using_repo_structure = true;
    repo_root        = repo_root_candidate;
    simulation_path  = simulation_dir;
    pso_results_path = pso_results_dir;
    if ~exist(pso_results_path, 'dir')
        mkdir(pso_results_path);
    end
else
    % Fallback: no repository structure detected.
    % Use the current working directory for all outputs.
    repo_root        = pwd;
    simulation_path  = '';      % unknown / not used in this case
    pso_results_path = pwd;
end

%% ======================= PROBLEM DEFINITION =======================
% Shared problem definition for both controllers (10 gains: 5 Kp + 5 Kv).
problem_template.nVar = 10;

% Gain ranges [Kp1 Kp2 ... Kp5 Kv1 Kv2 ... Kv5]
problem_template.lb = [ ...
    0,    0,       0,       0,      0, ...
    0,    0,       0,       0,      0  ];
problem_template.ub = [ ...
    1212, 18560.3, 958.22615, 988.6, 972.3, ...
    84.32, 2651.5, 151,      142.44,149.1 ];

%% ======================= PSO PARAMETERS ===========================
noP      = 100;   % Number of particles
maxIter  = 100;   % Maximum iterations
grapflag = 0;     % Disable graphical visualization in PSO2
visFlag  = 1;     % Enable textual progress reporting in PSO2

% Random seed for reproducibility (change if needed)
rng(1);

%% ======================= CONTROLLER SETUP =========================
% { id,   short, gains_file,   description }
CONTROLLERS = {
    'CTC', 'CTC', 'ctc.mat', 'Discrete-time computed torque controller';
    'PD',  'PD',  'pd.mat',  'Discrete-time PD controller'
};

%% ======================= INITIALIZATION ===========================
clc; close all; warning('off', 'all');
print_header();

if using_repo_structure
    fprintf('[INFO] Repository structure detected.\n');
    fprintf('       Repository root: %s\n', repo_root);
    fprintf('       PSO results dir: %s\n', pso_results_path);
    fprintf('       Simulation dir:  %s\n\n', simulation_path);
else
    fprintf('[WARNING] Repository structure not detected.\n');
    fprintf('          PSO results will be saved to the current directory:\n');
    fprintf('          %s\n\n', pso_results_path);
end

selection = select_controller_to_tune(CONTROLLERS);
if selection == 0
    fprintf('\n[INFO] Operation cancelled by user.\n');
    return;
end

%% ======================= RUN PSO PER CONTROLLER ===================
if selection == 3
    idx_list = 1:size(CONTROLLERS, 1);   % BOTH
else
    idx_list = selection;                % Single controller
end

for idx = idx_list
    tune_single_controller( ...
        idx, CONTROLLERS, ...
        problem_template, ...
        noP, maxIter, grapflag, visFlag, ...
        using_repo_structure, pso_results_path, simulation_path);
end

fprintf('\n[INFO] PSO tuning finished successfully.\n');
fprintf('[INFO] You can now run run_trajectory_experiments.m to evaluate the tuned controllers.\n');

end

%% ======================= LOCAL FUNCTIONS ==========================

function print_header()
    fprintf('\n');
    fprintf('╔═══════════════════════════════════════════════════════════════════╗\n');
    fprintf('║     PSO-BASED GAIN TUNING FOR DISCRETE-TIME CONTROLLERS (MM)     ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════════════╝\n\n');
end

function selection = select_controller_to_tune(controllers)
    fprintf('╔═══════════════════════════════════════════════════════════╗\n');
    fprintf('║              STEP 1: SELECT CONTROLLER(S) TO TUNE         ║\n');
    fprintf('╠═══════════════════════════════════════════════════════════╣\n');
    for i = 1:size(controllers, 1)
        fprintf('║  [%d] %-50s   ║\n', i, controllers{i, 4});
    end
    fprintf('╠═══════════════════════════════════════════════════════════╣\n');
    fprintf('║  [3] Tune BOTH (CTC and PD)                               ║\n');
    fprintf('║  [0] Cancel                                               ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════╝\n\n');
    selection = get_valid_input('Select option', 0, 3);
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

function tune_single_controller(idx, controllers, problem_template, ...
                                noP, maxIter, grapflag, visFlag, ...
                                using_repo_structure, pso_results_path, simulation_path)

    ctrl_id     = controllers{idx, 1};      % 'CTC' or 'PD'
    ctrl_short  = controllers{idx, 2};
    gains_file  = controllers{idx, 3};      % 'ctc.mat' or 'pd.mat'
    ctrl_desc   = controllers{idx, 4};

    fprintf('\n');
    fprintf('┌───────────────────────────────────────────────────────────────┐\n');
    fprintf('│ Controller: %-49s │\n', ctrl_desc);
    fprintf('│ ID:         %-49s │\n', ctrl_id);
    fprintf('│ Particles:  %3d | Iterations: %3d                        │\n', ...
            noP, maxIter);
    fprintf('└───────────────────────────────────────────────────────────────┘\n');

    % Build problem structure for this controller
    problem = problem_template;
    % Pass controller ID as second argument via anonymous function
    problem.fobj = @(x) ObjectiveFunction_error_pos(x, ctrl_id);

    tic;
    [GBEST, cgCurve, Swarm_save] = PSO2(noP, maxIter, problem, grapflag, visFlag);
    elapsed = toc;

    fprintf('\n=====================================================\n');
    fprintf('Best gain combination found for %s (GBEST.X):\n', ctrl_id);
    disp(GBEST.X);
    fprintf('Best objective value (GBEST.O): %.15f\n', GBEST.O);
    fprintf('PSO total elapsed time: %.2f minutes\n', elapsed/60);
    fprintf('=====================================================\n');

    % Basic sanity check on Swarm_save format
    if isempty(Swarm_save) || ~iscell(Swarm_save) || ...
       isempty(Swarm_save{1,end}) || ~isfield(Swarm_save{1,end}, 'GBEST')
        warning(['[WARNING] Swarm_save does not match the expected format. ', ...
                 'Ensure Swarm_save{1,end}.GBEST.X exists.']);
    else
        gbest_vec = Swarm_save{1,end}.GBEST.X;
        fprintf('[INFO] Length of GBEST.X in Swarm_save{1,end}: %d elements\n', ...
                numel(gbest_vec));
    end

    % Metadata for this controller
    metadata = struct(...
        'controller_id',   ctrl_id, ...
        'controller_name', ctrl_desc, ...
        'short_name',      ctrl_short, ...
        'num_vars',        problem.nVar, ...
        'num_particles',   noP, ...
        'num_iterations',  maxIter, ...
        'using_repo_structure', using_repo_structure, ...
        'created',         datestr(now, 'yyyy-mm-dd HH:MM:SS'));

    % Primary output path for gains file (ctc.mat or pd.mat)
    gains_file_primary = fullfile(pso_results_path, gains_file);
    save(gains_file_primary, 'Swarm_save', 'GBEST', 'cgCurve', 'metadata', 'problem');

    fprintf('[INFO] PSO results saved to: %s\n', gains_file_primary);

    % If the repository structure is available, also place the gains file
    % in the Simulation folder so that run_trajectory_experiments.m can
    % load it transparently. [file:21]
    if using_repo_structure && exist(simulation_path, 'dir')
        gains_file_sim = fullfile(simulation_path, gains_file);
        try
            copyfile(gains_file_primary, gains_file_sim);
            fprintf('[INFO] %s copied to Simulation folder:\n', gains_file);
            fprintf('       %s\n', gains_file_sim);
        catch ME
            warning('[WARNING] Could not copy %s to Simulation folder: %s', ...
                    gains_file, ME.message);
        end
    else
        % When no repository structure is detected, ensure the gains file
        % is available in the current working directory as well.
        try
            if ~strcmp(pso_results_path, pwd)
                copyfile(gains_file_primary, fullfile(pwd, gains_file));
                fprintf('[INFO] %s also copied to current directory:\n', gains_file);
                fprintf('       %s\n', fullfile(pwd, gains_file));
            else
                fprintf('[INFO] %s is available in the current directory.\n', gains_file);
            end
        catch ME
            warning('[WARNING] Could not ensure %s in current directory: %s', ...
                    gains_file, ME.message);
        end
    end
end
