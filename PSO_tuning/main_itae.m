clear all, close all, warning off, clc

% Problem preparation
problem.nVar = 10;

%Gain ranges [Kp1 Kp2 ... Kp5 Kv1 Kv2 ... Kv5]
problem.lb = [0,0,0,0,0,0,0,0,0,0];
problem.ub = [1212,18560.3,958.22615,988.6,972.3,84.32,2651.5,151,142.44,149.1];

problem.fobj = @ObjectiveFunction_error_pos;

% PSO parameters
noP = 100; 
maxIter = 100;
grapflag = 0;   %set visualization for graphics
visFlag = 1; % set this to 0 if you do not want visualization

RunNo  = 30;
BestSolutions_PSO = zeros(1 , RunNo);


[ GBEST  , cgcurve ] = PSO2( noP , maxIter, problem , grapflag, visFlag ) ;

disp('Best gains combination found:')
GBEST.X
disp('Best result found:')
GBEST.O
