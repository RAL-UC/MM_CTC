% Written by Dr. Seyedali Mirjalili
% To wach videos on this algorithm, enrol to the course with 95% discount using the following links: 

% ************************************************************************************************************************************************* 
%  A course on "Optimization Problems and Algorithms: how to understand, formulation, and solve optimization problems": 
%  https://www.udemy.com/optimisation/?couponCode=MATHWORKSREF
% ************************************************************************************************************************************************* 
%  "Introduction to Genetic Algorithms: Theory and Applications" 
%  https://www.udemy.com/geneticalgorithm/?couponCode=MATHWORKSREF
% ************************************************************************************************************************************************* 


function [ GBEST ,  cgCurve ] = PSO2 ( noP, maxIter,  problem, grapflag, dataVis )

% Define the details of the objective function
nVar = problem.nVar;
ub = problem.ub;
lb = problem.lb;
fobj = problem.fobj;

% Extra variables for data visualization
average_objective = zeros(1, maxIter);
cgCurve = zeros(1, maxIter);
FirstP_D1 = zeros(1 , maxIter);
position_history = zeros(noP , maxIter , nVar );

% Define the PSO's paramters
wMax = 0.9;
wMin = 0.2;
c1 = 2;
c2 = 2;
vMax = (ub - lb) .* 0.2;
vMin  = -vMax;

% The PSO algorithm

% Initialize the particles
for k = 1 : noP
    Swarm.Particles(k).X = (ub-lb) .* rand(1,nVar) + lb;
    Swarm.Particles(k).V = zeros(1, nVar);
    Swarm.Particles(k).PBEST.X = zeros(1,nVar);
    Swarm.Particles(k).PBEST.O = inf;
    
    Swarm.GBEST.X = zeros(1,nVar);
    Swarm.GBEST.O = inf;
end


% Main loop
for t = 1 : maxIter
    
    % Calcualte the objective value
    for k = 1 : noP
        
        currentX = Swarm.Particles(k).X;
        position_history(k , t , : ) = currentX;
        
        
        Swarm.Particles(k).O = fobj(currentX);
        average_objective(t) =  average_objective(t)  + Swarm.Particles(k).O;
        
        % Update the PBEST
        if Swarm.Particles(k).O < Swarm.Particles(k).PBEST.O
            Swarm.Particles(k).PBEST.X = currentX;
            Swarm.Particles(k).PBEST.O = Swarm.Particles(k).O;
        end
        
        % Update the GBEST
        if Swarm.Particles(k).O < Swarm.GBEST.O
            Swarm.GBEST.X = currentX;
            Swarm.GBEST.O = Swarm.Particles(k).O;
        end
        fprintf('Iteración=%d, contador=%d\n',t,k);
    end
    
    % Update the X and V vectors
    w = wMax - t .* ((wMax - wMin) / maxIter);
    
    FirstP_D1(t) = Swarm.Particles(1).X(1);
    
    for k = 1 : noP
        Swarm.Particles(k).V = w .* Swarm.Particles(k).V + c1 .* rand(1,nVar) .* (Swarm.Particles(k).PBEST.X - Swarm.Particles(k).X) ...
            + c2 .* rand(1,nVar) .* (Swarm.GBEST.X - Swarm.Particles(k).X);
        
        
        % Check velocities
        index1 = find(Swarm.Particles(k).V > vMax);
        index2 = find(Swarm.Particles(k).V < vMin);
        
        Swarm.Particles(k).V(index1) = vMax(index1);
        Swarm.Particles(k).V(index2) = vMin(index2);
        
        Swarm.Particles(k).X = Swarm.Particles(k).X + Swarm.Particles(k).V;
        
        % Check positions
        index1 = find(Swarm.Particles(k).X > ub);
        index2 = find(Swarm.Particles(k).X < lb);
        
        Swarm.Particles(k).X(index1) = ub(index1);
        Swarm.Particles(k).X(index2) = lb(index2);
        
    end
    
    if dataVis == 1
        disp('----------------------------------------------------------------------');
        fprintf('Iteración número %d, mejor J encontrado = %.15f\n',t,Swarm.GBEST.O);
        fprintf('con ganancias: ');
        disp(Swarm.GBEST.X);
        disp('----------------------------------------------------------------------');
        if mod(t,10)==0
            file_save=strcat('best_parameters',num2str(t),'.mat');
            save(file_save,'Swarm','Swarm_save');
            fprintf('Iteración %d grabada\n',t);
        end
    end
    
%     cgCurve(t) = Swarm.GBEST.O;
%     average_objective(t) = average_objective(t) / noP;
     Swarm_save{t}=Swarm;
%     
%     save graph.mat cgCurve average_objective
    %For save results
    %fileName = ['Resluts after iteration # ' , num2str(t)];
    %save( fileName)
end

GBEST = Swarm.GBEST;

if grapflag == 1
    iterations = 1: maxIter;
    
%% Draw the landscape 
    figure
    
    x = -50 : 1 : 50;
    y = -50 : 1 : 50;
    
    [x_new , y_new] = meshgrid(x,y);
    
    for k1 = 1: size(x_new, 1)
        for k2 = 1 : size(x_new , 2)
            X = [ x_new(k1,k2) , y_new(k1, k2) ];
            z(k1,k2) = ObjectiveFunction( X );
        end
    end
    
    subplot(1,5,1)
    surfc(x_new , y_new , z);
    title('Search landscape')
    xlabel('x_1')
    ylabel('x_2')
    zlabel('Objective value')
    shading interp
    camproj perspective
    box on
    set(gca,'FontName','Times')
    
    
%% Visualize the cgcurve    
    subplot(1,5,2);
    semilogy(iterations , cgCurve, 'r');
    title('Convergence curve')
    xlabel('Iteration#')
    ylabel('Weight')
    
%% Visualize the average objectives
    subplot(1,5,3)
    semilogy(iterations , average_objective , 'g')
    title('Average objecitves of all particles')
    xlabel('Iteration#')
    ylabel('Average objective')
    
 %% Visualize the fluctuations 
    subplot(1,5,4)   
    plot(iterations , FirstP_D1, 'k');
    title('First dimention in first Particle')
    xlabel('Iteration#')
    ylabel('Value of the first dimension')
    
 %% Visualize the search history
    subplot(1,5,5)
    hold on
    for p = 1 : noP
        for t = 1 : maxIter
            x = position_history(p, t , 1);
            y = position_history(p, t , 2);
            myColor = [0+t/maxIter  0 1-t/maxIter ];
            plot(x , y , '.' , 'color' , myColor );
        end
    end
    contour(x_new , y_new , z);
    plot(Swarm.GBEST.X(1) , Swarm.GBEST.X(2) , 'og');
    xlim([lb(1) , ub(1)])
    ylim([lb(2) , ub(2) ])
    title('search history')
    xlabel('x')
    ylabel('y')
    box on
    
    set(gcf , 'position' , [128         372        1634         259])
    
    
    
    
end