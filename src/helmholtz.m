%Dakota Dalton - 1366027
%Scientific Computing - MECE 5397
%Implementation of Helmholtz Equation in 2D - Semester Project
%Project code AHc2-1

clearvars; clc;

%% Given values and Boundary Conditions

%The domain
    ax = -pi; ay = ax; %given domain limits, these form a rectangle
    bx =  pi; by = bx;


%Lambda can be adjusted to create the Laplace/Poisson equations
    lambda = 1; %given value for lambda
   %lambda = 0;

%spatial element size, can be adjusted to test effects of grid size
    delta = 0.1; %step size, same for both x and y

    x = ax:delta:bx;  %discretizing the domain
    y = ay:delta:by;

%Dirichlet Boundary Conditions
    gb = (bx-x).^2 .* cos((pi*x)/bx); %boundary conditions for y 
    fb = x .* (bx - x).^2;
    hb = gb(1) + (y-ay)/(by-ay) * (fb(1)-gb(1)); %bc for x

%the denominator term of the discretization, this is calculated here to
%optimize the core loop speed
    constcoeff = 1/(4 - ((delta^2) * lambda));

%the given forcing function, can be set to zero for homogeneous case
    F = sin(pi * (x - ax)/(bx - ax))' ... %applied force
        * cos((pi/2)*(2*(y - ay)/(by - ay) + 1));
    %F = zeros(length(y),length(x));


%% Gauss-Seidel/Liebmann method

u = zeros(length(x)); %initial values of u to be iterated over
u(1,:) = gb;          %boundary condition for y (bottom)
u(end,:) = fb;        %boundary condition for y (top)
u(:,1) = hb;          %bc for x (left)

iter = 0;                   %used to count number of iterations
epsilon = ones(length(x));  %calculating relative change per iteration

%the error per iteration is stored to plot convergence
% iterarray = [];
% epsarray = [];

%     while iter < 5000     %break condition can be set to # iterations
    while epsilon > 0.01    %loop continues until error <= 1%
        
        uprev = u;              %iteration reference

        %the loop runs through columns before switching rows in order to
        %exploit Matlab's column-major orientation for few cache misses
        
        %sweeping through columns and rows, iterating values
        for i = 2:length(x)-1
            for j = 2:length(y)-1        
                   
                    u(j,i) = (u(j+1,i) + u(j-1,i) + u(j,i+1) + u(j,i-1) ...
                          - ((delta^2) * F(j,i))) * constcoeff;
        
            end

            %right side x bc is a Neumann condition (insulated)
            u(i,end) = (2 * u(i,end-1) + u(i+1,end) + u(i-1,end) ...
                        - (delta^2) * F(i,end)) * constcoeff; 

        end

        %the largest value change in the matrix will determine the epsilon
        epsilon = max(max(abs((u-uprev)./u)));  
        
        
        iter = iter + 1;   %counting the iterations
        
        
%this stores the error value for each iteration, sample rate can be changed       
%         if mod(iter,1) == 0
%             iterarray = [iterarray, iter];
%             epsarray = [epsarray, epsilon];
%         end
       
    end    
    
%% Gauss-Seidel with Successive Overrelaxtion (SOR)

%the weighting parameter, vary between 1 and 2 for overrelaxation
SORlambda = 1.2;         %coefficient to expedite convergence

SORu = zeros(length(x)); %initial values of u to be iterated over
SORu(1,:) = gb;          %boundary condition for y (bottom)
SORu(end,:) = fb;        %boundary condition for y (top)
SORu(:,1) = hb;          %bc for x (left)

SORiter = 0;                   %used to count number of iterations
SORepsilon = ones(length(x));  %calculating relative change per iteration

% SORiterarray = [];
% SORepsarray = [];

while SORepsilon > 0.01
    SORuprev = SORu;              %iteration reference
    
    %sweeping through columns and rows, iterating values
    for i = 2:length(y)-1
        for j = 2:length(x)-1
            SORu(j,i) = (SORu(j+1,i) + SORu(j-1,i) + SORu(j,i+1) + SORu(j,i-1) ...
                - (delta^2) * F(j)) * constcoeff;
            SORu(j,i) = SORlambda * SORu(j,i) + (1-SORlambda) * SORuprev(j,i);
        end
        
        %right side x bc is a Neumann condition (insulated)
        SORu(i,end) = (2*SORu(i,end-1) + SORu(i+1,end) + SORu(i-1,end) - (delta^2)*F(i)) ...
            * constcoeff; 
        SORu(i,end) = SORlambda * SORu(i,end) + (1-SORlambda) * SORuprev(i,end);
        
    end
    %the largest value change in the matrix will determine the epsilon
    SORepsilon = abs(max(max((SORu-SORuprev)./SORu)));  
    
    SORiter = SORiter + 1;                        %counting the iterations
    
%     if mod(SORiter,1) == 0
%         SORiterarray = [SORiterarray, SORiter];
%         SORepsarray = [SORepsarray, SORepsilon];
%     end
      
end

%% Output & Visualization

%Comparing iteration count for a given test
    % disp('Gauss-Seidel iterations:')
    % disp(iter)
    % disp('Gauss-Seidel iterations with SOR (' + string(SORlambda) + '):')
    % disp(SORiter)

%Plotting the boundary conditions
    % plot(x, gb)
    % xlabel('x')
    % ylabel('u(x)')
    % title('Dirichlet boundary condition u(x,y=ay) = gb(x)')
    % grid on

    % plot(x, fb)
    % xlabel('x')
    % ylabel('u(x)')
    % title('Dirichlet boundary condition u(x,y=by) = fb(x)')
    % grid on

    % plot(y, hb)
    % xlabel('y')
    % ylabel('u(y)')
    % title('Dirichlet boundary condition u(x=ax,y) = hb(y)')
    % grid on

    % plot(y, u(:,end))
    % xlabel('y')
    % ylabel('u(y)')
    % title('Neumann boundary condition du/dx(x=bx,y) = 0')
    % grid on

%Generating meshes and contours for the given test    
    % figure
    % mesh(x,y,u) 
    % xlabel('x')
    % ylabel('y')
    % zlabel('u')
    % grid on
    % legend(string(iter) + ' iterations','location','best')
    % view(-120,30)
    % title('Helmholtz Equation, \lambda = ' + string(lambda) +', F = F(x,y), \Delta = ' + string(delta))
    % 
    % figure
    % contour(x,y,u,15)
    % xlabel('x')
    % ylabel('y')
    % grid on
    % legend(string(iter) + ' iterations')
    % colorbar
    % title('Helmholtz Equation, \lambda = ' + string(lambda) + ', F = F(x,y), \Delta = ' + string(delta))
    % 
    % figure
    % mesh(x,y,SORu)
    % xlabel('x')
    % ylabel('y')
    % zlabel('u')
    % grid on
    % legend(string(SORiter) + ' iterations','location','best')
    % view(-120,30)
    % title('Helmholtz Equation with SOR, SOR\lambda = ' + string(SORlambda) + ', \lambda = ' + string(lambda) +', F = F(x,y), \Delta = ' + string(delta))
    %  
    % figure
    % contour(x,y,SORu,15)
    % xlabel('x')
    % ylabel('y')
    % grid on
    % legend(string(SORiter) + ' iterations')
    % colorbar
    % title('Helmholtz Equation with SOR, ' + string(SORlambda) + ', \lambda = ' + string(lambda) +', F = F(x,y), \Delta = ' + string(delta))

%Showing different numerical results for the test    
    % disp('Max u:')
    % max(max(u))
    % disp('Mean u:')
    % mean(mean(u))
    % disp('Min u:')
    % min(min(u))
    % 
    % disp('Max SORu:')
    % max(max(SORu))
    % disp('Mean SORu:')
    % mean(mean(SORu))
    % disp('Min SORu:')
    % min(min(SORu))

%Plotting the forcing function, F(x,y)    
    % figure
    % mesh(x,y,F) 
    % xlabel('x')
    % ylabel('y')
    % zlabel('F')
    % grid on
    % legend(string(iter) + ' iterations','location','best')
    % view(45,30)
    % title('F(x,y)')

%Plotting the error over the course of the iterations    
    % plot(iterarray, epsarray)
    % xlabel('# Iterations')
    % ylabel('Error (Infinity norm)')
    % title('Error over iterations')
    % grid on
    %  
    % hold on
    %  
    % plot(SORiterarray, SORepsarray)
    % xlabel('# Iterations')
    % ylabel('Error (Infinity norm)')
    % title('Error over iterations, \Delta = ' + string(delta))
    % legend('Gauss-Seidel,  ' + string(iter) + ' iterations', 'SOR \lambda = 1.2,   ' + string(SORiter) + ' iterations')
    % ylim([0 100])
    % grid on