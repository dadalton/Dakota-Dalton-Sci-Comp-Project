%Dakota Dalton - 1366027
%Scientific Computing - MECE 5397
%Implementation of Helmholtz Equation in 2D - Semester Project
%Project code AHc2-1
clearvars; clc;
%% Given values and Boundary Conditions
ax = -pi; ay = ax; %given domain limits, these form a rectangle
bx = pi; by = bx;

lambda = 1; %given value for lambda
%lambda = 0;

delta = 0.1; %step size, same for both x and y

x = ax:delta:bx;  %discretizing the domain
y = ay:delta:by;

gb = (bx-x).^2 .* cos((pi*x)/bx); %boundary conditions for y
fb = x .* (bx - x).^2;

F = sin(pi * (x - ax)/(bx - ax)) ... %applied force
    .* cos((pi/2)*(2*(y - ay)/(by - ay) + 1));
%F = 0;

%% Gauss-Seidel/Liebmann method

u = zeros(length(x)); %initial values of u to be iterated over
u(1,:) = gb;          %boundary condition for y (bottom)
u(end,:) = fb;        %boundary condition for y (top)
u(:,1) = gb(1) + (y-ay)/(by-ay) * (fb(1)-gb(1)); %bc for x (left)

iter = 0;                   %used to count number of iterations
epsilon = ones(length(x));  %calculating relative change per iteration

while epsilon > 0.01
    uprev = u;              %iteration reference
    
    %sweeping through columns and rows, iterating values
    for i = 2:length(y)-1
        for j = 2:length(x)-1
            u(j,i) = (u(j+1,i) + u(j-1,i) + u(j,i+1) + u(j,i-1) ...
                - (delta^2) * F(j)) * (1/(4 - delta^2 * lambda));
        end
        
        %right side x bc is a Neumann condition (insulated)
        u(i,end) = (2*u(i,end-1) + u(i+1,end) + u(i-1,end) - (delta^2)*F(i)) ...
            * (1/(4 - delta^2 * lambda)); 
    end
    %the largest value change in the matrix will determine the epsilon
    epsilon = abs(max(max((u-uprev)./u)));  
    iter = iter + 1;                        %counting the iterations
end

%% Gauss-Seidel with Successive Overrelaxtion (SOR)

SORlambda = 1.2;         %coefficient to expedite convergence

SORu = zeros(length(x)); %initial values of u to be iterated over
SORu(1,:) = gb;          %boundary condition for y (bottom)
SORu(end,:) = fb;        %boundary condition for y (top)
SORu(:,1) = gb(1) + (y-ay)/(by-ay) * (fb(1)-gb(1)); %bc for x (left)

SORiter = 0;                   %used to count number of iterations
SORepsilon = ones(length(x));  %calculating relative change per iteration

while SORepsilon > 0.01
    SORuprev = SORu;              %iteration reference
    
    %sweeping through columns and rows, iterating values
    for i = 2:length(y)-1
        for j = 2:length(x)-1
            SORu(j,i) = (SORu(j+1,i) + SORu(j-1,i) + SORu(j,i+1) + SORu(j,i-1) ...
                - (delta^2) * F(j)) * (1/(4 - delta^2 * lambda));
            SORu(j,i) = SORlambda * SORu(j,i) + (1-SORlambda) * SORuprev(j,i);
        end
        
        %right side x bc is a Neumann condition (insulated)
        SORu(i,end) = (2*SORu(i,end-1) + SORu(i+1,end) + SORu(i-1,end) - (delta^2)*F(i)) ...
            * (1/(4 - delta^2 * lambda)); 
        SORu(i,end) = SORlambda * SORu(i,end) + (1-SORlambda) * SORuprev(i,end);
        
    end
    %the largest value change in the matrix will determine the epsilon
    SORepsilon = abs(max(max((SORu-SORuprev)./SORu)));  
    SORiter = SORiter + 1;                        %counting the iterations
end
%% Output & Visualization

disp('Gauss-Seidel iterations:')
disp(iter)
disp('Gauss-Seidel iterations with SOR (' + string(SORlambda) + '):')
disp(SORiter)
 
% subplot(1,2,1)
% surface(x,y,u)
% subplot(1,2,2)
% surface(x,y,SORu)

% contour(x,y,u)
% contour3(x,y,SORu)

% mesh(x,y,SORu)