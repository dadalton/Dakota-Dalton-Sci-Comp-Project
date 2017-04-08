%Dakota Dalton - 1366027
%Scientific Computing - MECE 5397
%Implementation of Helmholtz Equation in 2D - Semester Project
%Project code AHc2-1

%% given values and boundary conditions
ax = -pi; ay = ax; %given domain limits, these form a rectangle
bx = pi; by = bx;

delta = 0.1; %step size, same for both x and y

x = ax:delta:bx;  %discretizing the domain
y = ay:delta:ax;

gb = (bx-x).^2 .* cos((pi*x)/bx); %boundary conditions for y
fb = x .* (bx - x).^2;

F = sin(pi * (x - ax)/(bx - ax)) ... %applied force
    * cos((pi/2)*(2*(y - ay)/(by - ay) + 1));

%% gauss-seidel method

u = zeros(length(x)); %initial values of u to be iterated over

for j = 1:length(y)
    for i = 1:length(x)
        uprev = u(i,j);
        u(i,j) = (u(i+1,j) + u(i-1,j) + u(i,j+1) + u(i,j-1) ...
            - delta^2 * F(i,j)) * (1/(4 - delta^2 * lambda));
        epsilon = abs((u(i,j)-uprev)/u(i,j));
        if epsilon < 0.01   %terminates iteration for less than 1% change
            break
        end
    end
end

%u = u * 1/(4 - delta^2 * lambda); %applying constant denominator coefficient






