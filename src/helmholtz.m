%Dakota Dalton - 1366027
%Scientific Computing - MECE 5397
%Implementation of Helmholtz Equation in 2D - Semester Project
%Project code AHc2-1

%% given values and boundary conditions
ax = -pi; ay = ax; %given domain limits, these form a rectangle
bx = pi; by = bx;

lambda = 1; %given value for lambda

delta = 0.1; %step size, same for both x and y

x = ax:delta:bx;  %discretizing the domain
y = ay:delta:by;

gb = (bx-x).^2 .* cos((pi*x)/bx); %boundary conditions for y
fb = x .* (bx - x).^2;

F = sin(pi * (x - ax)/(bx - ax)) ... %applied force
    .* cos((pi/2)*(2*(y - ay)/(by - ay) + 1));

%% gauss-seidel method

u = zeros(length(x)); %initial values of u to be iterated over
u(1,:) = gb;
u(end,:) = fb;
u(:,1) = gb(1) + (y-ay)/(by-ay) * (fb(1)-gb(1));

iter = 0;
epsilon = ones(length(x));
while epsilon > 0.1
    uprev = u;
    for i = 2:length(y)-1
        for j = 2:length(x)-1
            u(j,i) = (u(j+1,i) + u(j-1,i) + u(j,i+1) + u(j,i-1) ...
                - (delta^2) * F(j)) * (1/(4 - delta^2 * lambda));
        end
        u(i,end) = (2*u(i,end-1) + u(i+1,end) + u(i-1,end) - (delta^2)*F(i)) ...
            * (1/(4 - delta^2 * lambda));
    end
    epsilon = abs(max(max((u-uprev)./u)));
    iter = iter + 1;
end





