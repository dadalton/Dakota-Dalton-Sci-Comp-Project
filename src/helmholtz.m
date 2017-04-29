%Dakota Dalton - 1366027
%Scientific Computing - MECE 5397
%Implementation of Helmholtz Equation in 2D - Semester Project
%Project code AHc2-1
clearvars; %clc;
%% Given values and Boundary Conditions

% start = 0.15;
% step = -0.005;
% stop = 0.01;
%for i = start:step:stop

ax = -pi; ay = ax; %given domain limits, these form a rectangle
bx = pi; by = bx;

%lambda = 1; %given value for lambda
lambda = 0;

delta = 0.025; %step size, same for both x and y

x = ax:delta:bx;  %discretizing the domain
y = ay:delta:by;

gb = (bx-x).^2 .* cos((pi*x)/bx); %boundary conditions for y
fb = x .* (bx - x).^2;
hb =  gb(1) + (y-ay)/(by-ay) * (fb(1)-gb(1)); %bc for x

%fbana = -0.33 * cos(x);

constcoeff = 1/(4 - ((delta^2) * lambda));

F = sin(pi * (x - ax)/(bx - ax))' ... %applied force
    * cos((pi/2)*(2*(y - ay)/(by - ay) + 1));
%F = zeros(length(y),length(x));

% a = 1;                            %analytical solution w/different conditions
% Fana = sin(a*y)' * cos(0.5*x);
% uana = Fana / (lambda - a - 0.25); 



%% Gauss-Seidel/Liebmann method

u = zeros(length(x)); %initial values of u to be iterated over
u(1,:) = gb;          %boundary condition for y (bottom)
u(end,:) = fb;        %boundary condition for y (top)
u(:,1) = hb; %bc for x (left)

% plot(u(1,:))
% plot(u(end,:))
% u(end,:) = fbana;        %boundary condition for y (top) (analytical)

iter = 0;                   %used to count number of iterations
epsilon = ones(length(x));  %calculating relative change per iteration

iterlabel = [];
% iterarray = [];
% epsarray = [];

% ucon = u;
% loop = 1;
% dim = size(u);

%while converge1 ~= converge2
%for i = start:step:stop        
    %converge2 = converge1;
    %delta = i
    
%     while iter < 5000 
    while epsilon > 0.01    
        
        uprev = u;              %iteration reference

        %sweeping through columns and rows, iterating values
        for i = 2:length(x)-1
            for j = 2:length(y)-1        
                
                u(j,i) = (u(j+1,i) + u(j-1,i) + u(j,i+1) + u(j,i-1) ...
                          - ((delta^2) * F(j,i))) * constcoeff;
                %disp(u(1,:))
        
            end

            %right side x bc is a Neumann condition (insulated)
            u(i,end) = (2 * u(i,end-1) + u(i+1,end) + u(i-1,end) ...
                        - (delta^2) * F(i,end)) * constcoeff; 

        end

        %the largest value change in the matrix will determine the epsilon
        epsilon = max(max(abs((u-uprev)./u)));  
        
        
        iter = iter + 1;        %counting the iterations
        
%         if mod(iter,1000) == 0
% %             surface(x,y,u)
% %             pause
%              iterlabel = [iterlabel, string(iter)];
%              hold on
%              plot(y, u(:,end))
%              title(legend(iterlabel), '# iterations')
%              xlabel('y')
%              xlim([-4,5.5])
%              ylabel('u(y)')
%              title('Neumann boundary condition du/dx(x=bx,y) = 0')
%              grid on
%         end
        
%         if mod(iter,100) == 0
%             disp(iter)
%             disp(epsilon)
%             iterarray = [iterarray, iter];
%             epsarray = [epsarray, epsilon];
%         end
       
    end
%     subplot(1,2,1)
  
%     subplot(1,2,2)
%     surface(uana)
%     disp(min(min(u)))
%     loop = loop + 1;
    
%     ucon = [ucon, u];
%     
%     rowsub  = 1:dim(1);
%     colsub1 = (loop - 1) * dim(2) + 1 : loop * dim(2);
%     colsub2 = (loop - 2) * dim(2) + 1 : (loop - 1) * dim(2);   
%         
%     if ucon(rowsub,colsub1) == ucon(rowsub, colsub2) ...
%        & isfinite(ucon(rowsub, colsub1)) 
%         disp(start-loop*abs(step))
%         break
%     end
    
%end
%% Gauss-Seidel with Successive Overrelaxtion (SOR)

SORlambda = 1.2;         %coefficient to expedite convergence

SORu = zeros(length(x)); %initial values of u to be iterated over
SORu(1,:) = gb;          %boundary condition for y (bottom)
SORu(end,:) = fb;        %boundary condition for y (top)
SORu(:,1) = hb; %bc for x (left)

SORiter = 0;                   %used to count number of iterations
SORepsilon = ones(length(x));  %calculating relative change per iteration

SORiterarray = [];
SORepsarray = [];



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
    
%     if mod(SORiter,100) == 0
%         disp(SORiter)
%         disp(SORepsilon)
%         SORiterarray = [SORiterarray, SORiter];
%         SORepsarray = [SORepsarray, SORepsilon];
%     end
     
end

    
%% Output & Visualization

% disp('Gauss-Seidel iterations:')
% disp(iter)
%disp('Gauss-Seidel iterations with SOR (' + string(SORlambda) + '):')
% disp(SORiter)
 
% subplot(1,2,1)
% surface(x,y,u)
% subplot(1,2,2)
% surface(x,y,SORu)

% contour(x,y,u)
% contour3(x,y,SORu)

% mesh(x,y,SORu)

% plot(x, gb)
% xlabel('x')
% ylabel('u(x)')
% title('Dirichlet boundary condition u(x,y=ay) = gb(x)')
% grid on

% plot(x, fb)
% xlabel('x')
% ylabel('u(x)')
% title('Dirichlet boundary condition u(x,y=ay) = fb(x)')
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

% mesh(x,y,u) 
% xlabel('x')
% ylabel('y')
% zlabel('u')
% grid on
% legend(string(iter) + ' iterations','location','best')
% view(-120,30)
% title('Poisson Equation, \lambda = 0, F = F(x,y), \Delta = 0.025')
     
% contour(x,y,u,15)
% xlabel('x')
% ylabel('y')
% grid on
% legend(string(iter) + ' iterations')
% colorbar
% title('Poisson Equation, \lambda = 0, F = F(x,y), \Delta = 0.025')

% mesh(x,y,SORu)
% xlabel('x')
% ylabel('y')
% zlabel('u')
% grid on
% legend(string(SORiter) + ' iterations','location','best')
% view(-120,37.5)
% title('Poisson Equation with SOR, SOR\lambda = 1.2, \lambda = 0, F = F(x,y), \Delta = 0.025')

% contour(x,y,SORu,15)
% xlabel('x')
% ylabel('y')
% grid on
% legend(string(SORiter) + ' iterations')
% colorbar
% title('Poisson Equation with SOR, SOR\lambda = 1.2, \lambda = 0, F = F(x,y), \Delta = 0.025')

disp('Max u:')
max(max(u))
disp('Mean u:')
mean(mean(u))
disp('Min u:')
min(min(u))

disp('Max SORu:')
max(max(SORu))
disp('Mean SORu:')
mean(mean(SORu))
disp('Min SORu:')
min(min(SORu))