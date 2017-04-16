%testing use of pdetool to verify helmholtz
ax = -pi; ay = ax; %given domain limits, these form a rectangle
bx = pi; by = bx;

helmholtzmodel = createpde();
square = [3; 4; ax; bx; bx; ax; ay; ay; by; by];
domain = decsg(square);
% pdegplot(domain, 'EdgeLabels', 'on')
% xlim([-4,4])
% ylim([-4,4])

gb = @(region,state) (bx - region.x).^2 .* cos((pi * region.x)/bx); %boundary condition for y
fb = @(region,state) region.x .* (bx - region.x).^2;
xax = @(region,state) gb(ax) + (region.y-ay)/(by-ay) * (fb(ax)-gb(ax));
F = @(region,state) sin(pi * (region.x - ax)/(bx - ax)) ... %applied force
    .* cos((pi/2)*(2*(region.y - ay)/(by - ay) + 1));

geometryFromEdges(helmholtzmodel, domain);

applyBoundaryCondition(helmholtzmodel, 'dirichlet', 'Edge', 1, 'u', gb, 'Vectorized', 'on');
applyBoundaryCondition(helmholtzmodel, 'neumann', 'Edge', 2);
applyBoundaryCondition(helmholtzmodel, 'dirichlet', 'Edge', 3, 'u', fb, 'Vectorized', 'on');
applyBoundaryCondition(helmholtzmodel, 'dirichlet', 'Edge', 4, 'u', xax, 'Vectorized', 'on');

specifyCoefficients(helmholtzmodel, 'm', 0,...
                                    'd', 0,...
                                    'c', -1,...
                                    'a', 1,...
                                    'f', F);  

generateMesh(helmholtzmodel);

result = solvepde(helmholtzmodel);

%pdeplot(helmholtzmodel)