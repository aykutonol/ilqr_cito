%% Simulation
tf = 1.00;                  % [s] simulation final time
dt = 2e-3;                  % [s] dynamic sampling period
tc = 1e-1;                  % [s] control sampling period
%% Numerical differentiation
cdiff = 1;                  % 0/1: forward/central differences
h = 1e-6;                   % [-] perturbation
%% Number of variables
ncts = round(tf/tc);        % [-] number of control time steps
ndts = round(tf/tc)+1;      % [-] number of dynamic time steps
ndof = 4;                   % [-] degrees of freedom
ncc  = 4;                   % [-] number of contact candidates
%% Robot
tau_lim = 1;                % [N-m] torque limit
%% Object
mo = 1;                     % [kg] mass
eo = .2;                    % [m] edge length (cube)
% Jo = 1/6*mo*eo^2;           % [kg-m2] inertia matrix
Jo = .1;
% Dispacements of vertices and centers from the center of mass
vd(:,1) = [-1;-1]; vd(:,2) = [1;-1]; vd(:,3) = [1;1]; vd(:,4) = [-1;1];
ec(:,1) = [0;-1];  ec(:,2) = [1;0];  ec(:,3) = [0;1]; ec(:,4) = [-1;0];
%% Contact model
% kcon0 = 1e1;
% acon  = -log(5e0/kcon0)/eo;

kcon0 = 5;
acon  = 15;