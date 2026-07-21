function [xresult, xinit, uresult] = finite_horizon_solver(x0, goal, radii, centres) 
% T   = 10;                    % Horizon length
% N   = 64;                    % Number of discretisation intervals 
T   = 1;                    % Horizon length
N   = 32;                    % Number of discretisation intervals 
Nx  = 5;                     % Number of state variables 
Nu  = 2;                     % Number of input variables 

num     = length(radii);     % Number of obstacle
wp      = 0.1;
wh      = 1;

% prob_info is a structure containing problem related information, which
% will be passed to sub-functions whenever it's necessary 
prob_info = struct( 'T', T, 'N', N, 'Nu', Nu, 'Nx', Nx, 'x0', x0,...
                    'goal', goal, 'centres', centres, 'radii', radii,...
                    'num', num, 'wp', wp, 'wh', wh);

% ===================================================================
% preparing initial optimisation variables for the solver 
init_control = 0.01 + 0.02*rand(Nu, N);
init_state = zeros(Nx, N);

delta = T/N; % find the delta T (step)
init_state(:,1) = x0 + delta * sys_h(x0, init_control(:,1), prob_info); % initial first column

% subsequent columns
for idx = 2:N
    init_state(:,idx) = init_state(:,idx-1) + delta * sys_h(init_state(:,idx-1), init_control(:,idx), prob_info);
end 

% ===================================================================
% upper and lower bounds of decision variables
% initialisation 
lb_control = -inf(Nu, N);
ub_control =  inf(Nu, N);

lb_state = -inf(Nx, N);
ub_state =  inf(Nx, N);

% add constrains
% ub_control(1,: ) = 0.5;     % steering angle upper bound, rad
% lb_control(1,: ) = -0.5;    % steering angle lower bound, rad
ub_control(1,: ) = 1.6;     % steering angle upper bound, rad
lb_control(1,: ) = -1.6;    % steering angle lower bound, rad

ub_control(2,: ) = 2;       % acceleration upper bound, m/s^2 
lb_control(2,: ) = -3;      % acceleration lower bound, m/s^2 

% conbine lower bound and upper bound
lb = [lb_state(:); lb_control(:)];
ub = [ub_state(:); ub_control(:)];

% ===================================================================
% linear inequality and equality constraints, we don't have any of these in
% our problem, so we define them as empty variables. 
A   = [];
b   = [];
Aeq = [];
beq = [];

% eta is the vector that includes all the optimisation variables, x(1),...,
% x(N) and u(0),..., u(N-1)
eta = [init_state(:) ; init_control(:)];

% function handles for cost and constraints 
fun     = @(eta) cost(eta, prob_info); % wrapped as a simple function, eta as a parameter
nonlcon = @(eta) nconst(eta, prob_info); % wrapped as a simple function, eta as a parameter

options = optimoptions( 'fmincon','Display','iter',...
                        'MaxIterations', 1e2, 'MaxFunctionEvaluations', 1e6);

% Solve the constrained nonlinear optimisation problem using fmincon
[eta_star, ~, ~, ~] = fmincon(  fun, eta, A, b, Aeq, beq,...
                                lb, ub, nonlcon, options);

xresult = [x0 reshape(eta_star(1:N*Nx,1), Nx, N)];
xinit   = [x0 reshape(eta(1:N*Nx,1), Nx, N)];

uresult = reshape(eta_star((N*Nx+1):(N*Nx+N*Nu)), Nu, N);