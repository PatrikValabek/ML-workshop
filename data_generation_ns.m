clear; close all; rng(42);

%% Model
A = [0 1; 0 0];
B = [0; 1];
C = eye(2);
D = 0;

sysc = ss(A,B,C,D);
Ts = 1;
sysd = c2d(sysc,Ts);

A = sysd.A;             
B = sysd.B;
C = sysd.C;
D = sysd.D;

Tmax = 30;
kf = Tmax/Ts;


[nx,nu] = size(B);
[ny,~] = size(C);

xmin = [-10;-3];
xmax = [10;3];
umin = -2;
umax = 2;


% define MPC 
Qx = eye(nx);
Qu = eye(nu);
N  = 5;


%% Setting up MPC 
yalmip clear

uu = sdpvar(nu,N);
xx = sdpvar(nx,N+1);

obj = 0;
cst = [];

for k = 1:N
   % constraints
   cst = cst + [xx(:,k+1) == A*xx(:,k) + B*uu(:,k)];
   cst = cst + [xmin <= xx(:,k) <= xmax];
   cst = cst + [umin <= uu(:,k) <= umax];
   % objective
   obj = obj + xx(:,k)'*Qx*xx(:,k) + uu(:,k)'*Qu*uu(:,k);
end

x = zeros(nx,kf+1);
u = zeros(nu,kf);
x0 = [-7; -2];
x(:,1) = x0;


options = sdpsettings('verbose',0,'solver','gurobi');
controller = optimizer(cst, obj, options, xx(:,1), uu(:,1));



%% Closed Loop Simulation

t_start = cputime;
for k = 1:kf
    u(:,k) = controller(x(:,k));
    x(:, k+1) = A*x(:,k) + B*u(:,k);
end
t_end = cputime;

fprintf(['\nSimulation time: ', num2str(t_end-t_start), 's\n'])

x(:,end) = [];
t = linspace(0,Tmax,kf);


time = t;
figure, grid on
for k = 1:ny
    subplot(nx,1,k)
    hold on
    stairs(time,x(k,:),'b')
    plot([0 t(end)], [xmin(k) xmin(k)], 'r--')
    plot([0 t(end)], [xmax(k) xmax(k)], 'r--')
    plot([0 t(end)], [0 0], 'k--')
    xlabel('times [s]')
    ylabel(['y_',num2str(k)])
end

figure, grid on
for k = 1:nu
    subplot(nu,1,k)
    hold on
    
    stairs(time,u(k,:),'b')
    plot([0 t(end)], [umin(k) umin(k)], 'r--')
    plot([0 t(end)], [umax(k) umax(k)], 'r--')
    plot([0 t(end)], [0 0], 'k--')
    xlabel('times [s]')
    ylabel(['u_',num2str(k)])
end
%% Data collection
% initial conditions
% --- use your brain --- (and chat gpt)

%% placeholders for data
% --- use your brain --- (and chat gpt)

%% Simulation loop
t_start = cputime;
% --- use your brain --- (and chat gpt)
t_end = cputime;

fprintf(['\nData generation time: ', num2str(t_end-t_start), 's\n'])

%% visualize data
% --- use your brain --- (and chat gpt)

%% saving data
% --- use your brain --- (and chat gpt)
