clear; close all; rng(42);

%% Model and MPC part
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

options = sdpsettings('verbose',0,'solver','gurobi');
controller = optimizer(cst, obj, options, xx(:,1), uu(:,1));








%% Loading net
load('trained_NN.mat')
x0 = [-10; 0];

%% Closed Loop Simulation

x_nn = zeros(nx,kf+1);
u_nn = zeros(nu,kf);
x_mpc = zeros(nx,kf+1);
u_mpc = zeros(nu,kf);



x_nn(:,1) = x0;
x_mpc(:,1) = x0;

t_start = cputime;

for k = 1:kf
    u_nn(:,k) = min(max(net(x_nn(:,k)), umin), umax);
    x_nn(:, k+1) = A*x_nn(:,k) + B*u_nn(:,k);
end

t_end = cputime;
fprintf(['\nNN simulation time: ', num2str(t_end-t_start), 's\n'])


t_start = cputime;

for k = 1:kf
    u_mpc(:,k) = controller(x_mpc(:,k));
    x_mpc(:, k+1) = A*x_mpc(:,k) + B*u_mpc(:,k);
end


t_end = cputime;
fprintf(['\nMPC simulation time: ', num2str(t_end-t_start), 's\n'])

x_nn(:,end) = [];
x_mpc(:,end) = [];
%%
t = linspace(0,Tmax,kf);


time = t;
figure, grid on
for k = 1:ny
    subplot(nx,1,k)
    hold on
    stairs(time,x_mpc(k,:),'r')
    stairs(time,x_nn(k,:),'b')
    plot([0 t(end)], [xmin(k) xmin(k)], 'r--')
    plot([0 t(end)], [xmax(k) xmax(k)], 'r--')
    plot([0 t(end)], [0 0], 'k--')
    xlabel('times [s]')
    ylabel(['y_',num2str(k)])
    legend("MPC","NN")
end

figure, grid on
for k = 1:nu
    subplot(nu,1,k)
    hold on
    
    stairs(time,u_mpc(k,:),'r')
    stairs(time,u_nn(k,:),'b')
    plot([0 t(end)], [umin(k) umin(k)], 'r--')
    plot([0 t(end)], [umax(k) umax(k)], 'r--')
    plot([0 t(end)], [0 0], 'k--')
    xlabel('times [s]')
    ylabel(['u_',num2str(k)])
    legend("MPC","NN")
end
