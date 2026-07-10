%Define variables

m = 0.2; % mass of the object
M = 1; % mass of the cart
g = 9.81; % acceleration due to gravity
L = 0.5; % length of the pendulum


A = [0 1 0 0;   %describes how the system behaves
     0 0 -m*g/M 0; 
     0 0 0 1; 
     0 0 (M+m)*g/(M*L) 0]; 

B = [0; 1/M; 0; -1/(M*L)]; 

eig(A);

Co = ctrb(A,B);

rank(Co);

Q = diag([1,1,10,10]); %[x,x.,theta,theta.] weights
R = 1; %higher - gentler, lower - aggresive control

K = lqr(A, B, Q, R); %returns 1x4 gain vector 

eig(A - B*K); %check if system is stable

D = zeros(4,1);

sys_cl = ss(A-B*K, B, eye(4), 0);   % closed-loop system, all 4 states as output

t = 0:0.01:15; % set specific running time (ideal system)

%t = linspace(0,20, size(y,1))'; %use this line with simulink model
x0 = [0; 0; deg2rad(45); 0];    % initial condition: pendulum angle offset
[y_ideal, t] = initial(sys_cl, x0, t);
plot(t, y_ideal)
legend('x', 'xdot', 'theta', 'thetadot')
%axis auto
grid on
