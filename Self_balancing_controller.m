
%% System parameters
m = 53;     % lb
M = 10;     % lb
l = 17.5;   % ft
I = 9252; 
b = 0.1;    
g = 32.2;   % ft/s^2 

% Other units test parameters
% m = 5;  % lb
% M = 45; % lb
% l = 36; % in
% I = m*l^2/12;
% b = 0.1;
% g = 32.2*12; % in/s^2 

%% State space parameters A,B,C, and D
A = [0 1 0 0; 
    0 -b*(I+l^2*m)/(M*m*l^2+(M+m)*I) -l^2*m^2*g/(M*m*l^2+(M+m)*I) 0; 
    0 0 0 1; 
    0 l*m*b/(M*m*l^2+(M+m)*I) l*m*g*(M+m)/(M*m*l^2+(M+m)*I) 0];

B = [0 (I+l^2*m)/(M*m*l^2+(M+m)*I) 0 -l*m/(M*m*l^2+(M+m)*I)]';

C = [0 0 1 0]; 

D = 0; 

%% Checking controllability
P = ctrb(A,B)
rank(P)
% System is controllable!

%% Checking observability
Q = obsv(A,C)
rank(Q)
% System is observable!

%% Checking controllability
lambda = eig(A)
% System is unstable!

%% Eigen value plot
figure
im = [0, 0, 0, 0];
eigen = eig(A)
plot(eigen,im,'x')
grid on
legend('Eigen Values')
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
xlabel('real')
ylabel('im') 

%% Desired eigen values
eig_des1 = [ -0.09,-0.1, -0.001,-0.8999];
eig_des2 = [ -0.0999, -.5000, -9.9999,-20];
eig_des3 = [ -0.39999,-4, -4.00001,-40];
eig_des4 = [ -10,-9.9999, -10.00011,-100];

eig_des = eig_des2;
K = place(A,B,eig_des)

%% Initial conditions
x0=[0,0,pi/8,0];
t1=0:0.01:5;
t2=0:0.01:1000;

%% open −loop
%state space model for open loop system
sys= ss(A,B,C,D);
u = 1*ones(length(t1),1);
yol= lsim(sys,u,t1,x0);

%% Closed−loop
%state space model for closed loop
G =0;
syscl= ss(A-B*K,B*G,C-D*K,D*G);
r = 1*ones(length(t2),1);
ycl= lsim(syscl,r,t2,x0);

%% plotting y(t) with open loop and closed loop with PD control

% On same plot
% figure
% title('open loop response')
% plot(t1,yol,'b','LineWidth',2)
% hold on
% plot(t2,ycl,'r','LineWidth',2)
% hold off
% ylim([-.2 3.14])
% xlabel('$t$ (s)', 'Interpreter','latex')
% ylabel('$y$', 'Interpreter','latex')
% legend(["open loop", "closed loop"])

% On different plot
figure
title('open loop response')
plot(t1,yol,'b','LineWidth',2)
xlabel('$t$ (s)', 'Interpreter','latex')
ylabel('$y$', 'Interpreter','latex')

figure
title('close loop response')
plot(t2,ycl,'r','LineWidth',2)
hold on
yline(0,'--')
xlabel('$t$ (s)', 'Interpreter','latex')
ylabel('$y$', 'Interpreter','latex') 

set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

%% System properties 
stepinfo(ycl,t2,0,pi/8)

