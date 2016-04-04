% ECE 6504 Advanced Topics in Robotics
% HW2: Problem 4
% 2-D EKF Localization with landmark updates

clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Configuration settings

N = 400; % number of timesteps
dt = 1; % sampling time

% Landmark coordinates
n_lm = 10;
m = 10*(rand(2,n_lm));
%%
% Noise strengths
sigma_u = 0.2;
R = diag([sigma_u^2 sigma_u^2]);
sigma_r = 0.1; % [m]    distance measurements


% Initial conditions
x_true_1 = [0 0]'; % initial starting position [x y]
x_hat_1  = [0 0]'; % initial estimate
Sigma_1 = zeros(2); % initial covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Real World Simulation
% provides x_true, u, z_r

% True trajectory
u_true = 0.1*[ones(1,N/4) zeros(1,N/4) -1*ones(1,N/4) zeros(1,N/4); ...
    zeros(1,N/4) ones(1,N/4) zeros(1,N/4) -1*ones(1,N/4)];
%%
[x_true, u, z_r] = rws_2D(N, dt, x_true_1, u_true, m, sigma_u, sigma_r);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bookkeeping

% pre-allocate space for certain values we want to keep track of
x_hat_min = zeros(2,N); % state estimate after Propagation
x_hat_plus = zeros(2,N); % state estimate after update
Sigma_min = zeros(2,2,N); % covariance after Propagation
Sigma_plus = zeros(2,2,N); % covariance after update

% initialize those with the right values where appropriate
x_hat_min(:,1) = x_hat_1;
x_hat_plus(:,1) = x_hat_1;
Sigma_plus(:,:,1) = Sigma_1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EKF

% notice that we let the filter start with a propagation step.
for i = 2:N
    
    % Propagation
    [x_hat_min(:,i), Sigma_min(:,:,i)] = KF_propagate(x_hat_plus(:,i-1), Sigma_plus(:,:,i-1), u(:,i), R, dt);
    
    
    % distance update
    [x_hat_plus(:,i), Sigma_plus(:,:,i)] = EKF_update_range(x_hat_min(:,i), Sigma_min(:,:,i), m, z_r(:,i), sigma_r);
    
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization

% True state vs. posterior est
%%
figure('Name','2D Trajectory'); hold on
plot(x_true(1,:), x_true(2,:), 'b');

%%
plot(x_hat_plus(1,:), x_hat_plus(2,:), 'r');
plot(m(1,:), m(2,:), 'mx')
xlabel('x (m)')
ylabel('y (m)')
axis equal
legend('True State','Estimate','Landmarks')
%%
% add error ellipses
for i = 1:floor(N/20):N
    plot_error_ellipse(x_hat_plus(1:2,i), Sigma_plus(1:2,1:2,i));
end
t = 0:dt:(N-1)*dt;

% error2d = x_true-x_hat_plus;
 
 sigma_bound_y = zeros(1,N);
 error2d_iny = zeros(1,N);
 error2d_inx = zeros(1,N);
 for i = 1:N
    sigma_bound_y(1,i)= sqrt(Sigma_plus(2,2,i));
    sigma_bound_x(1,i)= sqrt(Sigma_plus(1,1,i));
    
    error2d_iny(1,i) = ((x_hat_plus(2,i)- x_true(2,i)));
    error2d_inx(1,i) = ((x_hat_plus(1,i)- x_true(1,i)));
 end
 
%  
%   sigma_bound_y = zeros(1,N);
%  error2d_iny = zeros(1,N);
%  for i = 1:N
%     sigma_bound_y(1,i)= sqrt(Sigma_plus(1,1,i));
%     
%     
%     error2d_iny(1,i) = ((x_hat_plus(1,i)- x_true(1,i)));
%  end
% plot error in x and y coordinates
% along with the 3sigma bounds
figure('Name','Position Error 2D in y '); hold on
plot(t, error2d_iny(:));
plot(t, 3*sigma_bound_y(:),'r');
plot(t,-3*sigma_bound_y(:),'r');


figure('Name','Position Error 2D in x '); hold on
plot(t, error2d_inx(:));
plot(t, 3*sigma_bound_x(:),'r');
plot(t,-3*sigma_bound_x(:),'r');



% % Error
% figure('Name','Position Error'); hold on
% plot(t, x_true-x_hat_plus) 
% plot(t, 3*sqrt(Sigma_plus),'r')
% plot(t,-3*sqrt(Sigma_plus),'r')








