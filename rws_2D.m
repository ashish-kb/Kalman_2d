function [x_true, u, z_g] = rws_2D(N, dt, x_true_1, u_true, m, sigma_u, sigma_r)

    n = length(x_true_1);
    [sz_m_r sz_m_c] = size(m);
    % preallocate
    x_true = zeros(n,N);

    %   apply u_true to determine x_true

    x_true(:,1) = x_true_1;

    for i = 2 : N

        % x_k = x_k-1 + uk*dt
        x_true(:,i) = x_true(:,i-1) + u_true(:,i)*dt;

    end


    % Generate noisy u by sampling from Gaussian(mean u_true, std sigma_u)
    % assume first u is always zero so that the robot starts at rest
    u = u_true + sigma_u .* randn(n,N);
    u(:,1) = 0; % ensure start from 0

    z_g_cell = cell(10,1);

    for i = 1:sz_m_c

        z_g_cell{i,1} = sqrt((x_true(1,:) - m(1,i)).^2 + (x_true(2,:) - m(2,i)).^2); %+ sigma_r.*randn(sz_m_c,N);

    end

    z_g = cell2mat(z_g_cell);

    z_g = z_g + sigma_r.*randn(sz_m_c,N); % adding the randomness- considering that each measurement draws from its own normal curve at every time step.






end

% n = length(x_true_1);
% 
% % preallocate
% x_true = zeros(n,N);
% 
% % for each timestep
% %   apply u_true to determine x_true
% x_true(:,1) = x_true_1;
% for i = 2 : N
%     
%     % x_k = x_k-1 + uk*dt
%     x_true(:,i) = x_true(:,i-1) + u_true(:,i)*dt;
%     
% end
% 
% % Generate noisy u by sampling from Gaussian(mean u_true, std sigma_u)
% % assume first u is always zero so that the robot starts at rest
% u = u_true + sigma_u .* randn(n,N);
% u(:,1) = 0;
% 
% % Generate noisy z_g by sampling from Gaussian(mean x_true, std sigma_g)
% z_g = x_true + sigma_g * eye(n) .* randn(n,N);