function [ x_hat_min, Sigma_min ] = KF_propagate(x_hat_plus, Sigma_plus, u, sigma_u, dt)
    
    % sigma_u in 2d case is R
    
    x_hat_min = x_hat_plus + dt*u;
    Sigma_min = Sigma_plus + sigma_u; 
    



end

%KF_propagate(x_hat_plus(1,i-1), Sigma_plus(1,i-1), u(1,i), sigma_u, dt)