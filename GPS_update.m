function [ x_hat_plus, Sigma_plus ] = GPS_update(x_hat_min, Sigma_min, z_g, sigma_g);
    
    Kt = Sigma_min /(Sigma_min + sigma_g);
    x_hat_plus =  x_hat_min + Kt*(z_g - x_hat_min);
    
    
    Sigma_plus = (1 - Kt)* Sigma_min;



end

%GPS_update(x_hat_min(1,i), Sigma_min(1,i), z_g(1,i), sigma_g);