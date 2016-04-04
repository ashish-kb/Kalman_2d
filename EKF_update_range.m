function [x_hat_plus, Sigma_plus] = EKF_update_range(x_hat_min, Sigma_min, m, z_r, sigma_r)

    %x_hat_min(:,i)
    [sz_m_r sz_m_c] = size(m);
    H_cell = cell(sz_m_c,1);
    h_func_cell = cell(sz_m_c,1);
    
    for i = 1:sz_m_c
        
       H_cell{i}  = [(x_hat_min(1)-m(1,i))/sqrt((x_hat_min(1)-m(1,i)).^2 + (x_hat_min(2)-m(2,i)).^2) (x_hat_min(2)-m(2,i))/sqrt((x_hat_min(1)-m(1,i)).^2 + (x_hat_min(2)-m(2,i)).^2)]; %x_hat_min(:,i) is the input so x_hat_min(1) is the x coordinate of ith state
       
       h_func_cell{i} = sqrt((x_hat_min(1)-m(1,i)).^2 + (x_hat_min(2)-m(2,i)).^2);
       
    end
    
    
    H = cell2mat(H_cell);
    h_func = cell2mat(h_func_cell);

    %x_hat_plus = Sigma_min 
    Kt =  Sigma_min*H'/(H*Sigma_min*H' + sigma_r^2*eye(10));
    
    
    x_hat_plus = x_hat_min + Kt*(z_r - h_func) ;
    
    Sigma_plus = (eye(2) - Kt*H)*Sigma_min;

end


%  Kt = Sigma_min /(Sigma_min + sigma_g);
%  x_hat_plus =  x_hat_min + Kt*(z_g - x_hat_min);
%     
%     
%  Sigma_plus = (1 - Kt)* Sigma_min;