function [p_hit ,p_short, p_rand, p_max]  = probs(z,range,range_max,sigma_a,lambda_a)
    %global z_hit z_short z_max z_rand sigma lambda
    if range<range_max
        pd_hit = makedist('Normal','mu',range,'sigma',sigma_a);
        pd_hit = truncate(pd_hit,0,range_max);

        pd_short = makedist('Exponential','mu',(1/(lambda_a)));
        pd_short = truncate(pd_short,0,range);

        pd_rand = makedist('Uniform','Lower',0,'Upper',range_max);  
        
        p_hit = pdf(pd_hit,z);
        p_short=pdf(pd_short,z);
        p_rand=pdf(pd_rand,z);
        
        p_max = 0;
    else
        p_hit = 0;
        p_short=0;
        p_rand=0; 
        p_max = 1/10*eps;
    end
    
   % pd_max = makedist('Uniform','Lower',range_max,'Upper',range_max+10*eps);
    

    %p_max=pdf(pd_max,z);
    %p = p_hit*z_hit_a + p_short*z_short_a +p_rand*z_rand_a + p_max*z_max_a;
end