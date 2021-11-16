function [ zz, tipo] = sample_reading(range,range_max)
    global z_hit z_short z_max z_rand sigma lambda
    
    %xxx = (z_hit+z_short +z_rand )/(z_hit+ z_short+ z_max+z_rand );
    tipo = rand();%*xxx;
    if range<range_max
        if tipo < z_hit
            tipo =1;
            pd = makedist('Normal','mu',range,'sigma',sigma);
            pd = truncate(pd,0,range_max);

        elseif tipo< z_hit+z_short
            tipo=2;
            pd = makedist('Exponential','mu',(1/(lambda)));
            pd = truncate(pd,0,range);

        elseif tipo< z_hit+z_short+z_rand
            tipo=3;
            pd = makedist('Uniform','Lower',0,'Upper',range_max);  
        else
            tipo=4;
            pd = makedist('Uniform','Lower',range_max,'Upper',range_max+10*eps);
        end
        zz= random(pd);
    else
            tipo=4;
            zz = range_max;
    end
    %if tipo<4
    
    %else
    %    zz=range_max;
    %end

end