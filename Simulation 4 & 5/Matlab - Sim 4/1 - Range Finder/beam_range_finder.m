
function q = beam_range_finder(z,pos,map,range_max,tamanho)
    %i=1;
    angle_step = 2*pi/size(z,1);
    %range_max=45;
    %result = zeros(2,size(angle_step:angle_step:2*pi,2));
    q =1;
    k=1;
    for ang = angle_step:angle_step:2*pi
            [range,aux1,~,index] = rangefinder(pos+[0 0 ang],map,range_max,tamanho);
            %plot(aux1(:,1),aux1(:,2),'ro')
            %plot(aux1(index,1),aux1(index,2),'go')
            p=probability(z(k),range,range_max);
            q = q*p;
            k=k+1;
    end
end

function p = probability(z,range,range_max)

    z_hit = 0.6;
    z_short = 0.1;
    z_max = 0.2;
    z_rand = 0.1;

    sigma = 1;
    lambda = 1;

    pd_hit = makedist('Normal','mu',range,'sigma',sigma);
    if z<=range_max
        p_hit = pdf(pd_hit,z);
        p_hit = p_hit/cdf(pd_hit,range_max);
    else
        p_hit = 0;    
    end

    pd_short = makedist('Exponential','mu',1/lambda);
    if z<=range %até o objeto em questão
        p_short = pdf(pd_short,z);
    else
        p_short = 0;    
    end

    if z<=range_max 
        p_rand = 1/range_max;
        p_max = 0;
    else
        p_rand = 0; 
        p_max = 1;    
    end
    
    p = p_hit*z_hit + p_short*z_short +p_rand*z_rand + p_max*z_max;
end