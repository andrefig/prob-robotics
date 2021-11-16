function p = landmark(fi, ci, xt, M)
% Função landmark
%
% fi    - 
% ci    - 
% M - 


% caracteristica fi
fi_aux = fi(ci,:);
ri = fi_aux(1);
phii = fi_aux(2);
si = fi_aux(3);

% landmark
M_aux = M(ci,:);
Mx = M_aux(1);
My = M_aux(2);

% variáveis de estado xt
x = xt(1);
y = xt(2);
theta = xt(3);

rerro = 0.1;
phierro = 0.1;
sierro = 0.1;

rhat = sqrt((Mx - x)^2 + (My - y)^2);
phihat = atan2(My - y, Mx - x);


p = mvnpdf(ri - rhat, 0, rerro) * mvnpdf(phii - phihat, 0, phierro) * ...
    mvnpdf(si - ci, 0, sierro);

end