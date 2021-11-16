function xt = sample_landmark(fi, ci, M, n)
% Função sample landmark
%
% fi    - 
% ci    - 
% m - 
% n - 


xt = zeros(3, n);
for a = 1:n
  % caracteristica fi
  fi_aux = fi(ci,:);
  ri = fi_aux(1);
  omegai = fi_aux(2);
  
  % landmark
  M_aux = M(ci,:);
  Mx = M_aux(1);
  My = M_aux(2);

  rerro = 0.1;
  omegaerro = 0.1;
  
  gammahat = rand*2*pi;
  rhat = ri + mvnrnd(0, rerro);
  omegahat = omegai + mvnrnd(0, omegaerro);
  
  
  x = Mx - rhat*cos(gammahat);
  y = My + rhat*sin(gammahat);
  theta = gammahat - pi - omegahat;

  xt(:,a) = [x; y; theta];

end
