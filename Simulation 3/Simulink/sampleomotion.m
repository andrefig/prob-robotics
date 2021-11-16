function xt = sampleomotion(ut, xs, alpha, n)
% SAMPLEVMOTION Sample from the velocity motion model
% Implements the motion model sampling algorithm described in
% Table 5.3 of the textbook. All vectors are column vectors and
% all angles are in radians.
%
% ut    - control at time t (linear and angular velocities)
% xs    - state at time t-1
% alpha - noise coefficients
% dt    - time step duration
% n     - the number of samples to generate
%
% xt    - a 3xn matrix of sampled states (each column representing
%         a separate sample)

xt = zeros(3, n);
for a = 1:n
  % state variables at time t-1
  x = xs(1);
  y = xs(2);
  th = xs(3);

  % controle u(t) = x_(t-1), x_(t) || x = x y th
  x_0 = ut(1);
  y_0 = ut(2);
  th_0 = ut(3);
  x_1=ut(4);
  y_1=ut(5);
  th_1 =ut(6);

  % deltas (lihas 2 a 4)
  delta_rot_1 = atan2(y_1-y_0,x_1-x_0)-th_0;
  delta_trans = sqrt((x_0-x_1)^2+(y_0-y_1)^2);
  delta_rot_2 = th_1-th_0-delta_rot_1;
  
  %variancias (auxiliar para linhas 5 a 7)
  s_rot_1 = abs(  alpha(1) * delta_rot_1 + alpha(2) * delta_trans);
  s_trans = abs( alpha(3) * delta_trans + alpha(4) * (delta_rot_1+delta_rot_2));
  s_rot_2 = abs( alpha(1) * delta_rot_2 + alpha(2) * delta_trans);
  
  %amostragem (linhas 5 a 7)
  delta_rot_1_hat = delta_rot_1 - mvnrnd(0, s_rot_1);
  delta_trans_hat = delta_trans - mvnrnd(0, s_trans);
  delta_rot_2_hat = delta_rot_2 - mvnrnd(0, s_rot_2);

  %novos x (linhas 8 a 10)
  xprime = x + delta_trans_hat*cos(th+delta_rot_1_hat) ;
  yprime = y + delta_trans_hat*sin(th+delta_rot_1_hat) ;
  thprime = th + delta_rot_1_hat + delta_rot_2_hat;

  xt(:,a) = [xprime; yprime; thprime];

end