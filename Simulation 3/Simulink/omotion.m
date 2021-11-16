function p = omotion(xt, ut, xs, alpha)
% VMOTION Velocity motion model
% Implements the velocity motion model described in Table 5.1
% of the textbook. All vectors are column vectors, and all angles
% are in radians.
%
% xt    - state at time t
% ut    - control at time t (linear and angular velocities)
% xs    - state at time t-1
% alpha - noise coefficients
% dt    - time step duration
%
% p     - probability density


% state variables at time t
x1 = xt(1);
y1 = xt(2);
th1 = xt(3);

% state variables at time t-1
x0 = xs(1);
y0 = xs(2);
th0= xs(3);

% control at time t
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

  %amostragem (linhas 5 a 7)
  delta_rot_1_hat = atan2(y1-y0,x1-x0)-th0;
  delta_trans_hat = sqrt((x0-x1)^2+(y0-y1)^2);
  delta_rot_2_hat = th1-th0-delta_rot_1_hat;
  
  
  s_1 = alpha(1)*delta_rot_1_hat+alpha(2)*delta_trans_hat;
  s_2 = alpha(3)*delta_trans_hat +  alpha(4)*(delta_rot_1_hat+delta_rot_2_hat);
  s_3 = alpha(1)*delta_rot_2_hat+alpha(2)*delta_trans_hat;
  
p1 = mvnpdf(delta_rot_1 - delta_rot_1_hat , 0, s_1);
p2 = mvnpdf(delta_trans - delta_trans_hat, 0, s_2);
p3 = mvnpdf(delta_rot_2 - delta_rot_2_hat, 0, s_3);

p = p1*p2*p3;