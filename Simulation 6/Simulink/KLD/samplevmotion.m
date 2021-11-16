function xt = samplevmotion(ut, xs, alpha, dt, n)
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
  theta = xs(3);

  % control at time t
  v = ut(1);
  omega = ut(2);

  % noise variances
  sv = alpha(1) * v^2 + alpha(2) * omega^2;
  somega = alpha(3) * v^2 + alpha(4) * omega^2;
  sgamma = alpha(5) * v^2 + alpha(6) * omega^2;

  % generate noisy velocities
  vhat = v + mvnrnd(0, sv);
  omegahat = omega + mvnrnd(0, somega);
  gammahat = 0 + mvnrnd(0, sgamma);

  % ratio of linear to angular velocities
  f = vhat / omegahat;

  xprime = x - f*sin(theta) + f*sin(theta + omegahat*dt);
  yprime = y + f*cos(theta) - f*cos(theta + omegahat*dt);
  thetaprime = theta + omegahat*dt + gammahat*dt;

  xt(:,a) = [xprime; yprime; thetaprime];

end