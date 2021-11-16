function p = vmotion(xt, ut, xs, alpha, dt)
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
xprime = xt(1);
yprime = xt(2);
thetaprime = xt(3);

% state variables at time t-1
x = xs(1);
y = xs(2);
theta = xs(3);

% control at time t
v = ut(1);
omega = ut(2);

mu = 0.5 * ((x - xprime)*cos(theta) + (y - yprime)*sin(theta)) / ...
           ((y - yprime)*cos(theta) - (x - xprime)*sin(theta));

xstar = 0.5 * (x + xprime) + mu * (y - yprime);

ystar = 0.5 * (y + yprime) + mu * (xprime - x);

rstar = sqrt((x - xstar)^2 + (y - ystar)^2);

dtheta = atan2(yprime - ystar, xprime - xstar) - ...
         atan2(y - ystar, x - xstar);

vhat = (dtheta / dt) * rstar;

omegahat = (dtheta / dt);

gammahat = (thetaprime - theta) / dt - omegahat;

% noise variances
sv = alpha(1) * v^2 + alpha(2) * omega^2;
somega = alpha(3) * v^2 + alpha(4) * omega^2;
sgamma = alpha(5) * v^2 + alpha(6) * omega^2;

p = mvnpdf(v - vhat, 0, sv) * mvnpdf(omega - omegahat, 0, somega) * ...
    mvnpdf(gammahat, 0, sgamma);