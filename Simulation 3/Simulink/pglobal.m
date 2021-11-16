function [P, X, Y] = pglobal(xt, ut, xs, alpha, dt)
% PGLOBAL Evaluate the motion model over a range of states.
% [P, X, Y] = pglobal(xt, ut, xs, alpha, dt) computes the approximate
% position probability density over a range of states; it does so in
% such a way that generating a contour plot of the density is simple.
%
% You should first generate many samples using SAMPLEVMOTION and then
% pass the results to PGLOBAL.
%
% X and Y are matrices of x and y coordinates suitable for use in the
% CONTOUR family of functions.

xmin = min(xt(1,:));
xmax = max(xt(1,:));
dx = (xmax - xmin) / 40;

ymin = min(xt(2,:));
ymax = max(xt(2,:));
dy = (ymax - ymin) / 40;

zmin = min(xt(3,:));
zmax = max(xt(3,:));
dz = (zmax - zmin) / 100;


[X,Y,Z] = meshgrid(xmin:dx:xmax, ymin:dy:ymax, zmin:dz:zmax);
[nx, ny, nz] = size(X);
P = zeros(size(X));
for ix = 1:nx
   for iy = 1:ny
      for iz = 1:nz
         x = X(ix, iy, iz);
         y = Y(ix, iy, iz);
         theta = Z(ix, iy, iz);
         P(ix, iy, iz) = vmotion([x y theta]', ut, xs, alpha, dt);
      end
   end
end
X = X(:,:,1);
Y = Y(:,:,1);
P = sum(P, 3);