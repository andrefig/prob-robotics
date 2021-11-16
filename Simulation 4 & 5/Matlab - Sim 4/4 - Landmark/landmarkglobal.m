function [P, X, Y] = landmarkglobal(xt, fi, ci, M)


%Minimos e máximos em cada coordenada
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
         P(ix, iy, iz) = landmark(fi, ci, [x y theta]', M);
      end
   end
end
X = X(:,:,1);
Y = Y(:,:,2);
P = sum(P, 3);