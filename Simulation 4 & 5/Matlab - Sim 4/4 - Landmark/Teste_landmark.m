clear
clc

fi = [5 -pi/2 1; 5 0 1; 5 pi/2 1];

M = [0 0; 0 0; 0 0; 0 0];

ci = 1;

n = 2000;


figure(1)
clf
xt = sample_landmark(fi, ci, M, n);
plot(xt(1,:), xt(2,:), '.');
hold on
[P, X, Y] = landmarkglobal(xt, fi, ci, M);
contour(X, Y, P, 20)
xlabel('X')
ylabel('Y')
axis([-7 7 -7 7])


ci = 2;
figure(2)
clf
xt = sample_landmark(fi, ci, M, n);
plot(xt(1,:), xt(2,:), '.');
hold on
[P, X, Y] = landmarkglobal(xt, fi, ci, M);
contour(X, Y, P, 20)
xlabel('X')
ylabel('Y')
axis([-7 7 -7 7])

ci = 3;
figure(3)
clf
xt = sample_landmark(fi, ci, M, n);
plot(xt(1,:), xt(2,:), '.');
hold on
[P, X, Y] = landmarkglobal(xt, fi, ci, M);
contour(X, Y, P, 20)
xlabel('X')
ylabel('Y')
axis([-7 7 -7 7])

