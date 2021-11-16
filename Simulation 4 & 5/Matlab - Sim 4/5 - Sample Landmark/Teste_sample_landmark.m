clear
clc
clf




fi = [5 pi/6 0; 10 pi/6 0];

M = [3 4; 0 0];

ci = 1;

n = 2000;

figure(1)
xt = sample_landmark(fi, ci, M, n);
plot(xt(1,:), xt(2,:), '.');
hold on
title('Landmark - 5m e 30º')
xlabel('X')
ylabel('Y')
axis equal


ci = 2;
figure(2)
xt = sample_landmark(fi, ci, M, n);
plot(xt(1,:), xt(2,:), '.');
hold on
title('Landmark - 10m e 30º')
xlabel('X')
ylabel('Y')
axis equal




