clc
clear

% start at origin facing to right
xs = [0; 0; 0];

% v = 1 m/s, w = 5 deg/s
ut = [1; 5*pi/180];

Ts = 1; % Tempo de amostragem
xs = [0; 0; 0]; % Pose inicial
N = 40; % Number of simulation sample times


xreal = [0];
yreal = [0];

% Loop
for k = 1:N
    % Simulation of the true mobile robot state (pose)
    xs = xs + Ts*[ut(1)*cos(xs(3)); ...
    ut(1)*sin(xs(3)); ...
    ut(2)];
    xs(3) = wrapToPi(xs(3));
      
    %Atualização das variáveis do gráfico (Movimento Real)
    xreal(k) = xs(1);
    yreal(k) = xs(2);
  
    
    if k == 10 | k == 20 | k == 30
    %Plot dos gráficos
    figure(1)
    plot(xreal,yreal,'-b')
    hold on
    
    % Robô
    r = [0:pi/45:2*pi];
    n = numel(r);
    c = 2*cos(r);
    s = 2*sin(r);
    hold on
    plot(c+xs(1),s+xs(2),'-k')
    xlabel('X')
    ylabel('Y')
    grid on
    set(gca, 'color', [0.8 0.8 0.8])
    axis([-10 30 -5 25])
    
    %Plot dos gráficos
    figure(2)
    plot(xreal,yreal,'-b')
    hold on
    
    % Robô
    r = [0:pi/45:2*pi];
    n = numel(r);
    c = 2*cos(r);
    s = 2*sin(r);
    hold on
    plot(c+xs(1),s+xs(2),'-k')
    xlabel('X')
    ylabel('Y')
    grid on
    set(gca, 'color', [0.8 0.8 0.8])
    axis([-10 30 -5 25])
    
    %Plot dos gráficos
    figure(3)
    plot(xreal,yreal,'-b')
    hold on
    
    % Robô
    r = [0:pi/45:2*pi];
    n = numel(r);
    c = 2*cos(r);
    s = 2*sin(r);
    hold on
    plot(c+xs(1),s+xs(2),'-k')
    xlabel('X')
    ylabel('Y')
    grid on
    set(gca, 'color', [0.8 0.8 0.8])
    axis([-10 30 -5 25])
    end

end


xs = [0; 0; 0];

 

for dt = [0 10 20 30]
    figure(1)
    % left-most figure from Slide 14
    alpha = [4 4 2 2 0.1 0.1] * 1e-4;  
    xt = samplevmotion(ut, xs, alpha, dt, 1000);
    [P, X, Y] = pglobal(xt, ut, xs, alpha, dt);
    contour(X, Y, P, 20)
    hold on
end

for dt = [0 10 20 30]
    figure(2)
    %middle figure from Slide 14
    alpha = [50 50 0.1 0.1 0.1 0.1] * 1e-4;  
    xt = samplevmotion(ut, xs, alpha, dt, 1000);
    [P, X, Y] = pglobal(xt, ut, xs, alpha, dt);
    contour(X, Y, P, 20)
    hold on
end

for dt = [0 10 20 30]
    figure(3)
    % right-most figure from Slide 14
    alpha = [1 1 8 8 0.1 0.1] * 1e-4;
    xt = samplevmotion(ut, xs, alpha, dt, 1000);
    [P, X, Y] = pglobal(xt, ut, xs, alpha, dt);
    contour(X, Y, P, 20)
    hold on
end

