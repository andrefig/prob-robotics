clc
clear


% v = 1 m/s, w = 5 deg/s
ut = [1; 5*pi/180];

Ts = 1; % Tempo de amostragem
xs = [0; 0; 0]; % Pose inicial
N = 40; % Number of simulation sample times


xreal = [0];
yreal = [0];
threal = [0];

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
    threal(k) = xs(3);
    
    if k == 10 | k == 20 | k == 30
    for fig=1:5
        %Plot dos gráficos
        figure(fig)
        subplot(1,5,[1:4]);
        plot(xreal,yreal,'-b')
        hold on
%         subplot(2,1,2);
%         plot(threal ,threal,'-k')
%         %plot(threal,'-b')
        hold on    %Plot dos gráficos

        % Robô
        r = [0:pi/45:2*pi];
        n = numel(r);
        c = 2*cos(r);
        s = 2*sin(r);
        
        subplot(1,5,[1:4]);
        hold on
        plot(c+xs(1),s+xs(2),'-k')
        xlabel('X')
        ylabel('Y')
        grid on
        set(gca, 'color', [0.8 0.8 0.8])
        axis([-10 30 -5 25])

    end
    end

end


xs = [0; 0; 0];

 A = [1  1  4  4;...
      10 1  4  4;...
      1  10 4  4;...
      1  1  20 4;...
      1  1  4  50]*1e-4;
 
for fig = 1:5
    alpha = A(fig,:);
for dt = [0 10 20 30]
    figure(fig)
    % left-most figure from Slide 14
    %alpha = [4 4 4 4] * 1e-4;  
    %xt = samplevmotion(ut, xs, alpha, dt, 1000);
    x_0 = xreal(1);
    y_0 = yreal(1);
    th_0 = threal(1);
    
    x_1 = xreal(dt+1);
    y_1 = yreal(dt+1);
    th_1 = threal(dt+1);
    
    ut_odom = [x_0 y_0 th_0 x_1 y_1 th_1];
    xt = sampleomotion(ut_odom, xs, alpha, 1000);
    subplot(1,5,[1:4]);
    [P, X, Y,Z,PP] = pglobal_odom(xt, ut_odom, xs, alpha);
    contour(X, Y, P, 20)
    subplot(1,5,5);
    if size(Z)>0
        plot((PP./sum(PP)),rad2deg(Z));
    end
    hold on
end
subplot(1,5,[1:4]);
hold off
subplot(1,5,5);
hold off
end
