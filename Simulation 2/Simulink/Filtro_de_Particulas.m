clear
clc

%% Filtro de Partículas
Ts = 0.1; % Sampling time
xTrue = [1; 2; pi/6]; % True initial pose
x = [3; 0; 0]; % Initial pose estimate
P = diag([9 9 0.6]); % Initial covariance matrix of the pose estimate
Q = diag([0.1 0.1]); % Noise covariance matrix of movement actuator
R = diag([0.5 0.3]); % Noise covariance matrix of distance and
% angle measurement
enableNoise = 1; % Enable noise: 0 or 1
N = 300; % Number of simulation sample times
marker = [0 0; 5 5]; % Positions of markers
R = R(1,1); % Only distance measurement

% Particle initialization
nParticles = 500;
xP = repmat(xTrue, 1, nParticles) + diag([4 4 1])*randn(3, nParticles);
W = ones(nParticles, 1)/nParticles; % All particles have equal probability

xreal = [1];
yreal = [2];
xprevista = [3];
yprevista = [0];

% Loop
for k = 1:N
    u = [0.5; 0.2]; % Movement command (translational and angular velocity)
    u_sum = u + sqrt(Q)*randn(2, 1)*enableNoise;

    % Simulation of the true mobile robot state (pose)
    xTrue = xTrue + Ts*[u_sum(1)*cos(xTrue(3)); ...
    u_sum(1)*sin(xTrue(3)); ...
    u_sum(2)];
    xTrue(3) = wrapToPi(xTrue(3));
    
    % Simulation of the true noisy measurements (distance)
    zTrue = [];
    for m = 1:size(marker, 1)
        dist = sqrt((marker(m,1)-xTrue(1))^2 + (marker(m,2)-xTrue(2))^2);
        
        zz = [dist] + sqrt(R)*randn(1, 1)*enableNoise;
        zz(1) = abs(zz(1));
        zTrue = [zTrue; zz];
    end

    % Prediction
    for p = 1:nParticles
        % Particles are moved according to the noise model
        un = u + sqrt(Q)*randn(2, 1)*1;
        xP(:,p) = xP(:,p) + Ts*[un(1)*cos(xP(3,p)); ...
        un(1)*sin(xP(3,p)); ...
        un(2)];
        xP(3,p) = wrapToPi(xP(3,p));
    end

    % Correction
    for p = 1:nParticles
        % Estimated measurement for every particle
        z = [];
        for m = 1:size(marker, 1)
            dist = sqrt((marker(m,1)-xP(1,p))^2 + (marker(m,2)-xP(2,p))^2);
            zz = [dist];
            zz(1) = abs(zz(1));
            z = [z; zz];
        end

        Innov = zTrue - z; % Determine innovation

        % Determine particle weights (particle probability)
        % Measurement covariance matrix
        RR = diag(repmat(diag(R), size(marker, 1), 1));
        W(p) = exp(-0.5*Innov.'*inv(RR)*Innov) + 0.0001;
    end

    iNextGeneration = obtainNextGenerationOfParticles(W, nParticles);
    xP = xP(:,iNextGeneration);
    
    % The new state estimate is the average of all the particles
    x = mean(xP, 2);
    x(3) = wrapToPi(x(3));
    % For robot orientation use the most likely particle instead of the
    % average angle of all the particles.
    [gg, ggi] = max(W);
    x(3) = xP(3,ggi);

    
    %% Gráficos
    %Atualização das variáveis do gráfico
    xreal(k) = xTrue(1);
    yreal(k) = xTrue(2);
    
        if k == 1
            %Plot dos gráficos
            figure(1)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
        end

    
        if k == 10
            %Plot dos gráficos
            figure(2)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
        end
        
    
        if k == 20
            %Plot dos gráficos
            figure(3)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
        end
        
        
        if k == 30
            %Plot dos gráficos
            figure(4)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
        end
    
        if k == 40
            %Plot dos gráficos
            figure(5)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
        end
            
        if k == 50
            %Plot dos gráficos
            figure(6)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
        end
      
        
        if k == 60
            %Plot dos gráficos
            figure(7)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
        end

        
        if k == 90
            %Plot dos gráficos
            figure(8)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
        end
        
        if k == 150
            %Plot dos gráficos
            figure(9)
            clf
            %Movimento real
            plot(xreal,yreal,'-')
            hold on
        
            xparticulas = xP(1,:);
            yParticulas = xP(2,:);
            plot(xparticulas,yParticulas,'.')
            
            % Robô
            r = [0:pi/45:2*pi];
            n = numel(r);
            c = 0.2*cos(r);
            s = 0.2*sin(r);
            hold on
            plot(c+xTrue(1),s+xTrue(2),'-k')
            xlabel('X')
            ylabel('Y')
            grid on
            axis equal
            set(gca, 'color', [0.8 0.8 0.8])
            axis([-3 5 -1 8])
            legend('Movimento Real','Partículas','Robô','Location','bestoutside')
        end
end



function iNextGeneration = obtainNextGenerationOfParticles(W, nParticles)
    % Selection based on the particle weights
    CDF = cumsum(W)/sum(W);
    iSelect = rand(nParticles, 1); % Random numbers
    % Indices of the new particles
    CDFg = [0; CDF];
    indg = [1; (1:nParticles).'];
    iNextGeneration_float = interp1(CDFg, indg, iSelect, 'linear');
    iNextGeneration=round(iNextGeneration_float + 0.5); % Round the indices
end