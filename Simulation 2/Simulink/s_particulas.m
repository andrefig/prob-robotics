function [sys,x0,str,ts] = s_particulas(t,x,u,flag, x0a)
% eixos = [-10 10 -10 10];
P = diag([3 3 2*pi]); % Initial covariance matrix of the pose estimate
Q = diag([0.1 0.1]); % Noise covariance matrix of movement actuator
R = diag([0.5 0.3]); % Noise covariance matrix of distance and
R = R(1,1);          % Only distance measurement

%Q = diag([eps eps]); % Noise covariance matrix of movement actuator
%R = diag([eps eps]); % Noise covariance matrix of distance and
%R = R(1,1);

n_Particulas = 1000;
n_estados = n_Particulas*3;
W = ones(n_Particulas, 1)/n_Particulas;

marker = [0 0; 5 5; 0 5]; % Positions of markers
num_referencias = size(marker, 1);

t_s=0.1;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,

        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = n_estados;
        sizes.NumOutputs     = n_estados;
        sizes.NumInputs      = 2+num_referencias;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        particulas = repmat(x0a', 1, n_Particulas) + P*2*(rand(3, n_Particulas)-0.5*ones(3, n_Particulas)); % Gera partículas aleatórias
        x0 = reshape(particulas,[],1);
        str = [];
        ts  = [t_s 0]; 

% end mdlInitializeSizes

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,  
  % 
    xP = reshape(x,3,[]);
    dist_medida = u(3:(2+num_referencias));
    for p = 1:n_Particulas
        % Aplica movimento aleatório:
        un = u(1:2) + sqrt(Q)*randn(2, 1)*1; %entrada com ruído
        xP(:,p) = xP(:,p) + t_s*[un(1)*cos(xP(3,p)); ...
        un(1)*sin(xP(3,p)); ...
        un(2)]; %estado atualizado pela entrada
        xP(3,p) = wrapToPi(xP(3,p));
    end

    % Correção
    for p = 1:n_Particulas
        % Estima mediçao para a partícula
        z = [];
        dist=zeros(num_referencias,1);
        for m = 1:num_referencias
            dist(m) = sqrt((marker(m,1)-xP(1,p))^2 + (marker(m,2)-xP(2,p))^2);
        end
        Innov = dist_medida - dist; % Inovação
        % Pesos e Matriz de Covariância
        RR = diag(repmat(diag(R), size(marker, 1), 1));
        W(p) = exp(-0.5*Innov.'*inv(RR)*Innov) + 0.0001;
    end
        fat=max(W)-min(W);
        fat=0;
        if fat>0.3 && fat<0.7
            iNextGeneration = LowVarianceResampler(W, n_Particulas);
        else
            iNextGeneration =obtainNextGenerationOfParticles(W, n_Particulas);
        end
    xP = xP(:,iNextGeneration);
    sys = reshape(xP,1,[]);
  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,  
      
      %[gg, ggi] = max(W);
      %x_ = x(ggi*3-2:ggi*3);
      sys = [x(:)];
%     % The new state estimate is the average of all the particles
%     xP = reshape(x,3,[]);
%     x_ = mean(xP, 2);
%     x_(3) = wrapToPi(x_(3));
%     % For robot orientation use the most likely particle instead of the
%     % average angle of all the particles.
%     [gg, ggi] = max(W);
%     x_(3) = xP(3,ggi);
% 
%     sys = [x_(:)];
%     particulas = reshape(x,3,[]);
%     %#codegen
%     %coder.extrinsic('scatter')
%     subplot(2,1,1);
%     scatter(particulas(1,:),particulas(2,:), 'b.');
%     axis([eixos]);
%     grid on;
%     hold on;
%     plot(x_(1),x_(2),'r*');
%     scatter(marker(:,1),marker(:,2), 'gs');
%     hold off;
%     % insert additional code as needed to turn on grid, setup axis limits, etc.
% 
%   %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,                                                
    sys = []; % do nothing

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
