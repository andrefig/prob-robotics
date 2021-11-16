function [sys,x0,str,ts] = s_aug_mcl(t,x,u,flag, x0a)
% eixos = [-10 10 -10 10];
P = diag([10 10 pi]); % Initial covariance matrix of the pose estimate

alpha = [1 1 1 1 0.1 0.1] * 1e-1; 

%a_slow= 0.1;
%a_fast=0.3;


global rerro
rerro= 0.5;
global phierro
phierro = 0.5;
global sierro
sierro = 0.1;



n_Particulas = 5000;
n_estados =  n_Particulas*3;
W = ones(n_Particulas, 1)/n_Particulas;

marker = [0 0; 5 5]; % Positions of markers
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
        sizes.NumOutputs     = n_estados+5;
        sizes.NumInputs      = 2+3*num_referencias;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        particulas = repmat(x0a', 1, n_Particulas) + P*randn(3, n_Particulas); % Gera partículas aleatórias
        %x0 = [0; 0; reshape(particulas,[],1)];
        x0 = [reshape(particulas,[],1)];
        str = [];
        ts  = [t_s 0]; 

% end mdlInitializeSizes

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,  
  % 
    %xP = reshape(x(3:end),3,[]);
    xP = reshape(x(1:end),3,[]);
    %dist_medida = u(3:(2+num_referencias));
    for p = 1:n_Particulas
        % Aplica movimento aleatório:
%         %un = u(1:2) + sqrt(Q)*randn(2, 1)*1; %entrada com ruído
%         %xP(:,p) = xP(:,p) + t_s*[un(1)*cos(xP(3,p)); ...
%         %un(1)*sin(xP(3,p)); ...
%         %un(2)]; %estado atualizado pela entrada
%         %xP(3,p) = wrapToPi(xP(3,p));
        xP(:,p) = samplevmotion(u, xP(:,p), alpha, t_s, 1);
%xt = samplevmotion(ut, xs, alpha, dt, n)
% ut    - control at time t (linear and angular velocities)
% xs    - state at time t-1
% alpha - noise coefficients
% dt    - time step duration
% n     - the number of samples to generate  ==>1!
%
% xt    - a 3xn matrix of sampled states (each column representing
%         a separate sample)
        
        
        
        
    end

    % Correção
%    for p = 1:n_Particulas
%         % Estima mediçao para a partícula
%         %z = [];
%         dist=zeros(num_referencias,1);
%         for m = 1:num_referencias
%             dist(m) = sqrt((marker(m,1)-xP(1,p))^2 + (marker(m,2)-xP(2,p))^2);
%         end %distancia dos marcadores
%         Innov = dist_medida - dist; % Inovação
%         % Pesos e Matriz de Covariância
%         RR = diag(repmat(diag(R), size(marker, 1), 1));
%         W(p) = exp(-0.5*Innov.'/(RR)*Innov) + 0.0001;
        %M = [0 0; 0 0; 0 0; 0 0]; 
        fi = [u(3:(3+num_referencias-1))'...
                 ;u((3+num_referencias):(3+2*num_referencias-1))'...
                 ;u((3+2*num_referencias):end)']';
             
        for p = 1:n_Particulas
            pp=1;
            for ci = 1:num_referencias
    %             %ci = [1 2];
    %             fi = [u(3:(3+num_referencias-1))'...
    %             ; u((3+num_referencias):(3+2*num_referencias-1))'...
    %             ;u((3+2*num_referencias):end)']';
                  %fi = [u(3+ci-1);u(3+num_referencias+ci-1);u(3+2*num_referencias+ci-1)]';
                  pp = pp.*landmark( fi, ci,xP(:,p), marker);
            end
            W(p) = pp;
        end
        %W = pp;
        %www = landmark( fi, ci,xP, marker);
%    end
    w_avg = sum(W);
    W = W/sum(W) +(eps*1e3)*ones(size(W));
%     w_slow=x(1);
%     w_fast=x(2);
%     w_slow=w_slow + a_slow*(w_avg-w_slow);
%     w_fast=w_fast + a_fast*(w_avg-w_fast);
    iNextGeneration = obtainNextGenerationOfParticles(W, n_Particulas);
    xP = xP(:,iNextGeneration);
    
%     num_aleat = n_Particulas*max(0,1-w_fast/w_slow);
%     num_aleat =min(round(num_aleat),n_Particulas);
%     if num_aleat>0
%      xP(:,(n_Particulas-num_aleat+1):n_Particulas) =  repmat(x0a', 1, num_aleat) + P*randn(3, num_aleat);
%     end
    


    sys = [reshape(xP,1,[])];
  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,  
      %x_ = x(3:end);
      
      [gg, ggi] = max(W);
      %x_ = x_(ggi*3-2:ggi*3);
      x_ = x(ggi*3-2:ggi*3);
      
      %sys = [x(1); x(2);x(3:end) ;x_];
      sys = [0; 0;x(1:end) ;x_];
      
      
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
