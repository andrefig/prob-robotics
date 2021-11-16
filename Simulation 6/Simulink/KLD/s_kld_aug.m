function [sys,x0,str,ts] = s_kld(t,x,u,flag, x0a)
% eixos = [-10 10 -10 10];
P = diag([10 10 5]); 
alpha = [1 1 1 1 0.1 0.1] * 1e-1; 

a_slow= 0.05;
a_fast=0.055;

e =5e-2;
z_1_d = 2;

M_min =10;
n_Particulas = 20000;

global rerro
rerro= 0.5;
global phierro
phierro = 0.5;
global sierro
sierro = 0.1;




n_estados =  n_Particulas*3;
%n_estados2 =  n_Particulas*4+1;


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
        sizes.NumDiscStates  = n_estados+6;
        sizes.NumOutputs     = n_estados+5;
        sizes.NumInputs      = 2+3*num_referencias;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        %particulas = repmat(x0a', 1, n_Particulas) + P*randn(3, n_Particulas); % Gera partículas aleatórias
        particulas = repmat(x0a', 1, n_Particulas) + 2*P*rand(3, n_Particulas) - P*ones(3, n_Particulas); % Gera partículas aleatórias
        %x0 = [0; 0; reshape(particulas,[],1)];
        W = ones(n_Particulas, 1)/n_Particulas;
        x0 = [0;0;reshape(particulas,[],1);0;0;0;n_Particulas];
        str = [];
        ts  = [t_s 0]; 

% end mdlInitializeSizes

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,  
%     xP = reshape(x(1:n_estados),3,[]);
%     W = x((n_estados+1):(n_estados2-1));
%     M = x(end);
    %M_ant = x(end);
    %xP = reshape(x(1:3*M),3,[]);
    xP = reshape(x(3:(n_estados+2)),3,[]);
    %W = x((n_estados+1):(n_estados+M_ant));
    
    
    xP_bins = [round(xP(1:2,:),0) ; round(xP(3,:),1)];
    k = size(unique(xP_bins','rows'),1);
    
    M = round(((k-1)/(2*e))*(1-2/(9*(k-1))+z_1_d*sqrt(2/(9*(k-1))))^3);
    M = min(max(M,M_min),n_Particulas);
    
    
    
    for p = 1:M%!!!!!!!!!!!!!!
        xP(:,p) = samplevmotion(u, xP(:,p), alpha, t_s, 1);
    end
    % Correção
        fi = [u(3:(3+num_referencias-1))'...
                 ;u((3+num_referencias):(3+2*num_referencias-1))'...
                 ;u((3+2*num_referencias):end)']';
        for p = 1:M
            pp=1;
            for ci = 1:num_referencias
                  pp = pp.*landmark( fi, ci,xP(:,p), marker);
            end
            W(p) = pp;
        end

    w_avg = sum(W);
    %W = W/sum(W) +(eps*1e3)*ones(size(W));
     w_slow=x(1);
     w_fast=x(2);


    
    W_norm = W(1:M)/sum(W(1:M)) +(eps*1e3)*ones(size(W(1:M)));
    iNextGeneration = obtainNextGenerationOfParticles(W_norm',M);
    xP = xP(:,iNextGeneration);
    
    num_aleat = M*max(0,1-w_fast/w_slow);
    num_aleat =min(round(num_aleat),M);
    if num_aleat>0
     xP(:,(M-num_aleat+1):M) =  repmat(x0a', 1, num_aleat) + P*randn(3, num_aleat);
    end
      
     w_slow=w_slow + a_slow*(w_avg-w_slow);
     w_fast=w_fast + a_fast*(w_avg-w_fast);

     
    xxx = reshape(xP,1,[]);
    xxx= padarray(xxx' ,n_estados-size(xxx,2),'post');
    
    %WW = padarray(W' ,n_Particulas-size(W,2),'post'); 
    
    [gg, ggi] = max(W);
    x_ = xxx(ggi*3-2:ggi*3);
    
    sys = [w_slow w_fast xxx' x_' M];
  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,  
      M = x(end); 
      x_trunc = x(3:(3*M+2));
          
      %W = x((n_estados+1):(n_estados2-1));
      %W = W(1:M);

      xxx= padarray(x_trunc ,n_estados-size(x_trunc,1),'post');
      x_ = x((end-3):(end-1));
      
      sys = [M;0;xxx ;x_];
      
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
