function [sys,x0,str,ts] = s_modelo_new(t,x,u,flag, x0a)

t_s=.1;
global marker
marker = [0 0;5 5]; % Positions of markers
num_referencias = size(marker, 1);
switch flag,

  case 0,
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 3;
        sizes.NumOutputs     = 3+3*num_referencias;
        sizes.NumInputs      = 4;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0 =[x0a ]; 
        str = [];
        ts  = [t_s 0]; 
  case 2,  
    %u = [0.5; 0.2]; % Movement command (translational and angular velocity)
    %u_sum = u + sqrt(Q)*randn(2, 1)*enableNoise;

    % Simulation of the true mobile robot state (pose)
    x_n = x + t_s*[u(1)*cos(x(3)); ...
                 u(1)*sin(x(3)); ...
                 u(2)];
             
    x_n(3) = (x_n(3));
    sys =[x_n'];  
  case 3,    
      %saidas - posiçoes reais e distancia medida
    dist = zeros(num_referencias,1);
    ang = zeros(num_referencias,1);
    id = zeros(num_referencias,1);
    x(1:2) = x(1:2)+u(3:4); %offset (robô sequestrado)
    for m = 1:num_referencias
        dist(m) = sqrt((marker(m,1)-x(1))^2 + (marker(m,2)-x(2))^2);
        ang(m) = atan2((marker(m,1)-x(1)),(marker(m,2)-x(2)))-x(3);
        id(m) = m;
    end
    sys = [x(:)' dist(:)' ang(:)' id(:)'];
  case 9,                                                
    sys = []; % do nothing
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end