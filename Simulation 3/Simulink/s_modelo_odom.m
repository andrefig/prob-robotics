function [sys,x0,str,ts] = s_modelo(t,x,u,flag, x0a)

t_s=1;
switch flag,

  case 0,
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 6;
        sizes.NumOutputs     = 3;
        sizes.NumInputs      = 2;
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
    x_n(4:6)=x(1:3);
    x_n(1:3) = x + t_s*[u(1)*cos(x(3)); ...
                 u(1)*sin(x(3)); ...
                 u(2)];
             
    x_n(3) = wrapToPi(x_n(3));
    
    sys =[x_n'];  
  case 3,    
      %saidas - posiçoes reais e distancia medida
             
    sys = [x];
  case 9,                                                
    sys = []; % do nothing
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end