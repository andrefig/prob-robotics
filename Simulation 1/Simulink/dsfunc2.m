function [sys,x0,str,ts] = dsfunc2(t,x,u,flag, x0a)
 
r=1;  
Q=[1 0; 0 0.1];
R=r*eye(2);         
t_s=1;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,

        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 6;
        sizes.NumOutputs     = 6;
        sizes.NumInputs      = 4;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0 =[x0a ]; 
        str = [];
        ts  = [t_s 0]; 

% end mdlInitializeSizes

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,  
  % 
    P=[x(3) x(4); x(5) x(6)];
    f=@(x)[x(1)+cos(u(2))*(u(1)^2)*t_s ; x(2)-sin(u(2))*(u(1)^2)*t_s  ];   
    z=u(3:4); 
    h=@(x)[x(1); x(2)];      
    [a,P]=ukf2(f,[x(1:2)],P,h,z,Q,R);
    sys =[a' P(1,:) P(2,:)];  
  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,           
    sys = [x(:)];

  %%%%%%%%%%%%%
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