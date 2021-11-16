function [sys,x0,str,ts] = dsfunc_s(t,x,u,flag, x0a)

t_s=0.01;

switch flag,

  case 0,
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 2;
        sizes.NumOutputs     = 2;
        sizes.NumInputs      = 2;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0 =[x0a ]; 
        str = [];
        ts  = [t_s 0]; 
  case 2,  
  % 
    f=@(x)[x(1)+cos(u(2))*(u(1)^2)*t_s ; x(2)-sin(u(2))*(u(1)^2)*t_s  ];  
    sys =[f(x)'];  
  case 3,           
    sys = [x(:)];
  case 9,                                                
    sys = []; % do nothing
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end