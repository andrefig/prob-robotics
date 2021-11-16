function [sys,x0,str,ts] = dsfunc_unkn(t,x,u,flag, x0a)
 
%Q=[1 0; 0 0.1];
%R=[100 0; 0 10*pi/360];    
alfa = [0.01 0.01 0.01 0.01];
delta = [0.1 0.1];
t_s=0.5;
global marker;
%marker = [0 0; 5 5]; % Positions of markers
num_referencias = size(marker, 1);

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,

        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 3+9+1;
        sizes.NumOutputs     = 3+9+1;
        sizes.NumInputs      = 2+ 3+3*num_referencias;
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
    P=[x(4) x(5) x(6); x(7) x(8) x(9); x(10) x(11) x(12)];
    %[v(1,2) x(1,2,3)' dist(:)' ang(:)' id(:)'];
    v = u(1:2);
    th =x(3);
    dist = u(6:(6+num_referencias-1));
    ang = u((6+num_referencias):(6+2*num_referencias-1));

    M = [alfa(1)*v(1)^2+alfa(2)*v(2)^2 0;...
         0   alfa(3)*v(1)^2+alfa(4)*v(2)^2];
     
    Q = diag(delta.^2);
    
    x_augm = [x(1:3)' 0 0 0 0]; 
    
    P_augm = [P         zeros(3,2)   zeros(3,2);...
              zeros(2,3)     M       zeros(2,2); ...
              zeros(2,3) zeros(2,2)     Q];
   %xv = [x y th v w]
   x_augv = x_augm + [0 0 0 v(1) v(2) 0 0];
   
   g =@(xv) xv +...
[-(xv(4)/xv(5))*sin(xv(3))+(xv(4)/xv(5))*sin(xv(3)+xv(5)*t_s);...
 (xv(4)/xv(5))*cos(xv(3))-(xv(4)/xv(5))*cos(xv(3)+xv(5)*t_s);...     
  xv(5)*t_s;...
  0;0;0;0]; %função de transição de estados
   
  %[mu,~,sigma,~]=utt(g,xv,P_augm);
  %xz = x_augm;
  h =@(xz) [xz(6);xz(7)] +...
 [sqrt((marker(1,1)-xz(1))^2 + (marker(1,2)-xz(2))^2);...
  atan2((marker(1,1)-xz(1)),(marker(1,2)-x(2)))-xz(3)...
 ];

 %[z_hat,~,sigma_xz,~]=utt(h,x_augm,P_augm);
 z = [dist ang];
 [xx,PP]=ukf(g,x_augv',P_augm,h,z');

 sys =[xx(1:3)' PP(1,1:3) PP(2,1:3) PP(3,1:3) 1];   
  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,           
    sys = [x];

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