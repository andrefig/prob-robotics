function [sys,x0,str,ts] = dsfunc3(t,x,u,flag, x0a)
 
%Q=[1 0; 0 0.1];
%R=[100 0; 0 10*pi/360];    
alfa = [0.001 0.001 0.001 0.001];
delta = [0.1 0.1 0.1];
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
    id = u((6+2*num_referencias):(6+3*num_referencias-1));
    G = [1 0 -(v(1)/v(2))*cos(th) + (v(1)/v(2))*cos(th+v(2)*t_s);...
         0 1 -(v(1)/v(2))*sin(th) + (v(1)/v(2))*sin(th+v(2)*t_s);...
         0 0    1];
    V = [(-sin(th)+sin(th+v(2)*t_s))/v(2) v(1)*(sin(th)-sin(th+v(2)*t_s))/v(2)^2 + v(1)*cos(th+v(2)*t_s)*t_s/v(2);...
          (cos(th)-cos(th+v(2)*t_s))/v(2 ) -v(1)*(cos(th)-cos(th+v(2)*t_s))/v(2)^2 + v(1)*sin(th+v(2)*t_s)*t_s/v(2);...
          0     t_s];
    M = [alfa(1)*v(1)^2+alfa(2)*v(2)^2 0;...
         0   alfa(3)*v(1)^2+alfa(4)*v(2)^2];
     
    x_ = x(1:3)+ ...
         [ -(v(1)/v(2))*sin(th)+(v(1)/v(2))*sin(th+v(2)*t_s);...
            (v(1)/v(2))*cos(th)-(v(1)/v(2))*cos(th+v(2)*t_s);...
             v(2)*t_s];
         
    P = G*P*(G') + V*M*(V');
      
    Q = diag(delta.^2);
          
    for i = 1:num_referencias
        dist_ = sqrt((marker(i,1)-x(1))^2 + (marker(i,2)-x(2))^2);
        ang_ = atan2((marker(i,1)-x(1)),(marker(i,2)-x(2)))-th;
        z_hat(:,i) = [dist_; ang_;i];
        z(:,i) = [dist(i); ang(i);id(i)];
        
        H = [-(marker(i,1)-x(1))/dist_ -(marker(i,2)-x(2))/dist_ 0; ...
              (marker(i,2)-x(2))/dist_ -(marker(i,1)-x(1))/dist_ -1;...
              0                  0           0];
          
        S(:,:,i) = H*P*(H') + Q;
        K = P*(H')/S(:,:,i);
        x_ = x_+ K*(z(:,i)-z_hat(:,i));
        P = (eye(size(P)) - K*H)*P;
    end
    pz=1;
    for i = 1:num_referencias
        pz = (pz/sqrt(det(2*pi*S(:,:,i))))*...
            exp(-0.5*(z(:,i)-z_hat(:,i))'/(S(:,:,i))*(z(:,i)-z_hat(:,i)));
        
    end
    
    
    %f=@(x)[x(1)+cos(u(2))*(u(1))*t_s ; x(2)-sin(u(2))*(u(1))*t_s  ];   
    %z=u(3:4);
    %h=@(x)[sqrt(x(1)^2+x(2)^2) angle(x(1)+1i*x(2))]
    %[a,P]=ekf(f,[x(1:2)],P,h,z,Q,R);
    sys =[x_(1:3)' P(1,:) P(2,:) P(3,:) pz];   
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