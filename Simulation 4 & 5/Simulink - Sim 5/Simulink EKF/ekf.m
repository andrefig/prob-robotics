function [x,P]=ekf(fstate,x,P,hmeas,z,R,Q)
                            %Predição
[x1,A]=jaccsd(fstate,x);    %1 - Atualiza média, calcula Jacobiano de g(x)
P=A*P*A'+R;                 %2 - Atualiza  matr. covariância 
                            %Correção
[z1,H]=jaccsd(hmeas,x1);    %3 - Calcula Jacobiano de h(x)
P12=P*H';                   %
K=P12/(H*P12+Q);            %4 - Ganho de Kalman
x=x1+K*(z-z1);              %5 - Corrige média
P=P-K*P12';                 %6 - Corrige matr. covariância

function [z,A]=jaccsd(fun,x)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,x)
% z = f(x)
% J = f'(x)
%
z=fun(x);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x;
    x1(k)=x1(k)+h*1i;
    A(:,k)=imag(fun(x1))/h;
end