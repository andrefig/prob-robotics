function [x,P]=ukf(fstate,x,P,hmeas,z,R,Q)
n=numel(x);                                 %Número de estados
m=numel(z);                                 %Número de medidas
alpha=1e-3;                                
ki=0;                                       
beta=2;                                     
lambda=alpha^2*(n+ki)-n;                    
c=n+lambda;                                 
Wm=[lambda/c 0.5/c+zeros(1,2*n)];           %pesos para médias
Wc=Wm;
Wc(1)=Wc(1)+(1-alpha^2+beta);               %pesos para matr. cov.
c=sqrt(c);
X=sigmas(x,P,c);                            %sigma points
[x1,X1,P1,X2]=ut(fstate,X,Wm,Wc,n,R);       %Predição (1 e 2)- nova variância e média
% X1 %sigma points de x1
% X2 desvios de X1 em torno da média
[z1,Z1,P2,Z2]=ut(hmeas,X1,Wm,Wc,m,Q);       %Correção (3)
P12=X2*diag(Wc)*Z2';                        %
K=P12/(P2);                                 %4 - Ganho de Kalman
x=x1+K*(z-z1);                              %5 - Corrige média
P=P1-K*P12';                                %6 - Corrige matr. covariância

function [y,Y,P,Y1]=ut(f,X,Wm,Wc,n,R)
L=size(X,2);
y=zeros(n,1);
Y=zeros(n,L);
for k=1:L                   
    Y(:,k)=f(X(:,k));   %calcula f(x) para os sigma-points      
    y=y+Wm(k)*Y(:,k);   %calcula valor médio por média ponderada
end
Y1=Y-y(:,ones(1,L));
P=Y1*diag(Wc)*Y1'+R;    %calcula matr. cov. por média ponderada       



function X=sigmas(x,P,c)
A = c*chol(P)';
Y = x(:,ones(1,numel(x)));  %replica valores de x para os sigma-points
X = [x Y+A Y-A];            %calcula sigma-points