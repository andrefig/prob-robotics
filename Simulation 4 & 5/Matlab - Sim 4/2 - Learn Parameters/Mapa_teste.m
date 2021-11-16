clear all

global z_hit z_short z_max z_rand sigma lambda tamanho
    z_hit = 0.7;
    z_short = 0.14;
    z_rand = 0.16;
    %z_max = 0.0; %Vai depender do mapa!!
    
    sigma = 0.6;
    lambda = 4;
    
    tamanho = 30;       %dimensão do mapa
    range_max = 25;     %alcance do sensor

sigma_a = 1.2;
lambda_a=1;
z_hit_a  =0.25;
z_short_a =0.25;
z_rand_a =0.25;
z_max_a =0.25;

eps_parada=1e-3;
num_iter=100;  
num_samples = 8000;




%Mapa:
A =zeros(tamanho);
A(1,:)=1;
A(tamanho,:)=1;
A(:,1)=1;
A(:,tamanho)=1;
A(floor(tamanho/6):floor(tamanho/5),1:floor(tamanho/4))=1;
A((floor(tamanho/2)-1):(floor(tamanho/2)+1),(floor(tamanho/2)-1):(floor(tamanho/2)+1))=1;
map = occupancyMap(A); 


% Gera posições e amostras:
a=0;
Z = zeros(1,num_samples);
Z_=Z;
tipo=Z;
for i = 1:num_samples
    pos = [1+ rand()*(tamanho-1) 1+rand()*(tamanho-1) 2*pi*rand()];
    [range,~,~,~] = rangefinder(pos,map,tamanho*1.5);
    [Z(i),tipo(i)]=sample_reading(range,range_max);
    Z_(i) = range;
    if range>=range_max
        a=a+1;
    end
    sample_perc =100*i/num_samples
end


normaliza = size(tipo,2);
z_hit = sum(tipo==1)/normaliza;
z_short = sum(tipo==2)/normaliza;
z_rand = sum(tipo==3)/normaliza;
z_max = sum(tipo==4)/normaliza;

alvos = [z_hit,z_short,z_rand,z_max]

pesos = [z_hit_a ,z_short_a, z_rand_a, z_max_a];
e = zeros(num_samples,4);

result(1,:) = [pesos sigma_a 1/lambda_a];

for jj=1:num_iter
    for i=1:num_samples
        [p_hit ,p_short, p_rand, p_max] = probs(Z(i),Z_(i),range_max,sigma_a,lambda_a);
        p = [p_hit ,p_short, p_rand, p_max];
        p=p.*pesos;
        gama = 1/sum(p);
        e(i,:) = gama*p;
    end
    
    pesos = sum(e)/norm(Z);
    pesos =pesos/sum(pesos);
    sigma_a= sqrt((1/sum(e(:,1)))*sum(e(:,1).*(Z(:) - Z_(:)).^2));
    lambda_a =sum(e(:,2))/sum(e(:,2).*Z(:));
    result(jj+1,:)=[pesos sigma_a 1/lambda_a];
    crit_parada =100*eps_parada/(norm(result(jj+1,:)-result(jj,:)))
    if crit_parada>=100
        break;
    end
    percent =100*jj/num_iter
    %pesos = pesos_new
end
result(end,:) = [alvos sigma 1/lambda];


l = size(result,1);
newcolors = {'#F00','#F80','#0B0','#00F','#50F','#A0F'};
colororder(newcolors);
plot(1:(l-1),result(1:end-1,:),'-');
hold on
colororder(newcolors);
plot(l:(l+1),[result(end,:);result(end,:)],'o-');
hold off
% plot(size(result,1)*ones(size(result,2))',result(end,:)','*');
% hold off