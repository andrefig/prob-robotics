clear all



global z_hit z_short z_max z_rand sigma lambda tamanho
    z_hit = 0.7;
    z_short = 0.14;
    z_rand = 0.16;
    %z_max = 0.0; %Vai depender do mapa!!
    
    sigma = 0.6;
    lambda = 4;
    
    tamanho = 50;       %dimensão do mapa
    range_max =  20;     %alcance do sensor

    angle_step = pi/2;  %ângulo entre as medidas
    pos = [35 23 0];
%pos = [floor(3*tamanho/7) floor(1*tamanho/2) pi/2];     %posição do robô
%pos = [pos(2) tamanho-pos(1) pos(3)];
%cria mapa:
A =zeros(tamanho);
%A(tamanho,:)=1;
A(1,:)=1;
A(tamanho,:)=1;
A(:,1)=1;
A(:,tamanho)=1;
A(floor(tamanho/6):floor(tamanho/5),1:floor(tamanho/4))=1;
A((floor(tamanho/2)-1):(floor(tamanho/2)+1),(floor(tamanho/2)-1):(floor(tamanho/2)+1))=1;

B=A;
B(11:12,10:11)=1;

%A(floor(tamanho/2 +tamanho/6):floor(tamanho/2 +tamanho/5),1:floor(tamanho/4))=1;

%A(1:floor(tamanho/2),floor(3*tamanho/4):tamanho)=1;


%A = flip(A);
map = occupancyMap(flip(A,1)); 
pos_map =[pos(2) tamanho-pos(1)  pos(3)];
subplot(2,2,3);
hold off
show(map)
    axis on
    axis 'xy'
hold on

subplot(2,2,1);
imshow((A), []);
axis on
axis 'xy'
hold on
plot(pos(1),pos(2),'wo')
grid on
hold on

x_sens =0;
y_sens=0;
%p_varr = zeros(8,2);
%f=@(x)[x^2];
p_varr =@(x,y,d,i,tam) ...
                   [x+i-1 y+d ;...  %r*
                    x+d y-i+1 ;...  %r*
                    x-d y+i-1 ;...  %m*
                    x-i+1 y-d ;...  %c*  ------                
                    x-i y+d ;...    %go
                    x+i y-d ;...    %bo
                    x+d y+i ;...    %yo
                    x-d y-i ];      %bo
p_varr_ = zeros(8,2);
%occ = zeros(8,1);
%x=pos(1);
%y=pos(2);
%d_min = -1;
D_min =-1*ones(tamanho);

%calcula menores distâncias
%algoritmo norma-manhattan

for x = 1:tamanho
   percent_ger = 100*x/tamanho
   for y=1:tamanho %varre todos os pontos
        manh=2*tamanho;  %distância manhattan
        d_min=2*tamanho; %distância euclidiana
%         subplot(2,1,1);
%         hold off
%         imshow((A), []);
%         axis on
%         axis 'xy'
%         hold on
        for d_m=0:(-1+(tamanho)/2) %distância do centro para os lados
           %   <-1a2->^
           %   ^      8
           %   3      d
           %   b      7
           %   4      v
           %   v<-5c6->
            occupancy=A(x,y);
            if occupancy %o ponto em si está ocupado
                d_min = 0;
                break;
            end
            if d_m^2>=d_min 
                %distância do centro ao lado é maior 
                %que a distância já encontrada
                break;
            end
           for i=1:d_m
                %manhattan = d_m+i
                if d_m^2+i^2>=d_min
                    break;
                    %distância do ponto varrido é maior
                    %que a distância mh já encontrada
                end
                p_varr_ = p_varr(x,y,d_m,i); 
                for j=1:8
%                     plot(p_varr_(1,1),p_varr_(1,2),'r*')
%                     plot(p_varr_(2,1),p_varr_(2,2),'r*')
% 
%                     plot(p_varr_(3,1),p_varr_(3,2),'m*')
%                     plot(p_varr_(4,1),p_varr_(4,2),'c*')
% 
%                     plot(p_varr_(5,1),p_varr_(5,2),'go')
%                     plot(p_varr_(6,1),p_varr_(6,2),'bo')
% 
%                     plot(p_varr_(7,1),p_varr_(7,2),'yo')
%                     plot(p_varr_(8,1),p_varr_(8,2),'bo')
%                     
                    aux = min(max(p_varr_(j,:),1),tamanho);
                    occupancy=A(aux(1),aux(2));
                    
                    if occupancy
                        aux = [x y]- aux;
                        manh = sum(aux);
                        d_min= min((sum(aux.^2)),d_min);
                        break;
                    end             
                end
           end
        end
        D_min(x,y)=sqrt(d_min);
   end
end


subplot(2,2,2);
hold off
imshow((D_min), []);
axis on
axis 'xy'
hold on
grid on


% % 
% % %exibe robô:
% % 
subplot(2,2,3);
hold on
quiver(pos(1),pos(2),sin(pos(3)),cos(pos(3)),1,'Color','b','LineWidth',1,'MaxHeadSize',1);
hold on
grid on
% %     
% % 
z = zeros(size(angle_step:angle_step:2*pi,2),1);
P=zeros(tamanho);
% Realiza medidas:
% %    subplot(2,2,3) 
% % 
% %     imshow(flip(~B,1), []);
% %     grid on
% %     axis on
% %     axis 'xy'
% %     
% % hold on
%pos_map =[tamanho-pos(1) tamanho-pos(2) pos(3)];

[z,ang] = beam_range_readings(z,pos_map,map,range_max,1,tamanho);

%[Z(i),tipo(i)]=sample_reading(range,range_max);
     %Calcula probabilidade:
%     beam_range_finder(z,pos,map,range_max,tamanho)
P =zeros(tamanho);
 %varre mapa
%  pos_x=pos(1);
%  pos_y=pos(2);
for pos_x=1:tamanho
        for pos_y = 1:tamanho
            q=1;
             %varrer as medidas...
              for k=1:size(z,1)
                  if z(k)<range_max
                      x_k= round(pos_x+z(k)*cos(ang(k)+pi/2));
                      y_k= round(pos_y+z(k)*sin(ang(k)+pi/2));
                      if ((pos(1)==pos_x)&&(pos(2)==pos_y))
                            subplot(2,2,1)
                            hold on
                            plot(x_k,y_k,'c*');
                      end
                      if ((x_k>tamanho)||(x_k<1))||((y_k>tamanho)||(y_k<1))
                          q=+z_rand/range_max;
                      else
%                           x_k=round(min(max(x_k,1),tamanho));
%                           y_k=round(min(max(y_k,1),tamanho));
                            q = q*(z_hit*normpdf(D_min(x_k,y_k),0,sigma)+z_rand/range_max);
                      end
                          
                      
                  end
                  
              end
              P(pos_x,pos_y)=q;
        end
        
        percent = 100*pos_x/tamanho
end
    hold off
    subplot(3,3,9);
    subplot(2,2,4);
    imshow(P', []);
    axis on
    grid on
    axis 'xy'
    



