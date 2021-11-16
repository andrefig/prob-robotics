clear all
subplot(1,1,1);
hold off

tamanho = 30;       %dimensão do mapa
range_max = 10;     %alcance do sensor
angle_step = 2*pi/3;  %ângulo entre as medidas
pos = [15 20 0]
%pos = [floor(3*tamanho/7) floor(1*tamanho/2) pi/2];     %posição do robô
pos = [pos(2) tamanho-pos(1) pos(3)];
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
map = occupancyMap(A); 

mapB = occupancyMap(B); 
subplot(2,2,1);
hold off
show(map)
    axis on
    axis 'xy'
hold on

%exibe robô:

subplot(2,2,1);
quiver(tamanho-pos(2),pos(1),-sin(pos(3)),cos(pos(3)),1,'Color','b','LineWidth',1,'MaxHeadSize',1);
hold on
grid on
    

z = zeros(size(angle_step:angle_step:2*pi,2),1);
P=zeros(tamanho);
    % Realiza medidas:
    subplot(2,2,3) 

    imshow(flip(~B,1), []);
    grid on
    axis on
    axis 'xy'
    
hold on
    [z ang] = beam_range_readings(z,pos,mapB,range_max,1,tamanho);
    %Calcula probabilidade:
    beam_range_finder(z,pos,map,range_max,tamanho)

%varre mapa
for pos_x=1:tamanho
        for pos_y = 1:tamanho
            z = beam_range_readings(z,[pos_x pos_y pos(3)],map,range_max,0,tamanho);
            P(pos_x,pos_y) =  beam_range_finder(z,pos,map,range_max,tamanho);
%             subplot(2,2,3);
%             axis([0 tamanho 0 tamanho]);
%             hold off
        end
        percent = 100*pos_x/tamanho
end
    hold off
    subplot(2,2,2);
    imshow((flip(P,2)), []);
    hold on
    plot(tamanho-pos(2)+1,pos(1),'go')
    grid on
    axis on
    axis 'xy'
    



