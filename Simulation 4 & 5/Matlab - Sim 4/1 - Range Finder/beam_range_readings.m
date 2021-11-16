
function [z angle] = beam_range_readings(z,pos,map,range_max,plota,tamanho)

    angle_step = 2*pi/size(z,1);
    %range_max=45;
    z = zeros(1,size(angle_step:angle_step:2*pi,2));
    angle = zeros(1,size(angle_step:angle_step:2*pi,2));
    k=1;
    for ang = angle_step:angle_step:2*pi
            [z(k), aux1,~,index] = rangefinder(pos+[0 0 ang],map,range_max,tamanho);
            angle(k) =ang;
            
            if plota
%             subplot(2,2,1)
%             plot(aux1(:,1),aux1(:,2),'ro')
%             plot(aux1(index,1),aux1(index,2),'go')
%             %dist = sqrt((aux1(index,1)-pos(2))^2+(aux1(index,2)-pos(1))^2);
%             dist = z(k);
%             %hold on
                subplot(2,2,3)          
                plot(tamanho-pos(2),pos(1),'r*')
                hold on
                plot([tamanho-pos(2) tamanho-pos(2)+z(k)*sin(-ang)],[pos(1) pos(1)+z(k)*cos(-ang)],'r-')
                %plot(aux1(:,1),aux1(:,2),'ro')
                plot(aux1(index,1),aux1(index,2),'go')
                %hold on
                subplot(2,2,4);
                hold on
                plot(k,z(k),'bo');
                hold on
                axis([1 size(z,2) 0 range_max]);
                subplot(2,2,3);
                
            end
            k=k+1;
    end
    z=z';
    %subplot(2,2,3)
    %hold off
    %plot(z'.*sin(angle),z'.*cos(angle) ,'k.')
end
