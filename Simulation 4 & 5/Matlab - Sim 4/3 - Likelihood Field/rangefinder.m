function [dist,aux1,aux2,index] = rangefinder(pos,map,range_max,tamanho)
        [endpoints,midpoints] = raycast(map,pos,range_max*10,0);
        midpoints = [midpoints ; endpoints];
        [row,~] = find(checkOccupancy(map,midpoints));
        %if size(row)>0
        aux1 = midpoints( row, :);
        %else
        %    aux1 = endpoints( 1, :);
        %end
        pos(:) = [tamanho-pos(2) pos(1) pos(3)];
        aux2 = sum((aux1-(pos(1:2))).^2,2);
        [range,index]=min(aux2);
%                 plot(tamanho-pos(2),pos(1),'y*')
%                 plot(aux1(index,1),aux1(index,2),'go')
        x = aux1(index,1);
        y = aux1(index,2);
        %dist=min(range);
        if size(range,1)>0
            dist=sqrt(range);
            if dist>range_max
                dist=range_max;
            %aux1 = endpoints( 1, :);
            end
        else
            dist=range_max;
        end
end