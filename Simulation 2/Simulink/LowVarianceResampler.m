function iNextGeneration = LowVarianceResampler(weight, state_size)
%function [state] = lowVarianceRS(prev_state, weight, state_size)
    %state = zeros(1,state_size);    % Initialize empty final state
    r = rand()/state_size;                       % Select random number between 0-1
    weight(:)=weight(:)/sum(weight(:));
    w = weight(1);                  % Initial weight
    i = 1;
    j = 1;
    iNextGeneration = zeros(1,state_size);
    for m = 1:state_size
        U = r + (m - 1)/state_size; % Index of original sample + size^-1
        while  w< U                 % I'm not sure what this loop is doing
            i = i + 1;
            if i<=state_size
                w = w + weight(i);
            else
                break;
            end
        end
        iNextGeneration(j) = min(max(i,1),state_size);   % Add selected sample to resampled array
        j = j + 1;
    end
%end
end

