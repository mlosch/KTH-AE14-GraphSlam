function [row col] = get_array_idx(i, n_per_row)
%member function of segment_walls
    row = floor(((i-1)/n_per_row))+1;
    col = mod(i-1,n_per_row)+1;
    
    if (n_per_row < 6)
        col = col + 6-n_per_row;
    end

end