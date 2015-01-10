% function [ys, xs] = o()
% Inputs:
%           t   1X1
%           j   1x1
%           n   1x1 number of desired rows
%           m   1x1 number of desired columns
% Outputs:  
%           list of row and column indices
function [ys, xs] = o(t, j, n, m)

    a = t*3-2 + j*2-1;
    ys = a:(a + n);
    xs = a:(a + m);
end
