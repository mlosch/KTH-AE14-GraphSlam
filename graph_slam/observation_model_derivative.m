% function h = observation_model_derivative
% This function is the implementation of the derivative of the h function.
% The bearing should lie in the interval [-pi,pi)
% Inputs:
%           pose                3X1
%           wall                4X1
%           sensor_offsets      2x1 IR Sensor offsets (x,y)
%           sensor_alignment    1x1 IR Sensor heading relative to robot
%           (S_theta)
% Outputs:  
%           dh/d[x,y,Theta,N,c] 5X1 

function dh = observation_model_derivative(pose, wall, sensor_offsets, sensor_alignment)
	
    dh = zeros(1,5);
	
    x = pose(1,1);
    y = pose(2,1);
    theta = pose(3,1);
    
    N = wall(1,1);
    c = wall(2,1);
    
    sx = offsets(1,1);
    sy = offsets(2,1);
    stheta = sensor_alignment;
    
    cosTheta = cos(theta);
    sinTheta = sin(theta);
    cosN = cos(N);
    sinN = sin(N);
    
    %dh/dx
    dh(1,1) = cosN;
    %dh/dy
    dh(1,2) = sinN;
    %dh/dtheta
    dh(1,3) = sy*sinN*cosTheta - sx*cosN*sinTheta - tan(N-stheta-theta) * ...
        (cosN*(sx*cosTheta+x) + sinTheta*(sy*sinTheta+y) - c);
    %dh/dN
    dh(1,4) = tan(N-stheta-theta) * (cosN*(sx*cosTheta+x) + sinN*(sy*sinTheta+y) - c) + ...
        cosN*(sy*sinTheta+y) - sinN*(sx*cosTheta+x);
    
    %dh/dc
    dh(1,5) = -1;
    
    dh = dh / cos(N - stheta - theta);
    
end