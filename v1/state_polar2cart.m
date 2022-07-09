function [x, y, vx, vy] = state_polar2cart(theta, u, R)
    vx = cos(theta-pi).*u;
    vy = sin(theta-pi).*u;   
    x = cos(theta).*R;
    y = sin(theta).*R;    
end


