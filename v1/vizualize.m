% close all
% constants 
deltaT = 0.1;
Radius = 4;

cr = load("data/CR.txt");
u0 = load("data/U0.txt");
r0 = load("data/R0.txt");

obst_x0 = load("data/OBST_X0.txt");
obst_y0 = load("data/OBST_Y0.txt");
obst_vx0 = load("data/OBST_VX0.txt");
obst_vy0 = load("data/OBST_VY0.txt");


% initialize actions
Ar = -1:0.2:1;
Au = -1:0.2:1;
% Ar = -1;
% Au = -1;



% main simulation loop
for i = 4:length(U0)    
   figure

for k = 1:length(Ar)
    for l = 1:length(Au)
        
        % intitialize state
        x = 0; y = 0; theta = 0; u = u0(i); v = 0; r = r0(i); 
        x_obst = obst_x0(i,:); y_obst = obst_y0(i,:); vx_obst = obst_vx0(i,:); vy_obst = obst_vy0(i,:); 
        
%         u = 1;
%         r = 0.1;
        
        % initialize plotting
        % close all

        plots(k*l) = plot(x,y);
        plots(k*l).Color = 'blue';
        hold on 


        % main trajectory loop
        done = false;
        ii = 0;
        distance = Inf;
        while ~done
            ii = ii+1;
            [x,y,theta,u,v,r] = simulateTimeStep(x,y,theta,u,v,r,Ar(k),Au(l));
            
            x_obst = x_obst + vx_obst * deltaT;
            y_obst = y_obst + vy_obst * deltaT;

            % find min distance to obstacle
            distance_old = distance;
            distance = sqrt((x-x_obst).^2 + (y-y_obst).^2);
            
%             if all(distance_old < distance) || ... % minimum distance of all obstacles reached
%                     abs(theta) > pi ||... % theta below 180°
            if      ii>=100              %  max steps reached
                done = true;
            end

            % plotting
            plots(k*l).XData = [plots(k*l).XData x ];
            plots(k*l).YData = [plots(k*l).YData y,];
            
            if any(distance < Radius)
                plots(k*l).Color = 'red';
            end
             
        end
    end
end
text(1,2,"CR: "+ num2str(cr(i)))
daspect([1 1 1])
pause(1)
end