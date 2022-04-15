close all

% constants
timesteps = 100;
obst_radius = 5;
N_obst = 3;
N_scen = 10;
deltaT = 0.1;

% initialize actions
Ar = -1:0.2:1;
Au = -1:0.2:1;

% initialize state space
tic

U0 = [];
R0 = [];
D_MIN = [];


% earth frame
x0 = 0;
y0 = 0;
theta0 = 0;

% body frame velocities
u0 = 1+rand(N_scen,1);
v0 = 0;
r0 = 0.1 * (2*rand(N_scen,1)-1) ;

% compute trajectory endpoint for each obstacle
for i=1:N_obst
        
    
    A_r = (rand()*2-1); 
    A_u = (rand()*2-1); 

    x(i) = x0; y(i) = y0; theta = theta0; u = u0; v = v0; r = r0;
    for j=1:timesteps
         [x(i),y(i),theta,u,v,r] = simulateTimeStep(x(i),y(i),theta,u,v,r,A_r,A_u);    
         if abs(theta) > pi
             break;
         end
    end
end

% compute obstacle trajectory
obst_vx = 2*(rand(1,N_obst)*2-1);
obst_vy = 2*(rand(1,N_obst)*2-1);

obst_x =  x - mtimes([-timesteps+1:2*timesteps]',-obst_vx*deltaT);
obst_y =  y - mtimes([-timesteps+1:2*timesteps]',-obst_vy*deltaT);

% initialize min distance to obstacle matrix
d_min = zeros(length(Ar),length(Au),N_obst);

% loop over different actions
for k = 1:length(Ar)
    for l = 1:length(Au)
        
        % intitialize state
        x = x0; y = y0; theta = theta0; u = u0; v = v0; r = r0; 
        
        % initialize plotting

        % close all
        plots(k*l) = plot(x,y);
        hold on 

        % main trajectory loop
        distances =  inf * ones(1,N_obst);
        done = false;
        ii = 0;
        while ~done
            ii = ii+1;
            [x,y,theta,u,v,r] = simulateTimeStep(x,y,theta,u,v,r,Ar(k),Au(l));

            % find min distance to obstacle
            d = sqrt((x-obst_x(ii,:)).^2 + (y-obst_y(ii,:)).^2);
            distances = min(distances, d);
            if all(d > distances) || ... % minimum distance of all obstacles reached
                    abs(theta) > pi || ii>=300     % theta below 180°
                done = true;
            end

%             plotting
            plots(k*l).XData = [plots(k*l).XData x];
            plots(k*l).YData = [plots(k*l).YData y];
%             pause(0.005)
             
        end
        d_min(k,l,:) = distances;
    end
end


OBST_X0(a) = obst_x(1);
OBST_Y0(a) = obst_y(1);
U0(a) = u0;
R0(a) = r0;
D_MIN(:,:,:,a) = d_min;


toc

plot(obst_x,obst_y)

for i=1:N_obst
figure
surf(Au, Ar, d_min(:,:,i))
xlabel('longitudinale Beschleunigung')
ylabel('Winkelbeschleunigung')
view(0,90)
colorbar
end
