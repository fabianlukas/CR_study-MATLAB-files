close all
clear variables
plotting = false;

N_scen = 500000;

% constants
timesteps = 100;
N_obst = 10;
deltaT = 0.1;
m = 15;
Radius = 3;

% initialize actions
Ar = -1:0.2:1;
Au = -1:0.2:1;

% initialize state space
tic


D_MIN = [];
TTC = [];
OBST_X0 = zeros(N_scen, N_obst);
OBST_Y0 = zeros(N_scen, N_obst);
OBST_VX0 = zeros(N_scen, N_obst);
OBST_VY0 = zeros(N_scen, N_obst);
for a = 1:N_scen


% earth frame
x0 = 0;
y0 = 0;
theta0 = 0;

% body frame velocities
u0 = 1+rand();
v0 = 0;
r0 = 0.1 * (2*rand()-1) ;

% compute trajectory endpoint for each obstacle
for i=1:N_obst
        
    
    A_r = (rand()*4 - 2); 
    A_u = (rand()*4 - 2); 

    x(i) = x0; y(i) = y0; theta = theta0; u = u0; v = v0; r = r0;
    
    u_end = u0 + 1/m * A_u * deltaT * timesteps; % u after n timesteps
    delta_timesteps = round(Radius/(u_end * deltaT)); % additional timesteps to compensate Radius

    for j=1:timesteps + delta_timesteps
         [x(i),y(i),theta,u,v,r] = simulateTimeStep(x(i),y(i),theta,u,v,r,A_r,A_u);    
         if abs(theta) > pi
             break;
         end
    end
end
obst_x = 0;
obst_y = 0;

while all(sqrt(obst_x(1,:).^2 + obst_y(1,:).^2) < u0*5+Radius)

% compute obstacle trajectory
obst_vx = 2*(rand(1,N_obst)*2-1);
obst_vy = 2*(rand(1,N_obst)*2-1);

obst_x =  x - mtimes([-timesteps+1:2*timesteps]',-obst_vx*deltaT);
obst_y =  y - mtimes([-timesteps+1:2*timesteps]',-obst_vy*deltaT);
end

% initialize min distance to obstacle matrix
d_min = zeros(length(Ar),length(Au),N_obst);
ttc = zeros(length(Ar),length(Au),N_obst);

% loop over different actions
for k = 1:length(Ar)
    for l = 1:length(Au)
        
        % intitialize state
        x = x0; y = y0; theta = theta0; u = u0; v = v0; r = r0; 
        
        % initialize plotting

        % close all
        if plotting
            plots(k*l) = plot(x,y);
            hold on 
        end

        % main trajectory loop
        distances =  inf * ones(1,N_obst);
        ttc_singleRow = zeros(1,N_obst);
        done = false;
        ii = 0;
        while ~done
            ii = ii+1;
            [x,y,theta,u,v,r] = simulateTimeStep(x,y,theta,u,v,r,Ar(k),Au(l));

            % find min distance to obstacle
            d = sqrt((x-obst_x(ii,:)).^2 + (y-obst_y(ii,:)).^2);
            distances = min(distances, d);
            ttc_singleRow(~(d > distances)) = ii; % safe current timestep for all obstacles with decreasing distance
            if all(d > distances) || ... % minimum distance of all obstacles reached
                    abs(theta) > pi ||... % theta below 180°
                    ii>=300              %  max steps reached
                done = true;
            end

%             plotting
            if plotting
                plots(k*l).XData = [plots(k*l).XData x];
                plots(k*l).YData = [plots(k*l).YData y];
        %             pause(0.005)
            end
             
        end
        d_min(k,l,:) = distances;
        ttc(k,l,:) = ttc_singleRow;
    end
end


OBST_X0(a,:) = obst_x(1,:);
OBST_Y0(a,:) = obst_y(1,:);
OBST_VX0(a,:) = obst_vx;
OBST_VY0(a,:) = obst_vy;
U0(a) = u0;
R0(a) = r0;
D_MIN(:,:,:,a) = d_min;
TTC(:,:,:,a) = ttc;
end

data.OBST_X0 = OBST_X0;
data.OBST_Y0 = OBST_Y0;
data.OBST_VX0 = OBST_VX0;
data.OBST_VY0 = OBST_VY0;
data.U0 = U0;
data.R0 = R0;
data.D_MIN = D_MIN;
data.TTC = TTC;
save('data.mat', 'data', '-v7.3')
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

% % reshape and minimize TTC array
% TTC = reshape(TTC,[],size(TTC,3),size(TTC,4));
% TTC = mean(TTC);
% TTC = reshape(TTC,size(TTC,2),[]);