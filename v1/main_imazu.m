close all
clear variables

delete 'Imazu/data/OBST_X0.txt'
delete 'Imazu/data/OBST_Y0.txt'
delete 'Imazu/data/OBST_VX0.txt'
delete 'Imazu/data/OBST_VY0.txt'
delete 'Imazu/data/U0.txt'
delete 'Imazu/data/R0.txt'
delete 'Imazu/data/CR.txt'




elements_per_cr_bin = 10;
no_cr_bins = 10;
cr_bin_edges = 0:0.1:1;

% constants
timesteps = 100;
deltaT = 0.1;
m = 15;
Radius = 3;

% initialize actions
Ar = -1:0.2:1;
Au = -1:0.2:1;

% initialize state space
tic

for imazu_counter = 1:22

    D_MIN = [];TTC = [];OBST_X0 = [];OBST_Y0 = [];OBST_VX0 = [];OBST_VY0 = [];CR = [];U0 = [];R0 = [];
    while any(histcounts(CR,cr_bin_edges)<elements_per_cr_bin)


   % constant agent vars
    r0 = 0;    
    v0 = 0;
    y0 = 0;
    theta0 = 0;


    [obst_x0, obst_y0, obst_vx, obst_vy, R, agent_u0] = init_imazu(imazu_counter, Radius, deltaT, timesteps);
    N_obst = numel(obst_x0);
    
    u0 = agent_u0;
    x0 = -R;

    obst_x =  obst_x0 + mtimes((0:3*timesteps)',obst_vx*deltaT);
    obst_y =  obst_y0 + mtimes((0:3*timesteps)',obst_vy*deltaT);


    % initialize min distance to obstacle matrix
    d_min = zeros(length(Ar),length(Au),N_obst);
    ttc = zeros(length(Ar),length(Au),N_obst);
    agent_x = [];
    agent_y = [];
    % loop over different actions
    for k = 1:length(Ar)
        for l = 1:length(Au)

            % intitialize state
            x = x0; y = y0; theta = theta0; u = u0; v = v0; r = r0; 

            % main trajectory loop
            distances =  inf * ones(1,N_obst);
            ttc_singleRow = zeros(1,N_obst);
            done = false;
            ii = 0;
            while ~done
                ii = ii+1;
                [x,y,theta,u,v,r] = simulateTimeStep(x,y,theta,u,v,r,Ar(k),Au(l));
                 agent_x = [agent_x x];
                 agent_y = [agent_y y];
              
                % find min distance to obstacle
                d = sqrt((x-obst_x(ii,:)).^2 + (y-obst_y(ii,:)).^2);
                distances = min(distances, d);
                if all(d > distances) || ... % minimum distance of all obstacles reached
                        any(d < Radius) || ...
                        abs(theta) > pi ||... % theta below 180°
                        ii>=300              %  max steps reached
                    done = true;
                end

            end
            d_min(k,l,:) = distances;
            ttc(k,l,:) = ii;
        end
    end
    d_min = min(d_min,[],3);
    cr = numel(find(d_min < Radius))/numel(d_min);
    histc = histcounts(CR,cr_bin_edges);
    cr_index = find(histcounts(cr,cr_bin_edges) == 1);
    if cr < 0.98 ...
        && cr > 0.02 ...
        && mean(ttc(:)) > 50 ...
        && histc(cr_index) < elements_per_cr_bin
        OBST_X0 = [OBST_X0;   [-x0 + obst_x(1,:) 9999*ones(1,3-length(obst_x(1,:)))]];
        OBST_Y0 = [OBST_Y0;   [obst_y(1,:) 9999*ones(1,3-length(obst_y(1,:)))]];
        OBST_VX0 = [OBST_VX0; [obst_vx 9999*ones(1,3-length(obst_vx))]];
        OBST_VY0 = [OBST_VY0; [obst_vy 9999*ones(1,3-length(obst_vy))]];

        U0 = [U0; u0];
        R0 = [R0; r0];
        CR = [CR; cr]; 
        if histc(cr_index) == 1
%             figure
%             scatter(agent_x, agent_y,1)
%             hold on
%             scatter(obst_x(:),obst_y(:),1)
%             title(strcat('CR: ',num2str(cr)))
        end
    end

    end


    writematrix(OBST_X0,'Imazu/data/OBST_X0.txt','WriteMode','append')
    writematrix(OBST_Y0,'Imazu/data/OBST_Y0.txt','WriteMode','append')
    writematrix(OBST_VX0,'Imazu/data/OBST_VX0.txt','WriteMode','append')
    writematrix(OBST_VY0,'Imazu/data/OBST_VY0.txt','WriteMode','append')
    writematrix(U0,'Imazu/data/U0.txt','WriteMode','append')
    writematrix(R0,'Imazu/data/R0.txt','WriteMode','append')
    writematrix(CR,'Imazu/data/CR.txt','WriteMode','append')
    
    %figure
    %scatter(OBST_X0,OBST_Y0)
end
