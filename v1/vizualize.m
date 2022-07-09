close all

DYNAMICS.SET = 3; % 1: simple   2: vessel 3: vessel2
DYNAMICS.deltaT = [0.1 0.08 7];

% constants
episodeLength = 120;
deltaT = DYNAMICS.deltaT(DYNAMICS.SET);
Radius = 3;

scenariosToPlot = [4,12,5];
n_scenarios = length(scenariosToPlot);

figure('units','normalized','outerposition',[0 0 1 1])
tiledlayout(1,n_scenarios,'TileSpacing','compact','Padding','compact')
set(gcf,'color','w');

%%%%%%%%%%%% load data %%%%%%%%%%%%%%%%
cr = load("data/CR.txt");
u0 = load("data/U0.txt");
r0 = load("data/R0.txt");

obst_x0 = load("data/OBST_X0.txt");
obst_y0 = load("data/OBST_Y0.txt");
obst_vx0 = load("data/OBST_VX0.txt");
obst_vy0 = load("data/OBST_VY0.txt");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% initialize actions
Ar = -1:0.1:1;
Ar = abs(Ar).*Ar;
Au = -1:0.2:1;
% Ar = 1;
% Au = 1;

%for plotting circle
th = linspace(0,2*pi,100);
darkColor = [ 0.6196    0.6196    0.6196];
lightColor = [0.9804    0.9804    0.9804];
GRADIENTflexible = @(i,N, lightColor, darkColor) lightColor + (darkColor-lightColor)*((i-1)/(N-1));

% main simulation loop
for i_scenario = 1:n_scenarios
    scen_index = scenariosToPlot(i_scenario);
    nexttile
    for k = 1:length(Ar)
        for l = 1:length(Au)
            
            % intitialize state
            x = 0; y = 0; theta = 0; u = u0(scen_index); v = 0; r = r0(scen_index);
            x_obst = obst_x0(scen_index,obst_x0(scen_index,:)~=9999); y_obst = obst_y0(scen_index,obst_x0(scen_index,:)~=9999); vx_obst = obst_vx0(scen_index,obst_x0(scen_index,:)~=9999); vy_obst = obst_vy0(scen_index,obst_x0(scen_index,:)~=9999);
            u=0;r=0;v=0; %!!!!!!!!!!!!!!!!!!!!!!!!!!!!REMOOOOOVE!"!"!"!"!"!"!"!"!"!"
            %         u = 1;
            %         r = 0.1;
            
            % initialize plotting
            % close all
            
            plots(k*l) = plot(x,y, 'LineWidth', 1);
            plots(k*l).Color = 'blue';
            hold on
            
            
            % main trajectory loop
            done = false;
            ii = 0;
            distance = Inf;
            while ~done
                ii = ii+1;
                [x,y,theta,u,v,r] = simulateTimeStep(x,y,theta,u,v,r,Ar(k),Au(l),DYNAMICS);
                
                x_obst = x_obst + vx_obst * deltaT;
                y_obst = y_obst + vy_obst * deltaT;
                
                % find min distance to obstacle
                distance_old = distance;
                distance = sqrt((x-x_obst).^2 + (y-y_obst).^2);
                
                %             if all(distance_old < distance) || ... % minimum distance of all obstacles reached
                %                     abs(theta) > pi ||... % theta below 180°
                if      ii>=episodeLength              %  max steps reached
                    done = true;
                end
                
                % plotting
                plots(k*l).XData = [plots(k*l).XData x ];
                plots(k*l).YData = [plots(k*l).YData y,];
                
                % plot obstacle
%                 if k==1                    
%                     color =  GRADIENTflexible(ii,episodeLength,lightColor,darkColor);
%                     for i_obst = 1:length(x_obst)
%                         p_obst = plot(x_obst(i_obst)+1.5*cos(th),y_obst(i_obst)+1.5*sin(th),'LineWidth',1,'Color', color);
%                     end   
%                 end
                
                if any(distance < Radius)
                    plots(k*l).Color = 'red';
                    p_red = plots(k*l);
                else
                    p_blue = plots(k*l);
                end
                
            end
        end
    end
    title("CR = "+ num2str(cr(scen_index),'%4.2f'),'FontSize',20)
    daspect([1 1 1])
    pbaspect([1 1.5 1])
    set(gca,'YTickLabel',[]);
    set(gca,'XTickLabel',[]);
    pause(0.001)
end
legend([p_red, p_blue, p_obst],{'collision','successful','obstacle'}, 'Location','northeast','FontSize',20)
set(gcf,'renderer','Painters')