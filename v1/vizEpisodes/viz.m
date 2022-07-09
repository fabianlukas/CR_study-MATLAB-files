close all
files = { [ "1obs_red.csv", "1obs_blue", "1obs_green", "1obs_purple"],
          [ "2obs_red.csv", "2obs_blue", "2obs_green", "2obs_purple"],
          [ "3obs_red.csv", "3obs_blue.csv", "3obs_green.csv", "3obs_purple.csv"]};
plotColors = { [0.6588         0         0], [0.2971    0.6735    0.8530], [0.6745    0.9686    0.6000],  [ 1.0000    0.2392    0.7098]};
legendLabels = ["mu1s02 | 025-05-1", "mu0s05 | 1-1-1", "mu0s02 | 1-05-025", "baseline", "obstacles"];

frames = [ 20 28 35 39;
           20 32 44 53;
           20 32 48 64];   
       
  deltaY = 35.3910;
  deltaX = 27.1331;
 YLIM = {[-deltaY/2 deltaY/2], [17.9150-deltaY 17.915], [-24.3120   11.0790]};
 XLIM = {[1.1064   28.2396]};
% figure('units','normalized','outerposition',[0 0 1 1])


n_episodes = size(files,1);
n_frames = size(frames,2);
figure('units','normalized','outerposition',[0 0 1 1])
tiledlayout(n_episodes,n_frames,'TileSpacing','compact','Padding','compact')
set(gcf,'color','w');

%for plotting circle
th = linspace(0,2*pi,100); 

for i_episodes=1:n_episodes
    max_timeSteps = 0;
    for i_frame=1:n_frames
        nexttile
        % prepare subplot
%         s = subplot(n_episodes,n_frames,((i_episodes-1)*n_frames)+i_frame);
        set(gca,'YTickLabel',[]);
        set(gca,'XTickLabel',[]);
        box on
        hold on
        title(strcat("t = ", num2str(frames(i_episodes,i_frame)*0.2),"s"));
        
        for i_file = 1:length(files{i_episodes})
            
            % read file
            data = readmatrix(files{i_episodes}(i_file));
            
            x = data(:,end-1);
            y = data(:,end);
            if frames(i_episodes,i_frame) <= length(x)
                timeStepsToPlot = frames(i_episodes,i_frame);
            else
                timeStepsToPlot = length(x);
            end
            
            if timeStepsToPlot > max_timeSteps
                max_timeSteps = timeStepsToPlot;
                max_timeSteps_iFile = i_file;
            end
            
            
            
            % plot agent trajectory
%             viscircles([x(timeStepsToPlot),y(timeStepsToPlot)],1.5,'LineWidth',0.5,'Color',plotColors{i_file})
                       
            plot(x(timeStepsToPlot)+1.5*cos(th),y(timeStepsToPlot)+1.5*sin(th),'LineWidth',0.5,'Color',plotColors{i_file})
            
            p(i_file) = plot3(x(1:timeStepsToPlot),y(1:timeStepsToPlot),ones(timeStepsToPlot,1),'-','Color', plotColors{i_file},'LineWidth',1);
        end
        
        
        
        % plot obstalces trajectory
        data = readmatrix(files{i_episodes}(max_timeSteps_iFile)); % set file for which obstacles to be load!!!
        n_obst = (size(data,2)-2)/2;
        x_obst = data(:,1:n_obst);
        y_obst = data(:,n_obst+1:2*n_obst);
        for i=1:n_obst
%             viscircles([x_obst(max_timeSteps,i),y_obst(max_timeSteps,i)],1.5,'LineWidth',0.5,'Color',[0.9020    0.9020    0.9020])
            plot(x_obst(max_timeSteps,i)+1.5*cos(th),y_obst(max_timeSteps,i)+1.5*sin(th),'LineWidth',0.5,'Color',[0.9020    0.9020    0.9020])
            p(length(files{i_episodes})+1) = plot3(x_obst(1:max_timeSteps,i),y_obst(1:max_timeSteps,i),ones(max_timeSteps,1),'-', 'Color', 0.5*ones(1,3),'LineWidth',1);
            
        end
        if i_frame == 1
            x_max = -Inf; y_min = -6; y_max = -Inf;
            x_max = max([x_max max(max(x_obst(timeStepsToPlot:end,:)))])+3;
            y_max = max([y_max max(max(y_obst(timeStepsToPlot:end,:)))])+3;
            y_min = min([y_min min(min(y_obst(timeStepsToPlot:end,:)))])-3;
        end
%         ylim([y_min y_max])
        xlim(XLIM{1})
       daspect([1.15 1 1])
       pbaspect([1 1.5 1])
        
    end
end
legend(p(1:5),legendLabels, 'Location','southwest')
 set(gcf,'renderer','Painters')
