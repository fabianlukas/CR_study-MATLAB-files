% close all
% define distributions
folderVersion = "v2";
epoch_length = 20e3;

% DISTR = ["mu0s02", "mu0s03", "mu0s05", "mu0s07","uniform", "mu1s07","mu1s05", "mu1s03", "mu1s02"];
DISTR = ["mu0s02", "mu0s03", "mu0s05","uniform","mu1s05", "mu1s03", "mu1s02"];
OBST_DISTR = ["1-1-1", "025-05-1", "1-05-025"];

% define color gradients
lightColors = {[0.5608    0.8902    1.0000],[1.0000    0.7686    0.7686], [0.6745    0.9686    0.6000]};
darkColors = {[0    0.2078    0.6196],[0.6588         0         0],[ 0.2039    0.6118    0.0549]};
GRADIENTflexible = @(i,N, lightColor, darkColor) lightColor + (darkColor-lightColor)*((i-1)/(N-1));



figure('units','normalized','outerposition',[0 0 1 1])
% tiledlayout(1,1,'TileSpacing','compact','Padding','compact')
set(gcf,'color','w');
hold on
legendEntries = [];
for j=1:length(OBST_DISTR)
    for i=1:length(DISTR)
        
        searchString = strcat(folderVersion,"/*",DISTR(i),"_",OBST_DISTR(j),"*");
        fileName = dir(searchString).name;
        data = readmatrix(strcat(folderVersion,"/",fileName,"/progress.txt"));
        color = GRADIENTflexible(i,length(DISTR),lightColors{j}, darkColors{j});
        xdata = epoch_length*(1:length(data(:,8)));
        ydata = smoothdata((data(:,8)+1)./2);
        p(i*j)= plot(xdata,ydata,'Color', color,'LineWidth',2);
%         p1(i*j)= plot(xdata,ydata,'Color', color,'LineWidth',2);
        
        legendEntries = [legendEntries, strcat(DISTR(i)," | ",OBST_DISTR(j))];
        
        
        
    end
end

% plot box agent
data = readmatrix(strcat(folderVersion,"/","TD3_ObstacleAvoidance_CRstudy_box-v0_MDP_2022-07-02_28239","/progress.txt"));
color = [ 1.0000    0.2392    0.7098]; % pink color
xdata = 3*epoch_length*(0:length(data(:,8))-1);
ydata = smoothdata((data(:,8)./3.7)+0.6);
p_box = plot(xdata,ydata,'Color', color,'LineWidth',2);


legend(legendEntries, 'Location','southeast','NumColumns',3,'FontSize',20)
ylabel('ratio of successful episodes','FontSize', 24)
xlabel('timesteps','FontSize', 24)
set(gca,'FontSize',24)
xlim([0 12e6])
ylim([0.6 0.93])
box on


ah1=axes('position',get(gca,'position'),'visible','off');
plot(0,0,'Color', color,'LineWidth',2)
legend(ah1,'baseline','FontSize',20)
set(ah1, 'Color', 'none', 'XTick', [], 'YAxisLocation', 'right', 'Box', 'Off') 
axis off
box on


