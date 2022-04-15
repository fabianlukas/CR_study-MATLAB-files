

% initialize actions
Ar = -1:0.2:1;
Au = -1:0.2:1;
x_temp = (1:length(Au)*length(Ar)-1)/(length(Au)*length(Ar));

%%%%%%% set CR distribution here  %%%%%%%%%%
mu = 1.0; sigma = 0.4;
pdf = normpdf(x_temp, mu, sigma)/max(normpdf(x_temp,mu,sigma)); % normal distribution
% pdf = ones(size(x_temp)); % uniform distribution
distrName = strcat('normal_mu',num2str(mu),'_sigma',num2str(sigma));
% distrName = 'uniform';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
min_N = Inf;

dminCell = {dmin1 dmin2 dmin3};
overlappedObstIndicesCell = {overlappedObstIndices_1, overlappedObstIndices_2, overlappedObstIndices_3};

% Main loop over all overlapping obstacles configurations
for i=1:length(dminCell)
    min_N_= min_N;
    dmin = dminCell{i};
    overlappedObstIndices = overlappedObstIndicesCell{i};

    clear x0 y0 vx0 vy0
    N_actionSpace = length(Au)*length(Ar);
    for i=1:N_actionSpace-1    
        min_N_ = min(min_N_, floor(length(find(dmin == i/N_actionSpace))/pdf(i)));
    end

    indices_obst_scen = [];
    for i=1:N_actionSpace-1
        n_toExtract = floor(min_N_ * pdf(i));
        [obst, scen] = find(dmin == i/N_actionSpace,n_toExtract);
        indices_obst_scen = [indices_obst_scen; [obst, scen]];
    end 

    % shuffle data
    rand_indices = randperm(length(indices_obst_scen));
    indices_obst_scen = indices_obst_scen(rand_indices,:);

    N_overlappedObstacles = size(overlappedObstIndices,3);
    for i = 1:N_overlappedObstacles % loop over all overlapped obstacles
        temp_overlappedObstIndices = overlappedObstIndices(:,:,i); % obstIndices Array for ONE obstacle
        obstacle_indices = temp_overlappedObstIndices(sub2ind(size(temp_overlappedObstIndices), indices_obst_scen(:,2), indices_obst_scen(:,1)));

        x0(:,i) = OBST_X0(sub2ind(size(OBST_X0), indices_obst_scen(:,2), obstacle_indices));
        y0(:,i) = OBST_Y0(sub2ind(size(OBST_Y0), indices_obst_scen(:,2), obstacle_indices));
        vx0(:,i) = OBST_VX0(sub2ind(size(OBST_VX0), indices_obst_scen(:,2), obstacle_indices));
        vy0(:,i) = OBST_VY0(sub2ind(size(OBST_VY0), indices_obst_scen(:,2), obstacle_indices));
    end


    u0 = U0(indices_obst_scen(:,2))';
    r0 = R0(indices_obst_scen(:,2))';

    dmin = dmin';
    dmin = dmin(sub2ind(size(dmin), indices_obst_scen(:,2), indices_obst_scen(:,1)));

    pathToFolder = strcat('data/',num2str(N_overlappedObstacles),'_obst');
    mkdir(pathToFolder, distrName)
    writematrix(x0, strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/OBST_X0'))
    writematrix(y0, strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/OBST_Y0'))
    writematrix(vx0, strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/OBST_VX0'))
    writematrix(vy0, strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/OBST_VY0'))
    writematrix(u0, strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/U0'))
    writematrix(r0, strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/R0'))
    writematrix(dmin, strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/CR'))
    size(x0)
    figure 
    histogram(dmin,200)
    title(strcat('CR distribution __',distrName,'__',num2str(N_overlappedObstacles),'obst'))
    saveas(gcf,strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/CR_histogram.png'))
end
    