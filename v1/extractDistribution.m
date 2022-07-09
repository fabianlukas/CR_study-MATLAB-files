% %%%%%%%%%%%%%%% load data %%%%%%%%%%%%%%%%%
% load('dmin1.mat')
% load('dmin2.mat')
% load('dmin3.mat')
% load('overlappedObstIndices_1.mat')
% load('overlappedObstIndices_2.mat')
% load('overlappedObstIndices_3.mat')
% load('data.mat')
% OBST_X0 = data.OBST_X0;
% OBST_Y0 = data.OBST_Y0;
% OBST_VY0 = data.OBST_VY0;
% OBST_VX0 = data.OBST_VX0;
% D_MIN = data.D_MIN;
% U0 = data.U0;
% R0 = data.R0;
% TTC = data.TTC;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% initialize actions
Ar = -1:0.2:1;
Au = -1:0.2:1;
x_temp = (1:length(Au)*length(Ar)-1)/(length(Au)*length(Ar));

%%%%%%% set CR distribution here  %%%%%%%%%%
% mu = 1; sigma = 0.2;
% pdf = normpdf(x_temp, mu, sigma)/max(normpdf(x_temp,mu,sigma)); % normal distribution
pdf = ones(size(x_temp)); % uniform distribution

% distrName = strcat('normal_mu',num2str(mu),'_sigma',num2str(sigma));
distrName = 'uniform_1-05-025';

% weights = [0.25 0.5 1];
weights = [1 0.5 0.25];


savingDistr = 1;
shufflingData = 1;
min_N = Inf;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



dminCell = {dmin1 dmin2 dmin3};
overlappedObstIndicesCell = {overlappedObstIndices_1, overlappedObstIndices_2, overlappedObstIndices_3};
u0_toSave = {}; r0_toSave = {}; x0_toSave= {}; y0_toSave = {}; vx0_toSave = {}; vy0_toSave = {};

% Main loop over all overlapping obstacles configurations
for k=1:length(dminCell)
    min_N_= min_N;
    dmin = dminCell{k};
    overlappedObstIndices = overlappedObstIndicesCell{k};

    clear x0 y0 vx0 vy0
    N_actionSpace = length(Au)*length(Ar);
    for i=1:N_actionSpace-1    
        min_N_ = min(min_N_, floor(length(find(dmin == i/N_actionSpace))/pdf(i)));
    end

    indices_obst_scen = [];
    for i=1:N_actionSpace-1
        n_toExtract = floor(min_N_ * pdf(i));
        if n_toExtract > 0
            [obst, scen] = find(dmin == i/N_actionSpace,n_toExtract);
            indices_obst_scen = [indices_obst_scen; [obst, scen]];
        end
    end 

    % shuffle data
    if shufflingData
        rand_indices = randperm(length(indices_obst_scen));
        indices_obst_scen = indices_obst_scen(rand_indices,:);
    end
    
    N_overlappedObstacles = size(overlappedObstIndices,3);
    for j = 1:N_overlappedObstacles % loop over all overlapped obstacles
        temp_overlappedObstIndices = overlappedObstIndices(:,:,j); % obstIndices Array for ONE obstacle
        obstacle_indices = temp_overlappedObstIndices(sub2ind(size(temp_overlappedObstIndices), indices_obst_scen(:,2), indices_obst_scen(:,1)));

        x0(:,j) = OBST_X0(sub2ind(size(OBST_X0), indices_obst_scen(:,2), obstacle_indices));
        y0(:,j) = OBST_Y0(sub2ind(size(OBST_Y0), indices_obst_scen(:,2), obstacle_indices));
        vx0(:,j) = OBST_VX0(sub2ind(size(OBST_VX0), indices_obst_scen(:,2), obstacle_indices));
        vy0(:,j) = OBST_VY0(sub2ind(size(OBST_VY0), indices_obst_scen(:,2), obstacle_indices));
    end


    u0 = U0(indices_obst_scen(:,2))';
    r0 = R0(indices_obst_scen(:,2))';

    dmin = dmin';
    dmin = dmin(sub2ind(size(dmin), indices_obst_scen(:,2), indices_obst_scen(:,1)));
    
    size(x0)
    figure 
    histogram(dmin,200)
    title(strcat('CR distribution __',distrName,'__',num2str(N_overlappedObstacles),'obst'))
    
    cr_toSave{k} = dmin;
    u0_toSave{k} = u0;
    r0_toSave{k} = r0;    
    x0_toSave{k} = x0;
    y0_toSave{k} = y0;
    vx0_toSave{k} = vx0;
    vy0_toSave{k} = vy0;
    

end

% combine distributions for all obstacle sizes

sampleSizes = cell2mat(cellfun(@length,u0_toSave,'UniformOutput',false));
minSampleSize = min(sampleSizes.*(1./weights));
 [cr, u0, r0, x0, y0, vx0, vy0]= deal([]);
for i=1:length(dminCell)
  sampleSize = round(minSampleSize*weights(i))
  cr = [cr;cr_toSave{i}(1:sampleSize)];
  u0 = [u0;u0_toSave{i}(1:sampleSize)];
  r0 = [r0;r0_toSave{i}(1:sampleSize)];
  x0 = [x0;  [x0_toSave{i}(1:sampleSize,:) 9999*ones(sampleSize,3-i)]];
  y0 = [y0;  [y0_toSave{i}(1:sampleSize,:) 9999*ones(sampleSize,3-i)]];
  vx0 = [vx0; [vx0_toSave{i}(1:sampleSize,:) 9999*ones(sampleSize,3-i)]];
  vy0 = [vy0;[vy0_toSave{i}(1:sampleSize,:) 9999*ones(sampleSize,3-i)]];
end

% shuffle data
if shufflingData
    shuffleIndices = randperm(length(u0));
    cr = cr(shuffleIndices);
    u0 = u0(shuffleIndices);
    r0 = r0(shuffleIndices);
    x0 = x0(shuffleIndices,:);
    y0 = y0(shuffleIndices,:);
    vx0 = vx0(shuffleIndices,:);
    vy0 = vy0(shuffleIndices,:);  
end



if savingDistr
    pathToFolder = strcat('data/');
    mkdir(pathToFolder, distrName)
    writematrix(x0, strcat('data/',distrName,'/OBST_X0'))
    writematrix(y0, strcat('data/',distrName,'/OBST_Y0'))
    writematrix(vx0, strcat('data/',distrName,'/OBST_VX0'))
    writematrix(vy0, strcat('data/',distrName,'/OBST_VY0'))
    writematrix(u0, strcat('data/',distrName,'/U0'))
    writematrix(r0, strcat('data/',distrName,'/R0'))
    writematrix(cr, strcat('data/',distrName,'/CR'))

%     saveas(gcf,strcat('data/',num2str(N_overlappedObstacles),'_obst/',distrName,'/CR_histogram.png'))
end
