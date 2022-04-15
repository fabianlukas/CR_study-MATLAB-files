% load data file and decompile struct
load('data.mat')
D_MIN = data.D_MIN;
U0 = data.U0;
R0 = data.R0;
OBST_X0 = data.OBST_X0;
OBST_Y0 = data.OBST_Y0;
OBST_VY0 = data.OBST_VY0;
OBST_VX0 = data.OBST_VX0;
TTC = data.TTC;


D_MIN_2 = []; % min dist array for two overlapping obstacles
D_MIN_3 = []; % min dist array for three overlapping obstacles
TTC_2 = [];
TTC_3 = [];

% number of overall overlaps for two and three obstacles
N_overlap_2 = size(D_MIN,3)-1;
N_overlap_3 = size(D_MIN,3)-2;

% which obstacle was overlapped with which obstacle
overlappedObstIndices = []; 
overlappedObstIndices_2 = zeros(size(D_MIN,4),N_overlap_2,2);
overlappedObstIndices_3 = zeros(size(D_MIN,4),N_overlap_3,3);

% fill overlappedObstIndices for ONE obstacle
overlappedObstIndices_1 = (1:size(D_MIN,3)) .* ones(size(D_MIN,4),1);

% overlap D_MIN arrays for TWO obstacles
for i = 1:N_overlap_2
    D_MIN_2 = cat(3, D_MIN_2, min(D_MIN(:,:,i,:), D_MIN(:,:,i+1,:)));
    TTC_2 = cat(3, TTC_2, min(TTC(:,:,i,:), TTC(:,:,i+1,:)));
    overlappedObstIndices_2(:,i,1) = deal(i);
    overlappedObstIndices_2(:,i,2) = deal(i+1);
end

% overlap D_MIN arrays for THREE obstacles
for i = 1:N_overlap_3
    D_MIN_3 = cat(3, D_MIN_3, min(D_MIN(:,:,i,:), min(D_MIN(:,:,i+1,:), D_MIN(:,:,i+2,:))));
    TTC_3 =   cat(3, TTC_3, min(TTC(:,:,i,:), min(TTC(:,:,i+1,:),  TTC(:,:,i+2,:))));

    overlappedObstIndices_3(:,i,1) = deal(i);
    overlappedObstIndices_3(:,i,2) = deal(i+1);
    overlappedObstIndices_3(:,i,3) = deal(i+2);
end