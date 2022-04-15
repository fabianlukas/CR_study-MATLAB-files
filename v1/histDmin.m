dmin1 = reshape(D_MIN,[],size(D_MIN,3),size(D_MIN,4));
dmin2 = reshape(D_MIN_2,[],size(D_MIN_2,3),size(D_MIN_2,4));
dmin3 = reshape(D_MIN_3,[],size(D_MIN_3,3),size(D_MIN_3,4));

ttc1 = reshape(TTC,[],size(TTC,3),size(TTC,4));
ttc2 = reshape(TTC_2,[],size(TTC_2,3),size(TTC_2,4));
ttc3 = reshape(TTC_3,[],size(TTC_3,3),size(TTC_3,4));

R = 3; % SET RADIUS!!!

dmin1(dmin1 < R) = 1;
dmin1(dmin1 >= R) = 0;

dmin2(dmin2 < R) = 1;
dmin2(dmin2 >= R) = 0;

dmin3(dmin3 < R) = 1;
dmin3(dmin3 >= R) = 0;

dmin1 = sum(dmin1,1)./size(dmin1,1);
dmin2 = sum(dmin2,1)./size(dmin2,1);
dmin3 = sum(dmin3,1)./size(dmin3,1);

ttc1_mean = mean(ttc1);
ttc2_mean = mean(ttc2);
ttc3_mean = mean(ttc3);

dmin1(ttc1_mean < 60) = 1;
dmin2(ttc2_mean < 60) = 1;
dmin3(ttc3_mean < 60) = 1;

% replace scenarios with mean ttc < 60s by CR = 1


figure
subplot(1,3,1)
histogram(dmin1(:,1,:),1000)
subplot(1,3,2)
histogram(dmin2(:,1,:),1000)
subplot(1,3,3)
histogram(dmin3(:,1,:),1000)

dmin1 = reshape(dmin1, size(dmin1,2),[]);
dmin2 = reshape(dmin2, size(dmin2,2),[]);
dmin3 = reshape(dmin3, size(dmin3,2),[]);

ttc1 = reshape(ttc1, size(ttc1,2),[]);
ttc2 = reshape(ttc2, size(ttc2,2),[]);
ttc3 = reshape(ttc3, size(ttc3,2),[]);