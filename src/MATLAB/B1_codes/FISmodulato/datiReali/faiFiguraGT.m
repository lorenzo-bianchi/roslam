load roslam_data3.mat
load gt_test7.mat

xGT = robot1(:,2);
yGT = robot1(:,3);

figure
plot(xGT,yGT,'r--','LineWidth',2)
hold on
plot(xGT(1),yGT(1),'g.','MarkerSize',24)
plot(xGT(end),yGT(end),'r.','MarkerSize',24)
grid on

xGT = robot2(:,2);
yGT = robot2(:,3);

plot(xGT,yGT,'c','LineWidth',2)
hold on
plot(xGT(1),yGT(1),'g.','MarkerSize',24)
plot(xGT(end),yGT(end),'r.','MarkerSize',24)
xlabel('x [m]')
ylabel('y [m]')

cTag = [0, 0;
    1.7903, 0;
    1.7241, 3.6934;
    -0.1471, 3.7211]

for indTag = 1:4
    plot(cTag(indTag,1),cTag(indTag,2),'rh','LineWidth',2,'MarkerSize',12)
end