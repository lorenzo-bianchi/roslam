% figure
plot([-.50 2],[0 0],'k','Linewidth',2)
hold on
% axis([-1*(L/100)/100 (L/100)+1*(L/100)/100 -1*(L/100)/100 (L/100)+1*(L/100)/100])
axis equal
plot([-.5 2],[4 4],'k','Linewidth',2)
plot([-.5 -.5],[0 4],'k','Linewidth',2)
plot([2 2],[0 4],'k','Linewidth',2)
% plot(robots(1).xVett/100,robots(1).yVett/100,':','Linewidth',1);
% plot(robots(1).xVett(1)/100,robots(1).yVett(1)/100,'k^','Linewidth',2)
% plot(robots(1).xVett(end)/100,robots(1).yVett(end)/100,'ks','Linewidth',2)
for indTag = 1:nTag,
%     disCerchio(cTag(indTag,1)/100,cTag(indTag,2)/100,rangeAntenna/100);
%     plot(cTag(indTag,1)/100,cTag(indTag,2)/100,'rH');
    plot(cTag(indTag,1)/100,cTag(indTag,2)/100,'rh','Linewidth',2,'MarkerSize',12);
end