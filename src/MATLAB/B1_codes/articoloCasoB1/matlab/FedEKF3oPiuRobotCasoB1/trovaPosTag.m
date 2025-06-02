function [xT,yT,allineati] = trovaPosTag(xCentri,yCentri,raggiCerchi)

nCerchi = numel(xCentri);

% Verifica che i centri non siano allineati...
[xOrd,indOrd] = sort(xCentri);
yOrd = yCentri(indOrd);
deltaPhiA = atan2(yOrd(2)-yOrd(1),xOrd(2)-xOrd(1));
deltaPhiB = atan2(yOrd(3)-yOrd(2),xOrd(3)-xOrd(2));
indCerchio = 3;
while indCerchio<nCerchi && abs(deltaPhiB-deltaPhiA)<10*pi/180
    deltaPhiA = deltaPhiB;
    deltaPhiB = atan2(yOrd(indCerchio+1)-yOrd(indCerchio),xOrd(indCerchio+1)-xOrd(indCerchio));
    indCerchio = indCerchio +1;
end
if indCerchio == nCerchi && abs(deltaPhiB-deltaPhiA)<10*pi/180 
    allineati = 1;
    disp('allineati!')
    % return
else
    allineati = 0;
end


A = [ones(nCerchi,1) -2*xCentri -2*yCentri];
b = raggiCerchi.^2-xCentri.^2-yCentri.^2;

soluzEstesa = pinv(A)*b;
posL_hat0 = soluzEstesa(2:end)*sqrt(abs(soluzEstesa(1)))/norm(soluzEstesa(2:end));
pos = [xCentri'; yCentri'];
opts1 = optimset('Display','off');
% posL_hatFB = lsqnonlin(@(x) funEnneTot(x,pos,raggiCerchi),posL_hat0,-Inf*ones(numel(posL_hat0),1),Inf*ones(numel(posL_hat0),1),opts1);
posL_hatFB = lsqnonlin(@(x) funGen(x,pos,raggiCerchi'),posL_hat0,-Inf*ones(numel(posL_hat0),1),Inf*ones(numel(posL_hat0),1),opts1);

xT = posL_hatFB(1);
yT = posL_hatFB(2);

% figure
% plot(xPunti(1),yPunti(1),'b*')
% hold on
% for indCerchio = 2:nCerchi
%     plot(xPunti(indCerchio),yPunti(indCerchio),'b*')
% end
% plot(xT,yT,'ro','LineWidth',2)
% for indCerchio = 1:nCerchi
%     plot(xPunti(indCerchio)+raggiCerchi(indCerchio)*cos(2*pi*[0:.1:10]/10),yPunti(indCerchio)+raggiCerchi(indCerchio)*sin(2*pi*[0:.1:10]/10),'b')
% end
% axis equal
