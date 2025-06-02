function [xT,yT] = trovaPosTag(x1,y1,x2,y2,x3,y3,R1,R2,R3)

% Verifica che i 3 punti non siano allineati...
xPunti = [x1 x2 x3];
yPunti = [y1 y2 y3];
[xOrd,indOrd] = sort(xPunti);
yOrd = yPunti(indOrd);
phi12 = atan2(yOrd(2)-yOrd(1),xOrd(2)-xOrd(1));
phi23 = atan2(yOrd(3)-yOrd(2),xOrd(3)-xOrd(2));
% abs(phi12-phi23)
if abs(phi12-phi23)<10*pi/180
    disp('allineati!')
    return
end

A = [1 -2*x1 -2*y1; 1 -2*x2 -2*y2;  1 -2*x3 -2*y3];
b = [R1^2-x1^2-y1^2; R2^2-x2^2-y2^2; R3^2-x3^2-y3^2];

soluzEstesa = pinv(A)*b; %randn(3,1);
posL_hat0 = soluzEstesa(2:3)*sqrt(abs(soluzEstesa(1)))/norm(soluzEstesa(2:3));
rhoVett = [R1 R2 R3];
pos = [x1 x2 x3; y1 y2 y3];
posL_hatFB = lsqnonlin(@(x) funTreTot(x,pos,rhoVett),posL_hat0);

xT = posL_hatFB(1);
yT = posL_hatFB(2);

figure
plot(x1,y1,'k*')
hold on
plot(x2,y2,'r*')
plot(x3,y3,'b*')
plot(posL_hatFB(1),posL_hatFB(2),'o')
plot(x1+R1*cos(2*pi*[0:.1:10]/10),y1+R1*sin(2*pi*[0:.1:10]/10),'k')
plot(x2+R2*cos(2*pi*[0:.1:10]/10),y2+R2*sin(2*pi*[0:.1:10]/10),'r')
plot(x3+R3*cos(2*pi*[0:.1:10]/10),y3+R3*sin(2*pi*[0:.1:10]/10),'b')
axis equal
