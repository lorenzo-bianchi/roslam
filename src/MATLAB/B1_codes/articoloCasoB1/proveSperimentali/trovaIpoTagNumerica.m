function [xT1,yT1,xT2,yT2] = trovaIpoTagNumerica(x1,y1,x2,y2,R1,R2)

A = [1 -2*x1 -2*y1; 1 -2*x2 -2*y2];
b = [R1^2-x1^2-y1^2; R2^2-x2^2-y2^2];

soluzEstesa = pinv(A)*b; %randn(2,1);
posL_hat0 = soluzEstesa(2:3)*sqrt(abs(soluzEstesa(1)))/norm(soluzEstesa(2:3));
rhoVett = [R1 R2];
pos = [x1 x2; y1 y2];
opts1 = optimset('Display','off');
% posL_hatFB1 = lsqnonlin(@(x) funTot(x,pos,rhoVett),posL_hat0,-[Inf Inf],[Inf Inf],opts1);
posL_hatFB1 = lsqnonlin(@(x) funGen(x,pos,rhoVett),posL_hat0,-[Inf Inf],[Inf Inf],opts1);

% D12 = sqrt((x1-x2)^2+(y1-y2)^2);
% angPhi = acos((x2-x1)/D12);
% cosPhi = cos(angPhi);
% sinPhi = sin(angPhi);
% Rphi = [cosPhi sinPhi; -sinPhi cosPhi];
% % Rphi*[x2-x1 y2-y1]'

D12 = sqrt((x1-x2)^2+(y1-y2)^2);
angPhi = acos((x2-x1)/D12);
cosPhi = cos(angPhi);
if abs(x2-x1)>10^(-10)
    sinPhi = (y2-y1)/(x2-x1)*cos(angPhi);
else
    sinPhi = sign(y2-y1);
end
Rphi = [cosPhi sinPhi; -sinPhi cosPhi];

posL_hatFB1Trasf = Rphi*(posL_hatFB1-[x1;y1]);
posL_hatFB2 = [x1;y1] + (Rphi')*[posL_hatFB1Trasf(1); -posL_hatFB1Trasf(2)];

xT1 = posL_hatFB1(1);
yT1 = posL_hatFB1(2);
xT2 = posL_hatFB2(1);
yT2 = posL_hatFB2(2);

% figure
% plot(x1,y1,'*')
% hold on
% plot(x2,y2,'*')
% plot(posL_hatFB1(1),posL_hatFB1(2),'o')
% plot(posL_hatFB2(1),posL_hatFB2(2),'s')
% plot(x1+R1*cos(2*pi*[0:.1:10]/10),y1+R1*sin(2*pi*[0:.1:10]/10))
% plot(x2+R2*cos(2*pi*[0:.1:10]/10),y2+R2*sin(2*pi*[0:.1:10]/10))
% axis equal