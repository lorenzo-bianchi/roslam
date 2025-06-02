% Chiamata da faiFigConfrontoTransitorio.m

% indTag = 4; % 2 e 4 sono i tag reinizializzati nell'esempio
mediaInnovazione = zeros(nPassi,1);
passiReInit = find(reinizializzaVett(indTag,:));
passiReInit = [0 passiReInit];
indReInit = 1;
for k = 0:nPassi-1
    if k > passiReInit(indReInit)+49
        passoMin = max(passiReInit(indReInit),k-49);
        mediaInnovazione(k+1) = mean(storiaInnovazione(indTag,passoMin:k+1));
        if indReInit < numel(passiReInit)
            if k == passiReInit(indReInit+1)
                indReInit = indReInit + 1;
            end
        end
    else
        mediaInnovazione(k+1) = NaN;
    end
end
figure
plot(storiaInnovazione(indTag,:),'.-.','MarkerSize',8)
hold on
plot(mediaInnovazione,'r','LineWidth',2)
plot([0 nPassi],10*[1 1],'k--','LineWidth',1)
plot([0 nPassi],-10*[1 1],'k--','LineWidth',1)
plot(50*[1 1],50*[-1 1],'k--','LineWidth',1)
plot(125*[1 1],50*[-1 1],'k--','LineWidth',1)
plot(175*[1 1],50*[-1 1],'k--','LineWidth',1)
xlabel('Step k')
ylabel('cm')
grid on
legend('$I_{4,k}$','$\bar{I}_{4,k}$','Interpreter','latex')