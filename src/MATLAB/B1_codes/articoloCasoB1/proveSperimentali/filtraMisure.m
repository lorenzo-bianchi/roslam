misureGrezze = misureTutte(1,:);

nMisure = numel(misureGrezze);
misureFiltrate = zeros(nMisure,1); 
misureFiltrate(1) = misureGrezze(1);

alfa = 0.2;

for k = 1:nMisure-1

    if isnan(misureGrezze(k+1))
        misureFiltrate(k+1) = alfa*misureGrezze(k) + (1-alfa)*misureFiltrate(k);
    else
        misureFiltrate(k+1) = alfa*misureGrezze(k+1) + (1-alfa)*misureFiltrate(k);
    end
end

figure
plot(misureGrezze)
hold on
plot(misureFiltrate,'r')