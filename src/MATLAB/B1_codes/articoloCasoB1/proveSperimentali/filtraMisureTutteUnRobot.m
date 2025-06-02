for indMisura = 1:4

    misureGrezze = misureTutte(indMisura,:);
    
    nMisure = numel(misureGrezze);
    misureFiltrate = zeros(nMisure,1); 
    misureFiltrate(1) = misureGrezze(1);
    
    % Elimino i NaN
    for k = 1:nMisure
        if isnan(misureGrezze(k))
            misureGrezze(k) = misureGrezze(k-1);
        end
    end
    
    finestra = 10;
    for k = 1:nMisure-1
        kIn = max(1,k-finestra+1);
        misureFiltrate(k+1) = mean(misureGrezze(kIn:k+1));
    end
    
    % figure
    % plot(misureGrezze)
    % hold on
    % plot(misureFiltrate,'r')
    % 
    % pause

    misureTutte(indMisura,:) = misureFiltrate;

end