% FedEkf class definition
classdef FedEkf < handle
    properties
        data
        id
        innovazione
        pesi
        xHatSLAM
        xHatTagStoria
        yHatTagStoria
        xHatTagSigmaStoria
        yHatTagSigmaStoria
        P
        Ptag
        F
        W
        H
        Rs
        k
        sigmaD
        sigmaPhi
        sigmaMisuraMedia
        nPhiVett
        xHatIndices
        xHatCumIndices
        Hx
        Hy
        innovazioneX
        innovazioneY
        RsX
        RsY
        startPruning
        minZerosStartPruning
        varX
        varY
        covXY
        do_reset
        nReset
        lastMeasures
        varsStoria
        firstCorrection
    end
    
    methods
        % Class constructor
        function obj = FedEkf(data, id, x0, misure)
            obj.data = data;

            obj.id = id;

            obj.sigmaD = data.sigmaDistanzaModello;
            obj.sigmaPhi = data.sigmaPhi;
            obj.sigmaMisuraMedia = data.sigmaMisuraMedia;

            nTag = data.nTag;
            nPhiMax = data.nPhi;
            obj.nPhiVett = data.nPhi*ones(1, nTag);
            nPassi = data.nPassi;

            obj.innovazione = zeros(nTag*nPhiMax, 1);
            obj.pesi = (1/nPhiMax)*ones(nTag, nPhiMax);
            obj.xHatSLAM = zeros(3+(3+nPhiMax)*nTag, nPassi);
            obj.P = zeros(3+(3+nPhiMax)*nTag, 3+(3+nPhiMax)*nTag);
            obj.Ptag = diag([0, 0, obj.sigmaD^2, obj.sigmaPhi^2*ones(1, nPhiMax)]);

            obj.F = eye(3+(3+nPhiMax)*nTag);
            obj.W = zeros(3+(3+nPhiMax)*nTag, 2);
            obj.H = zeros(nTag*nPhiMax, 3+(3+nPhiMax)*nTag);
            obj.Rs = zeros(nTag*nPhiMax, nTag*nPhiMax);

            obj.k = 0;

            obj.xHatIndices = [1 3 (3+nPhiMax)*ones(1, nTag)];
            obj.xHatCumIndices = cumsum(obj.xHatIndices(1:end-1));

            obj.Hx = zeros(nTag*nPhiMax,3+(3+nPhiMax)*nTag);
            obj.Hy = zeros(nTag*nPhiMax,3+(3+nPhiMax)*nTag);
            obj.innovazioneX = zeros(nTag*nPhiMax, 1);
            obj.innovazioneY = zeros(nTag*nPhiMax, 1);
            obj.RsX = zeros(nTag*nPhiMax, nTag*nPhiMax);
            obj.RsY = zeros(nTag*nPhiMax, nTag*nPhiMax);

            % Inizializzazione
            obj.xHatSLAM(1:3, 1) = x0;
            obj.xHatSLAM(4:nPhiMax+3:end, 1) = obj.xHatSLAM(1,1);
            obj.xHatSLAM(5:nPhiMax+3:end, 1) = obj.xHatSLAM(2,1);
            obj.xHatSLAM(6:nPhiMax+3:end, 1) = misure;
            for jndPhi = 1:nPhiMax
                obj.xHatSLAM(6+jndPhi:nPhiMax+3:end,1) = data.possibiliPhi(jndPhi);
            end
            for indTag = 1:nTag    
                obj.P(4+(3+nPhiMax)*(indTag-1):3+(3+nPhiMax)*indTag, 4+(3+nPhiMax)*(indTag-1):3+(3+nPhiMax)*indTag) = obj.Ptag;
            end

            obj.xHatTagStoria = zeros(nTag, nPassi);
            obj.yHatTagStoria = zeros(nTag, nPassi);
            obj.xHatTagSigmaStoria = zeros(1, nPassi);
            obj.yHatTagSigmaStoria = zeros(1, nPassi);

            obj.startPruning = zeros(1, nTag);
            obj.minZerosStartPruning = data.minZerosStartPruning;

            obj.varX = zeros(1, nTag);
            obj.varY = zeros(1, nTag);
            obj.covXY = zeros(1, nTag);

            obj.varsStoria = zeros(2, nTag, nPassi);

            obj.nReset = 0;
            obj.do_reset = 0;

            obj.firstCorrection = 1;
        end
        
        %
        function [obj] = prediction(obj, uRe, uLe)
            obj.k = obj.k + 1;

            uk = (uRe + uLe)/2;
            omegak = (uRe - uLe)/obj.data.d;

            cosk = cos(obj.xHatSLAM(3, obj.k));
            sink = sin(obj.xHatSLAM(3, obj.k));
        
            % Gli unici elementi che cambiano sono le coordinate
            % del robot in posizione 1, 2 e 3 del vettore xHatSLAM
            obj.xHatSLAM(:, obj.k+1) = obj.xHatSLAM(:, obj.k);
            obj.xHatSLAM(1, obj.k+1) = obj.xHatSLAM(1, obj.k) + uk*cosk;
            obj.xHatSLAM(2, obj.k+1) = obj.xHatSLAM(2, obj.k) + uk*sink;
            obj.xHatSLAM(3, obj.k+1) = obj.xHatSLAM(3, obj.k) + omegak;
        
            % Aggiornamento degli elementi variabili della jacobiana F = df/dx
            obj.F(1,3) = -uk*sink;
            obj.F(2,3) = uk*cosk;
            
            % Jacobiana W = df/dw
            obj.W(1,1) = 0.5*cosk;
            obj.W(1,2) = 0.5*cosk;
            obj.W(2,1) = 0.5*sink;
            obj.W(2,2) = 0.5*sink;
            obj.W(3,1) = 1/obj.data.d;
            obj.W(3,2) = -1/obj.data.d;
            
            % Matrice covarianza errore odometrico
            Q = diag([obj.data.KR*abs(uRe); obj.data.KL*abs(uLe)]);
        
            % Calcolo matrice P^-
            obj.P = obj.F*obj.P*obj.F' + obj.W*Q*obj.W';
        end
        
        % 
        function [obj] = correction(obj, misureRange)
            if obj.firstCorrection
                nPhiMax = obj.data.nPhi;
                obj.firstCorrection = 0;
                obj.xHatSLAM(4:nPhiMax+3:end, obj.k+1) = obj.xHatSLAM(1, obj.k+1);
                obj.xHatSLAM(5:nPhiMax+3:end, obj.k+1) = obj.xHatSLAM(2, obj.k+1);
                obj.xHatSLAM(6:nPhiMax+3:end, obj.k+1) = misureRange;
            end
            obj.lastMeasures = misureRange;
            nTag = obj.data.nTag;

            % Stima a priori posizione robot
            x_r = obj.xHatSLAM(1, obj.k+1);
            y_r = obj.xHatSLAM(2, obj.k+1);

            indMatCum = cumsum([0 obj.nPhiVett(1:end-1)]);
    
            for indTag = 1:nTag
                % Calcolo della stima a priori della posizione dell'ipotesi 
                % j del landmark i, quindi la misura attesa da questa posizione, 
                % la sua probabilità e la corrispondente riga della jacobiana H
                indMat = indMatCum(indTag);

                nPhi = obj.nPhiVett(indTag);
                ind0 = obj.xHatCumIndices(indTag+1);
                x_i   = obj.xHatSLAM(0+ind0, obj.k+1);
                y_i   = obj.xHatSLAM(1+ind0, obj.k+1);
                rho_i = obj.xHatSLAM(2+ind0, obj.k+1);

                probMisura_ij = zeros(nPhi, 1);

                var_ = nPhi * obj.sigmaD^2;

                for indPhi = 1:nPhi
                    phi_ij = obj.xHatSLAM(2+ind0+indPhi, obj.k+1);
                    cosPhi_ij = cos(phi_ij);
                    sinPhi_ij = sin(phi_ij);
                    xTag_ij = x_i + rho_i*cosPhi_ij;
                    yTag_ij = y_i + rho_i*sinPhi_ij;
                    misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                    deltaMisura_ij = misureRange(indTag) - misuraRange_ij;

                    obj.innovazione(indMat+indPhi) = deltaMisura_ij;
                    probMisura_ij(indPhi) = exp(-deltaMisura_ij^2/(2*var_));
                    probMisura_ij(indPhi) = max(probMisura_ij(indPhi), 1e-100);
                    obj.pesi(indTag,indPhi) = obj.pesi(indTag,indPhi)*probMisura_ij(indPhi);

                    obj.H(indMat+indPhi, 1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                    obj.H(indMat+indPhi, 0+ind0) = xTag_ij-x_r;
                    obj.H(indMat+indPhi, 1+ind0) = yTag_ij-y_r;
                    obj.H(indMat+indPhi, 2+ind0) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                    obj.H(indMat+indPhi, 2+ind0+indPhi) = ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                    obj.H(indMat+indPhi, :) = obj.H(indMat+indPhi, :) / misuraRange_ij;
                end
                lambda_ij = probMisura_ij/sum(probMisura_ij);
                
                % Matrice covarianza misure con formula che tiene conto dell'Information Sharing
                for indPhi = 1:nPhi
                    obj.Rs(indMat+indPhi, indMat+indPhi) = var_/(max(0.0001, lambda_ij(indPhi)));
                end
    
            end
    
            % Aggiornamento stima (a posteriori)
            KalmanGain = obj.P*obj.H'*pinv(obj.H*obj.P*obj.H'+obj.Rs);
            obj.xHatSLAM(:, obj.k+1) = obj.xHatSLAM(:, obj.k+1) + KalmanGain*obj.innovazione;
            obj.P = (eye(sum(obj.xHatIndices)-1) - KalmanGain*obj.H)*obj.P;
    
            % Aggiornamento pesi
            obj.pesi = obj.pesi ./ sum(obj.pesi, 2);
        end

        % 
        function [obj] = pruning(obj)
            nTag = obj.data.nTag;
            change = false;

            if obj.id == 1 && obj.k == 720
                a = 1;
            end

             pruning_thr = min(5e-2, 0.00001 * obj.k);

            obj.save_history();

            if any(obj.startPruning == 0)
                for indTag = 1:nTag
                    if obj.startPruning(indTag) > 0
                        continue
                    end

                    if 1
                        n_steps = 10;
    
                        t = 1:obj.k+1;
                        var_x = squeeze(obj.varsStoria(1, indTag, t))';
                        var_y = squeeze(obj.varsStoria(2, indTag, t))';
    
                        sigma_x = sqrt(var_x); sigma_y = sqrt(var_y);
                        grad_sigma_x = diff(sigma_x);
                        grad_sigma_y = diff(sigma_y);
                        
                        cond = (grad_sigma_x == 0);
                        grad_sigma_x(cond) = []; grad_sigma_y(cond) = [];
                        if length(grad_sigma_x) < n_steps+1
                            return
                        end
                        grad_sigma_x = grad_sigma_x(end-n_steps+2:end);
                        grad_sigma_y = grad_sigma_y(end-n_steps+2:end);
    
                        % Filtro FIR per grad_sigma_x e grad_sigma_y
                        filtro_finestra = 6; % Lunghezza finestra del filtro
                        kernel = ones(1, filtro_finestra) / filtro_finestra; % Kernel media mobile
                        
                        % Applicazione del filtro (manuale)
                        grad_sigma_x_filtrato = zeros(size(grad_sigma_x));
                        grad_sigma_y_filtrato = zeros(size(grad_sigma_y));
                        
                        for i = 1:length(grad_sigma_x)
                            for j = 1:filtro_finestra
                                if i - j + 1 > 0
                                    grad_sigma_x_filtrato(i) = grad_sigma_x_filtrato(i) + kernel(j) * grad_sigma_x(i - j + 1);
                                    grad_sigma_y_filtrato(i) = grad_sigma_y_filtrato(i) + kernel(j) * grad_sigma_y(i - j + 1);
                                end
                            end
                        end
    
                        % Sostituisci grad_sigma_x e grad_sigma_y con i valori filtrati
                        grad_sigma_x = grad_sigma_x_filtrato;
                        grad_sigma_y = grad_sigma_y_filtrato;
                        
                        diff_thr = 0.0;
                        decreasing = (grad_sigma_x < diff_thr) & (grad_sigma_y < diff_thr);
    
                        if all(decreasing)
                            obj.startPruning(indTag) = obj.k;
                        end
                    else
                        nPhi = obj.nPhiVett(indTag);
                        nZeri = 0;
                        for indPhi = 1:nPhi
                            if obj.pesi(indTag, indPhi) < pruning_thr/nPhi
                                nZeri = nZeri + 1;
                            end
                        end
                        if nZeri >= obj.minZerosStartPruning
                            obj.startPruning(indTag) = obj.k;
                        end
                    end
                end
            end

            for indTag = 1:nTag
                if obj.startPruning(indTag) == 0
                    continue
                end

                nPhi = obj.nPhiVett(indTag);
                indPhi = 1;
                while indPhi <= nPhi
                    if obj.pesi(indTag, indPhi) < pruning_thr/nPhi
                        change = true;
                        nPhi = nPhi - 1;

                        temp = obj.pesi(indTag, indPhi+1:end);
                        obj.pesi(indTag, indPhi:indPhi+length(temp)-1) = temp;
                        obj.pesi(indTag, end) = 0;

                        obj.xHatIndices(2+indTag) = obj.xHatIndices(2+indTag) - 1;
                        obj.xHatCumIndices(2+indTag:end) = obj.xHatCumIndices(2+indTag:end) - 1;
                        obj.nPhiVett(indTag) = obj.nPhiVett(indTag) - 1;

                        ind0 = obj.xHatCumIndices(indTag+1);
                        i = ind0 + 2 + indPhi;
                        obj.P = obj.P([1:i-1, i+1:end], [1:i-1, i+1:end]);
                        obj.xHatSLAM = obj.xHatSLAM([1:i-1, i+1:end], :);
                    else
                        indPhi = indPhi + 1;
                    end
                end

                ind0 = obj.xHatCumIndices(indTag+1);
                if nPhi == 2
                    phi1 = obj.xHatSLAM(2+ind0+1, obj.k+1);
                    phi2 = obj.xHatSLAM(2+ind0+2, obj.k+1);

                    peso1 = obj.pesi(indTag, 1);
                    peso2 = obj.pesi(indTag, 2);

                    min_w = 0.9;
                    w = max(min_w, -(1-min_w)/6000*obj.k+1);
                    delta = mod(abs(phi1 - phi2), 2*pi);
                    if delta > pi
                        delta = 2*pi - delta;
                    end
                    if max(peso1, peso2) > w || delta < 10*pi/180
                        change = true;

                        if peso1 > peso2
                            indPhi = 2;
                        else
                            indPhi = 1;
                        end

                        temp = obj.pesi(indTag, indPhi+1:end);
                        obj.pesi(indTag, indPhi:indPhi+length(temp)-1) = temp;
                        obj.pesi(indTag, end) = 0;

                        obj.xHatIndices(2+indTag) = obj.xHatIndices(2+indTag) - 1;
                        obj.xHatCumIndices(2+indTag:end) = obj.xHatCumIndices(2+indTag:end) - 1;
                        obj.nPhiVett(indTag) = obj.nPhiVett(indTag) - 1;

                        ind0 = obj.xHatCumIndices(indTag+1);
                        i = ind0 + 2 + indPhi;
                        obj.P = obj.P([1:i-1, i+1:end], [1:i-1, i+1:end]);
                        obj.xHatSLAM = obj.xHatSLAM([1:i-1, i+1:end], :);
                    end
                end
            end
            if change
                nPhiTagNew = sum(obj.nPhiVett);
                stateLenNew = sum(obj.xHatIndices)-1;

                obj.innovazione = zeros(nPhiTagNew, 1);
    
                obj.F = eye(stateLenNew);
                obj.W = zeros(stateLenNew, 2);
                obj.H = zeros(nPhiTagNew, stateLenNew);
                obj.Rs = zeros(nPhiTagNew, nPhiTagNew);

                obj.Hx = zeros(nPhiTagNew, stateLenNew);
                obj.Hy = zeros(nPhiTagNew, stateLenNew);
                obj.innovazioneX = zeros(nPhiTagNew, 1);
                obj.innovazioneY = zeros(nPhiTagNew, 1);
                obj.RsX = zeros(nPhiTagNew, nPhiTagNew);
                obj.RsY = zeros(nPhiTagNew, nPhiTagNew);
            end
        end
    
        %
        function [obj] = save_history(obj)
            nTag = obj.data.nTag;
            for indTag = 1:nTag
                nPhi = obj.nPhiVett(indTag);
                ind0 = obj.xHatCumIndices(indTag+1);
                x_i   = obj.xHatSLAM(0+ind0, obj.k+1);
                y_i   = obj.xHatSLAM(1+ind0, obj.k+1);
                rho_i = obj.xHatSLAM(2+ind0, obj.k+1);

                phiTagMediato = 0.0;

                for indPhi = 1:nPhi
                    phi_ij = obj.xHatSLAM(2+ind0+indPhi, obj.k+1);
                    pesoTag = obj.pesi(indTag, indPhi);

                    phiTagMediato = phiTagMediato + phi_ij*pesoTag;
                end

                cosPhi_ij = cos(phiTagMediato);
                sinPhi_ij = sin(phiTagMediato);

                obj.xHatTagStoria(indTag, obj.k+1) = x_i + rho_i*cosPhi_ij;
                obj.yHatTagStoria(indTag, obj.k+1) = y_i + rho_i*sinPhi_ij;

                % varianze
                varXTagMediata = 0.0;
                varYTagMediata = 0.0;
                covXYTagMediata = 0.0;

                ind_x = 0+ind0;
                ind_y = 1+ind0;
                ind_r = 2+ind0;
                for indPhi = 1:nPhi
                    ind_p = 3+ind0+indPhi-1;
        
                    rho_i = obj.xHatSLAM(ind_r, obj.k+1);
                    phi_ij = obj.xHatSLAM(ind_p, obj.k+1);
                    cosPhi_ij = cos(phi_ij);
                    sinPhi_ij = sin(phi_ij);
        
                    varXi  = obj.P(ind_x, ind_x);
                    varYi  = obj.P(ind_y, ind_y);
                    varRho = obj.P(ind_r, ind_r);
                    varPhi = obj.P(ind_p, ind_p);
        
                    % covXiYi   = obj.P(ind_x, ind_y);
                    covXRho   = obj.P(ind_x, ind_r);
                    covXPhi   = obj.P(ind_x, ind_p);
                    covYRho   = obj.P(ind_y, ind_r);
                    covYPhi   = obj.P(ind_y, ind_p);
                    covRhoPhi = obj.P(ind_r, ind_p);

                    var_x = varXi + cosPhi_ij^2 * varRho + rho_i^2 * sinPhi_ij^2 * varPhi + ...
                            2*cosPhi_ij*covXRho - 2*rho_i*sinPhi_ij*covXPhi - 2*rho_i*cosPhi_ij*sinPhi_ij*covRhoPhi;

                    var_y = varYi + sinPhi_ij^2 * varRho + rho_i^2 * cosPhi_ij^2 * varPhi + ...
                            2*sinPhi_ij*covYRho + 2*rho_i*cosPhi_ij*covYPhi + 2*rho_i*cosPhi_ij*sinPhi_ij*covRhoPhi;
                    % all'inizio c'era - 2*rho_i*cosPhi_ij*sinPhi_ij*covRhoPhi
    
                    cov_xy = 0;
                            %covXiYi + cosPhi_ij*sinPhi_ij*varRho - rho_i^2*sinPhi_ij*cosPhi_ij * varPhi + ...
                            %cosPhi_ij*sinPhi_ij*covXRho - rho_i^2*sinPhi_ij*cosPhi_ij*covXPhi + ...
                            %cosPhi_ij*sinPhi_ij*covYRho - rho_i^2*sinPhi_ij*cosPhi_ij*covYPhi + ...
                            %cosPhi_ij*sinPhi_ij*covRhoPhi;

                    pesoTag = obj.pesi(indTag, indPhi);

                    varXTagMediata = varXTagMediata + var_x*pesoTag;
                    varYTagMediata = varYTagMediata + var_y*pesoTag;
                    covXYTagMediata = covXYTagMediata+ cov_xy*pesoTag;
                end

                obj.varX(indTag) = varXTagMediata;
                obj.varY(indTag) = varYTagMediata;
                obj.covXY(indTag) = covXYTagMediata;

                sigma = [ obj.varX(indTag) obj.covXY(indTag);
                         obj.covXY(indTag) obj.varY(indTag)];
                if any(eig(sigma) < 0)
                    disp("Error with eigs")
                    disp(sigma)
                    return
                end
            end
            obj.varsStoria(:, :, obj.k+1) = [obj.varX; obj.varY];
        end

        %
        function structCondivisa = data_to_share(obj)           
            tags = [obj.xHatTagStoria(:, obj.k+1) obj.yHatTagStoria(:, obj.k+1)]';
            vars = [obj.varX; obj.varY; obj.covXY];
            structCondivisa = struct('id', obj.id, 'tags', tags, 'vars', vars);
        end

        %
        function [obj] = correction_shared(obj, all_measures)
            nTag = obj.data.nTag;

            thisRobotTags = [obj.xHatTagStoria(:, obj.k+1) obj.yHatTagStoria(:, obj.k+1)];

            posTagRobot = zeros(2, nTag, 0);
            vars = zeros(2, nTag, 0);

            other_measures = struct('id', {}, 'tags', {}, 'vars', {});
            for idx = 1:length(all_measures)
                if obj.id == all_measures(idx).id
                    continue
                end
                other_measures(end+1) = all_measures(idx);
            end

            if length(other_measures) < 2
                return;
            end

            inliersGood = zeros(2, nTag, length(other_measures));
            empty = 1;
            for robot1 = 1:length(other_measures)-1
                tags1 = other_measures(robot1).tags;
                for robot2 = robot1+1:length(other_measures)
                    tags2 = other_measures(robot2).tags;
                    [~, ~, inliers] = ransacRototranslation(tags1', tags2', obj.data.numIterationsA, obj.data.distanceThresholdA, round(obj.data.percentMinInliersA * nTag));
                    if ~isempty(inliers)
                        empty = 0;
                        inliersGood(:, inliers, robot1) = 1;
                        inliersGood(:, inliers, robot2) = 1;
                    end
                end
            end

            if empty
                return
            end

            to_be_removed = [];

            for idx = 1:length(other_measures)
                otherRobotTags = other_measures(idx).tags;
                [R, t, inliers] = ransacRototranslation(otherRobotTags', thisRobotTags, obj.data.numIterationsB, obj.data.distanceThresholdB, round(obj.data.percentMinInliersB * nTag));
                if isempty(inliers)
                    to_be_removed(end+1) = idx;
                    continue
                end

                T = [R, t; zeros(1, 2), 1];

                % Applica rototraslazione alle posizioni dei tag
                otherRobotTagsRot = T*[otherRobotTags; ones(1, nTag)];
                posTagRobot(:, :, end+1) = otherRobotTagsRot(1:2, :);

                % Applica rotazione alle varianze
                for indTag = 1:nTag
                    varX_ = other_measures(idx).vars(1, indTag);
                    varY_ = other_measures(idx).vars(2, indTag);
                    covXY_ = other_measures(idx).vars(3, indTag);
                    Sigma = [ varX_, covXY_; 
                             covXY_,  varY_];
        
                    Sigma_prime = R * Sigma * R';
                    varX_prime = Sigma_prime(1,1);
                    varY_prime = Sigma_prime(2,2);
        
                    vars(:, indTag, size(posTagRobot, 3)) = [varX_prime, varY_prime]';
                end
            end

            % check reset
            if isempty(posTagRobot)
                if obj.data.reset && sum(obj.nPhiVett == 1) >= 3
                    if length(other_measures) > 1 || other_measures(1).id ~= obj.id
                        obj.nReset = obj.nReset + 1;
                    end
                    if obj.nReset >= obj.data.resetThr
                        obj.do_reset = 1;
                    end
                end

                return
            end
            obj.nReset = 0;

            % vars = (0 * vars + 1);    % FIXME
            % temp = 1 ./ vars;
            % W_ = temp ./ sum(temp, 3);

            inliersGood(:, :, to_be_removed) = [];
            den = sum(inliersGood, 3);
            den(den == 0) = 1;
            W_ = inliersGood ./ den;
            measures_weighted = sum(W_ .* posTagRobot, 3);

            tags_to_use = sum(W_(1, :, :), 3);
            if all(tags_to_use == 0)
                return
            end

            for idx_pos = 1%:size(posTagRobot, 3)       % fare la media sembra funzionare meglio
                % obj.restore_matrices();
                indMatCum = cumsum([0 obj.nPhiVett(1:end-1)]);
                change = 0;
                % pos = posTagRobot(:, :, idx_pos);
                for indTag = 1:nTag
                    nPhi = obj.nPhiVett(indTag);
                    indMat = indMatCum(indTag) - change;
                    if tags_to_use(indTag) == 0
                        i = indMat + 1;
                        for indPhi = 1:nPhi
                            change = change + 1;

                            obj.Hx = obj.Hx([1:i-1, i+1:end], :);
                            obj.Hy = obj.Hy([1:i-1, i+1:end], :);

                            obj.innovazioneX(i) = [];
                            obj.innovazioneY(i) = [];

                            obj.RsX = obj.RsX([1:i-1, i+1:end], [1:i-1, i+1:end]);
                            obj.RsY = obj.RsY([1:i-1, i+1:end], [1:i-1, i+1:end]);

                            indMat = indMat - 1;
                        end
                        continue
                    end

                    % var_ = 0.15;                                         % varianza statica
                    % fused_var_x = var_;
                    % fused_var_y = var_;
                    % fused_var_x = 100*vars(1, indTag, idx_pos);           % singole misure
                    % fused_var_y = 100*vars(2, indTag, idx_pos);
                    % fused_var_x = 1 / sum(1 ./ vars(1, indTag, :));   % media misure
                    % fused_var_y = 1 / sum(1 ./ vars(2, indTag, :));
    
                    % sigmaX = sqrt(fused_var_x);
                    % sigmaY = sqrt(fused_var_y);

                    sigmaX = obj.data.sigmaMisuraMedia;
                    sigmaY = obj.data.sigmaMisuraMedia;
    
                    ind0 = obj.xHatCumIndices(indTag+1);
                    x_i   = obj.xHatSLAM(0+ind0, obj.k+1);
                    y_i   = obj.xHatSLAM(1+ind0, obj.k+1);
                    rho_i = obj.xHatSLAM(2+ind0, obj.k+1);
    
                    % misuraX_ij = pos(1, indTag);
                    % misuraY_ij = pos(2, indTag);
                    misuraX_ij = measures_weighted(1, indTag);
                    misuraY_ij = measures_weighted(2, indTag);
    
                    probMisuraX_ij = zeros(nPhi, 1);
                    probMisuraY_ij = zeros(nPhi, 1);
    
                    for indPhi = 1:nPhi
                        phi_ij = obj.xHatSLAM(2+ind0+indPhi, obj.k+1);
                        cosPhi_ij = cos(phi_ij);
                        sinPhi_ij = sin(phi_ij);
                        xTag_ij = x_i + rho_i*cosPhi_ij;
                        yTag_ij = y_i + rho_i*sinPhi_ij;
                        deltaMisuraX_ij = misuraX_ij - xTag_ij;
                        deltaMisuraY_ij = misuraY_ij - yTag_ij;

                        obj.innovazioneX(indMat+indPhi) = deltaMisuraX_ij;
                        obj.innovazioneY(indMat+indPhi) = deltaMisuraY_ij;
                        probMisuraX_ij(indPhi) = exp(-deltaMisuraX_ij^2/(2*sigmaX^2));
                        probMisuraX_ij(indPhi) = max(probMisuraX_ij(indPhi), 1e-100);
                        probMisuraY_ij(indPhi) = exp(-deltaMisuraY_ij^2/(2*sigmaY^2));
                        probMisuraY_ij(indPhi) = max(probMisuraY_ij(indPhi), 1e-100);
                        obj.pesi(indTag, indPhi) = obj.pesi(indTag, indPhi)*probMisuraX_ij(indPhi)*probMisuraY_ij(indPhi);
                        obj.pesi(indTag, indPhi) = max(obj.pesi(indTag, indPhi), 1e-100);
                    
                        obj.Hx(indMat+indPhi, 0+ind0) = 1;
                        obj.Hy(indMat+indPhi, 1+ind0) = 1;
                        obj.Hx(indMat+indPhi, 2+ind0) = cosPhi_ij;
                        obj.Hy(indMat+indPhi, 2+ind0) = sinPhi_ij;
                        obj.Hx(indMat+indPhi, 2+ind0+indPhi) = -rho_i*sinPhi_ij;
                        obj.Hy(indMat+indPhi, 2+ind0+indPhi) =  rho_i*cosPhi_ij;
                    end
                    lambdaX_ij = probMisuraX_ij/sum(probMisuraX_ij);
                    lambdaY_ij = probMisuraY_ij/sum(probMisuraY_ij);
    
                    % Matrice covarianza misure con formula che tiene conto dell'Information Sharing
                    for indPhi = 1:nPhi
                        obj.RsX(indMat+indPhi, indMat+indPhi) = sigmaX^2/(max(0.0001, lambdaX_ij(indPhi)));
                        obj.RsY(indMat+indPhi, indMat+indPhi) = sigmaY^2/(max(0.0001, lambdaY_ij(indPhi)));
                    end
                end
            end
            
            % Aggiornamento stima (stima a posteriori)
            Htot = [obj.Hx; obj.Hy];
            RsTot = blkdiag(obj.RsX, obj.RsY);
            innovazioneTot = [obj.innovazioneX; obj.innovazioneY];
            KalmanGain = obj.P*Htot'*pinv(Htot*obj.P*Htot'+RsTot);
            obj.xHatSLAM(:, obj.k+1) = obj.xHatSLAM(:, obj.k+1) + KalmanGain*innovazioneTot;
            obj.P = (eye(sum(obj.xHatIndices)-1) - KalmanGain*Htot)*obj.P;
    
            % Aggiornamento pesi
            obj.pesi = obj.pesi ./ sum(obj.pesi, 2);

            if change > 0
                obj.restore_matrices();
            end
        end

        %
        function [obj] = restore_matrices(obj)
            % restore previous dimensions
            nPhiTagNew = sum(obj.nPhiVett);
            stateLenNew = sum(obj.xHatIndices)-1;

            obj.Hx = zeros(nPhiTagNew, stateLenNew);
            obj.Hy = zeros(nPhiTagNew, stateLenNew);
            obj.innovazioneX = zeros(nPhiTagNew, 1);
            obj.innovazioneY = zeros(nPhiTagNew, 1);
            obj.RsX = zeros(nPhiTagNew, nPhiTagNew);
            obj.RsY = zeros(nPhiTagNew, nPhiTagNew);
        end

        %
        function [obj] = reset(obj)
            data_ = obj.data;

            nTag = data_.nTag;
            nPhiMax = data_.nPhi;
            obj.nPhiVett = data_.nPhi*ones(1, nTag);
            nPassi = data_.nPassi;

            obj.innovazione = zeros(nTag*nPhiMax, 1);
            obj.pesi = (1/nPhiMax)*ones(nTag, nPhiMax);
            pose = obj.xHatSLAM(1:3, :);
            obj.xHatSLAM = zeros(3+(3+nPhiMax)*nTag, nPassi);
            obj.xHatSLAM(1:3, :) = pose;
            obj.P = zeros(3+(3+nPhiMax)*nTag, 3+(3+nPhiMax)*nTag);
            obj.Ptag = diag([0, 0, obj.sigmaD^2, obj.sigmaPhi^2*ones(1, nPhiMax)]);

            obj.F = eye(3+(3+nPhiMax)*nTag);
            obj.W = zeros(3+(3+nPhiMax)*nTag, 2);
            obj.H = zeros(nTag*nPhiMax, 3+(3+nPhiMax)*nTag);
            obj.Rs = zeros(nTag*nPhiMax, nTag*nPhiMax);

            obj.xHatIndices = [1 3 (3+nPhiMax)*ones(1, nTag)];
            obj.xHatCumIndices = cumsum(obj.xHatIndices(1:end-1));

            obj.Hx = zeros(nTag*nPhiMax,3+(3+nPhiMax)*nTag);
            obj.Hy = zeros(nTag*nPhiMax,3+(3+nPhiMax)*nTag);
            obj.innovazioneX = zeros(nTag*nPhiMax, 1);
            obj.innovazioneY = zeros(nTag*nPhiMax, 1);
            obj.RsX = zeros(nTag*nPhiMax, nTag*nPhiMax);
            obj.RsY = zeros(nTag*nPhiMax, nTag*nPhiMax);

            % Inizializzazione
            obj.xHatSLAM(1:3, obj.k+1) = [0, 0, 0];
            obj.xHatSLAM(4:nPhiMax+3:end, obj.k+1) = obj.xHatSLAM(1, obj.k+1);
            obj.xHatSLAM(5:nPhiMax+3:end, obj.k+1) = obj.xHatSLAM(2, obj.k+1);
            obj.xHatSLAM(6:nPhiMax+3:end, obj.k+1) = obj.lastMeasures;
            for jndPhi = 1:nPhiMax
                obj.xHatSLAM(6+jndPhi:nPhiMax+3:end, obj.k+1) = data_.possibiliPhi(jndPhi);
            end
            for indTag = 1:nTag    
                obj.P(4+(3+nPhiMax)*(indTag-1):3+(3+nPhiMax)*indTag, 4+(3+nPhiMax)*(indTag-1):3+(3+nPhiMax)*indTag) = obj.Ptag;
            end

            obj.startPruning = zeros(1, nTag);

            obj.varX = zeros(1, nTag);
            obj.varY = zeros(1, nTag);
            obj.covXY = zeros(1, nTag);

            obj.nReset = 0;
            %obj.data.resetThr = ;
            obj.do_reset = 0;
        end

        %
        function structCondivisa = globalize(obj, all_measures)
            nTag = obj.data.nTag;

            thisRobotTags = [obj.xHatTagStoria(:, obj.k+1) obj.yHatTagStoria(:, obj.k+1)]';

            posTagRobot = zeros(2, nTag, 0);

            other_measures = struct('id', {}, 'tags', {}, 'vars', {});
            for idx = 1:length(all_measures)
                if obj.id == all_measures(idx).id
                    continue
                end
                other_measures(end+1) = all_measures(idx);
            end

            N = length(other_measures);
            if N < 2
                return;
            end

            Ts = zeros(3, 3, N);
            for idx = 1:N
                otherRobotTags = other_measures(idx).tags;
                [R, t, inliers] = ransacRototranslation(otherRobotTags', thisRobotTags', obj.data.numIterationsB, obj.data.distanceThresholdB, round(obj.data.percentMinInliersB * nTag));
                if isempty(inliers)
                    continue
                end

                T = [R, t; zeros(1, 2), 1];
                Ts(:, :, idx) = T;

                otherRobotTagsRot = T*[otherRobotTags; ones(1, nTag)];
                posTagRobot(:, :, end+1) = otherRobotTagsRot(1:2, :);
            end
            if size(posTagRobot, 3) == 0
                structCondivisa = struct('ids', [], 'use_avg', [], 'pose', [], 'tags', []);
                return
            end

            point1_idx = 0;
            point2_idx = 0;

            use_max_distance = 1;
            if use_max_distance
                % Max distance
                max_distance = 0;
                
                for i = 1:N
                    for j = i+1:N
                        dist = sqrt((posTagRobot(1, i) - posTagRobot(1, j))^2 + (posTagRobot(2, i) - posTagRobot(2, j))^2);
                        if dist > max_distance
                            max_distance = dist;
                            point1_idx = i;
                            point2_idx = j;
                        end
                    end
                end
            else
                % Min sigma
                sigma_pos = sqrt(var(posTagRobot, 0, 3));
                sigma_pos_comb = sqrt(sigma_pos(1, :).^2 + sigma_pos(2, :).^2);
                [~, point1_idx] = min(sigma_pos_comb);
                sigma_combined_excluded = sigma_pos_comb;
                sigma_combined_excluded(point1_idx) = Inf;
                [~, point2_idx] = min(sigma_combined_excluded);
            end

            use_avg = 1;
            if use_avg
                mean_pos = mean(posTagRobot, 3);
                point1 = mean_pos(:, point1_idx);
                point2 = mean_pos(:, point2_idx);
            else
                point1 = thisRobotTags(:, point1_idx);
                point2 = thisRobotTags(:, point2_idx);
            end

            t = point1;
            s = (point2(2)-point1(2)) / norm(point2-point1);
            c = (point2(1)-point1(1)) / norm(point2-point1);
            T = [c -s t(1); s c t(2); 0 0 1]^-1;

            global_tags = T * [thisRobotTags; ones(1, nTag)];
            global_pos = T * [obj.xHatSLAM(1:2, obj.k+1); 1];
            global_angle = obj.xHatSLAM(3, obj.k+1) + atan2(s, c);

            structCondivisa = struct('ids', [point1_idx, point2_idx], ...
                                     'use_avg', use_avg, ...
                                     'pose', [global_pos(1:2); global_angle], ...
                                     'tags', global_tags(1:2, :));
        end
    end % methods
end % class