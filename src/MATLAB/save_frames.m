% Imposta i parametri
videoFile = '/media/lorenzo/52387916-e258-4af3-95a4-c8701e29a684/@home/lorenzo/Desktop/DJI/B2_test8.MP4'; % Percorso completo del video
outputFolder = 'frames_output'; % Cartella principale di output
resizeFactor = 0.5; % Fattore di ridimensionamento (es. 0.5 per metà dimensione)

% Ottieni il nome base del file senza estensione
[~, baseFileName, ~] = fileparts(videoFile);

% Crea la cartella principale di output se non esiste
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

% Crea la sottocartella con il nome del file video
subFolder = fullfile(outputFolder, baseFileName);
if ~exist(subFolder, 'dir')
    mkdir(subFolder);
end

% Carica il video
seconds = 25;
video = VideoReader(videoFile);
frameRate = video.FrameRate; % Ottiene il frame rate dal video
totalFrames = floor(frameRate * video.Duration);
startFrames = floor(seconds * frameRate); % Primi 10 secondi
endFrames = totalFrames - startFrames; % Ultimi 10 secondi

% Processa i primi e ultimi 10 secondi
processFrames(video, 1, startFrames, subFolder, resizeFactor, frameRate);
processFrames(video, endFrames, totalFrames, subFolder, resizeFactor, frameRate);

disp('Elaborazione completata.');

function processFrames(video, startFrame, endFrame, outputFolder, resizeFactor, frameRate)
    video.CurrentTime = (startFrame - 1) / frameRate;
    
    for frameIdx = startFrame:endFrame
        % Legge il frame
        if hasFrame(video)
            frame = readFrame(video);
        else
            break;
        end
        
        % Ridimensiona il frame
        frame = imresize(frame, resizeFactor);
        
        % Calcola il timestamp con centesimi di secondo
        timestamp = sprintf('%.2f', frameIdx / frameRate);
        
        % Crea il nome del file con il nome del video
        fileName = fullfile(outputFolder, sprintf('%s.jpg', timestamp));
        
        % Salva l'immagine
        imwrite(frame, fileName);
    end
end
