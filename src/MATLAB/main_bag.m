clc; clear; close all;

load('./data/postprocess_20250406_test2.mat', 'roslam_data');

dati_bag

ekfs = FedEkfBag.empty(nRobot, 0);
TsGL = cell(1, nRobot);
TsLG = cell(1, nRobot);
for robot = 1:nRobot
    TsGL{robot} = zeros(3, 3, 0);
    TsLG{robot} = zeros(3, 3, 0);
end

main_simulazione_bag;

%% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
% displayErrori = 1;
% calcolo_errori;

%% Grafici
grafici_bag;