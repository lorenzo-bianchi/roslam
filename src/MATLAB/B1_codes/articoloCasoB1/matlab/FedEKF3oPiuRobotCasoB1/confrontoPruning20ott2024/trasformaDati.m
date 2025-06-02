% Permette di passare dai dati dopo le 100 simulazioni a quelli necessari
% per fare le fig con faiFig.m. Se li ho salvati in tutto.mat carico questi
% dati e poi li salvo in un file mat piu' piccolo che contiene solo le
% variabili necessarie per fare le fig.

% load tutto
erroreMedioAssolutoAltriRobotDi5storia = mean(erroriMediAssolutiAltriRobotStoria);
erroreMedioAssolutoRobot1di5storia = mean(erroriAssolutiRobot1Storia);
erroriAssoluti5robot = erroreAssolutoRobotMat;
erroriAssolutiTag5robot = erroreAssolutoTagMat;
mediaErroriMediAssolutiTagStoriaCinqueRobot = mean(erroriMediAssolutiTagStoria);
stdErroreMedioAssolutoAltriRobotDi5storia = std(erroriMediAssolutiAltriRobotStoria);
stdErroreMedioAssolutoRobot1di5storia = std(erroriAssolutiRobot1Storia);
stdErroriMediAssolutiTagStoriaCinqueRobot = std(erroriMediAssolutiTagStoria);
save confrontoPruning200treRobotCon0p5inizCentraleSenzaOttimizzGrafo erroriAssoluti5robot erroriAssolutiTag5robot erroriAssolutiTag5robot erroreMedioAssolutoRobot1di5storia erroreMedioAssolutoAltriRobotDi5storia stdErroreMedioAssolutoRobot1di5storia stdErroreMedioAssolutoAltriRobotDi5storia mediaErroriMediAssolutiTagStoriaCinqueRobot stdErroriMediAssolutiTagStoriaCinqueRobot