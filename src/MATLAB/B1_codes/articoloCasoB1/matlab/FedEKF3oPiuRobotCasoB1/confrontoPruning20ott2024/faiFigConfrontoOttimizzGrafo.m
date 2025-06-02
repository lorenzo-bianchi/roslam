load confrontoPruning200cinqueRobotCon0p5

nTag = 4;
[nSim,nRobot] = size(erroriAssoluti5robot);

figure % Fig. 1
plot(sort(erroriAssoluti5robot(:,1)),'k')
hold on

figure % Fig. 2
plot(erroreMedioAssolutoRobot1di5storia,'k')
hold on


load confrontoPruning200cinqueRobotCon0p5senzaOttimizzGrafo

figure(1)
plot(sort(erroriAssoluti5robot(:,1)),'r--')
xlabel('Simulation')
ylabel('Final Robot 1 errors [cm]')
grid on
legend('With Optimization','Without Optimization')


figure(2)
plot(erroreMedioAssolutoRobot1di5storia,'r--')
xlabel('Step k')
ylabel('Robot 1 errors [cm]')
grid on
legend('With Optimization','Without Optimization')



