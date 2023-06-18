A = [1 1000 1000^2;
     1 1300 1300^2;
     1 1600 1600^2];
b = [0.0854858; 1.36777; 4.1888];

%gazebo iril model parameter
% A = [1 1480 1480^2;
%      1 1638 1638^2;
%      1 1770 1770^2];
% b = [0.75*9.81/4; 1.25*9.81/4; 1.75*9.81/4];

%test quantrotor
A = [1 1272.2 1272.2^2;
     1 1347.7 1347.7^2;
     1 1442.2 1442.2^2];
b = [0.95*9.81/4; 1.20*9.81/4; 1.50*9.81/4];

c = inv(A)*b;
c1 = c(3)
c2 = c(2)
c3 = c(1)
% c1 = 0.000008492875480;
% c2 = -0.002777454295627;
% c3 = 0.108400169428593;
% c1 = 4.623452619996466e-06;
% c2 = 0.0014142896764618592;
% c3 = -1.0321173041040559;

thrust = 0;
thrust_max = 10;
h = 0.0001;
Step = 1;
while(thrust < thrust_max)
    thrust = thrust + h;
    PWM = -c2/(2*c1) + 1/(2*c1)*sqrt((c2)^2 - 4*c1*(c3 - thrust));
    
    arraythrust(1,Step) = thrust;
    arrayPWM(1,Step) = PWM;
    Step = Step + 1;
end

figure(1);
plot(arrayPWM,arraythrust,'r','LineWidth',1);
xlabel('PWM','FontSize',16);
ylabel('Thrust','FontSize',16);