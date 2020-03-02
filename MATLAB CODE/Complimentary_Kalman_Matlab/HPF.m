clear all; close all;

%%%%% Data read from the file %%%%%
data=xlsread('dataa.csv');
accx=data(:,1);
accy=data(:,2);
accz=data(:,3);

%calibration stage%
gyrox=data(:,4);
gyroy=data(:,5);
gyroz=data(:,6);

% data=xlsread('DATA.csv');
% accx=data(:,2);
% accy=data(:,3);
% accz=data(:,4);
% 
% %calibration stage%
% gyrox=data(:,5);
% gyroy=data(:,6);
% gyroz=data(:,7);

% clculate mean bias value %
meangyrox = sum(gyrox(1:100))/100;
meangyroy = sum(gyroy(1:100))/100;
meangyroz = sum(gyroz(1:100))/100;

% true value = measured - Mean baised value%
gyroy = gyroy-meangyroy;
%sampling rate%
Ts=1/200;   %delta t from the Data rate

%compute drift in a loop%
angy(1)=0;
for i=1:length(gyroy)
    if i< length(gyroy)
        angy(i+1) = angy(i) + gyroy(i+1)*Ts; %integrationg the gyroscope
    end
end

% plotting angles value from Gyroscope
plot(angy, '-');
hold on;
anggyr = angy;


%%%Filter Coefficient%%%
%fc is cutoff frequency
%fc = 1/(2*pi*RC);
%tau = RC - 1/(2*pi*fc);
%alphp = tau/(tau + Ts);
alphp = 0.8641;

%%%pass the signal from High Pass Filter %%%
anggyryhp(1) = 0;
for i=2:length(gyrox)
    anggyryhp(i)= (alphp*anggyryhp(i-1) + alphp*(anggyr(i)-anggyr(i-1)));
end

%%plot High Pass Filter %%%
hold on;
plot(anggyryhp, 'k-');
legend('Angle using raw gyro','Angle passed through HPF');
xlabel('Data Sample');
ylabel('Angles');
title('Drift Correction');
