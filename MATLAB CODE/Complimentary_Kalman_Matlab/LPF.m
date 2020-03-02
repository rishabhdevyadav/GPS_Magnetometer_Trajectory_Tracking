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
gyrox = gyrox-meangyrox;
gyroy = gyroy-meangyroy;
gyroz = gyroz-meangyroz;
%sampling rate%
Ts=1/200;   %delta t from the Data rate

for i= 1:length(accx)
    angacc(i)=atan2(accx(i),sqrt((accy(i)*accy(i))+accz(i)))*(180/pi);
end

%%%Filter Coefficient%%%
%fc is cutoff frequency
%fc = 1/(2*pi*RC);
%tau = RC - 1/(2*pi*fc);
%alplp = Ts/(tau + Ts);
alplp = 0.0909;

%%%%%pass the signal through the Low Pass Filter%%%
angxlpf(1)=angacc(1);
for i=2:length(accx)
    angxlpf(i) = alplp*angacc(i)+((1-alplp)*angxlpf(i-1));
end

%%%%%%plotting the angles derived from gyroscope%%%%
plot(angacc,'r-');
hold on;
plot(angxlpf,'y-');

legend('Angle using raw Accel','Angle passed though LPF');
xlabel('Data Samples');
ylabel('Angles');
title('Jitter Corection');
