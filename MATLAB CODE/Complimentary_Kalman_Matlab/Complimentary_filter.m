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



% data=xlsread('.csv');
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

gyry = gyroy-meangyroy;

%sampling rate%
Ts=1/200;   %delta t from the Data rate
%take intial angle for gyroscope from the accelerometer
angacc(1)=atan2(accx(1),sqrt((accy(1)*accy(1)) + (accz(1)*accz(1))))*(180/pi);
anggyr(1)=atan2(accx(1),sqrt((accy(1)*accy(1)) + (accz(1)*accz(1))))*(180/pi);

%Complimentary filter without HPf or LPF 
for i = 2:length(accx)
    %if compute from gyro alone
    anggyr(i) = anggyr(i-1) + (gyry(i))*(Ts);
    angacc(i) = atan2(accx(i), sqrt((accy(i)*accy(i)) + (accz(i)*accz(i))))*(180/pi);
    %combine to form the complimentary filter, where equal weightage to
    %both accelerometer an and gyroscope
    angcf(i) = anggyr(i)*0.02 + 0.98*angacc(i);
end

figure;
plot(anggyr)
hold on;
plot(angacc,'g')
hold on;
plot(angcf,'r')
legend('Gyro-Angle', 'Accel-Angle','Complimementary without pre-filtering')


%LPF HPF then complimentary filter

alphp=0.8641;
alplp = 0.0909;

%%%Intialise the value %%%%
accxlp(1) = accx(1);
accylp(1) = accy(1);
acczlp(1) = accz(1);

xlpfiltx(1) = accx(1);
xlpfilty(1) = accy(1);
xlpfiltz(1) = accz(1);

anggyr(1)=angacc(1);

angaccllp(1)=angacc(1);
angacclhp(1)=angacc(1);
anggyryhp(1)=angacc(1);
for i=2:length(gyrox)
    
    angaccl(i) = atan2(accx(i),sqrt((accy(i)*accy(i)) + (accz(i)*accz(i))))*(180/pi);
    
    %low pass filter in accelerometer data
     xlpfiltx(i) = (1-alplp)*xlpfiltx(i-1) + alplp*accx(i);
     xlpfilty(i) = (1-alplp)*xlpfilty(i-1) + alplp*accy(i);
     xlpfiltz(i) = (1-alplp)*xlpfiltz(i-1) + alplp*accz(i);
      
     %compute gyro angle
     anggyryhp(i) = (alphp*anggyryhp(i-1) + alphp*(anggyr(i)-anggyr(i-1)));
     
     %compute accel angle where the accel data is low passed
    angaccel(i) = atan2(xlpfiltx(i),sqrt((xlpfilty(i)*xlpfilty(i)) + (xlpfiltz(i)*xlpfiltz(i))))*180/pi;
     
     %%%combine by complimentary filter
     angcffilt(i)=anggyryhp(i)*0.1 + 0.9*angaccl(i);
     
end
% 
figure;
% plot(anggyryhp);
% hold on;
% plot(angaccl,'g');

plot(anggyryhp)
hold on;
plot(angaccl,'g')
hold on;
plot(angcffilt,'r')
legend('Gyro-Angle', 'Accel-Angle','Complimementary pre-filtering')

