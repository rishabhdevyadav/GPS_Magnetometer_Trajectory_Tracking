clear all; close all;

%%%%% Data read from the file %%%%%
data=xlsread('imu_ros.xlsx');
accx=data(:,2);
accy=data(:,3);
accz=data(:,4);

gyrox=data(:,5);
gyroy=data(:,6);
gyroz=data(:,7);

magx=data(:,8);
magy=data(:,9);
magz=data(:,10);

%Calibarte
meanaccx = sum(accx(1:100))/100;
meanaccy = sum(accy(1:100))/100;
meanaccz = sum(accz(1:100))/100;

meangyrox = sum(gyrox(1:100))/100;
meangyroy = sum(gyroy(1:100))/100;
meangyroz = sum(gyroz(1:100))/100;

meanmagx = sum(magx(1:100))/100;
meanmagy = sum(magy(1:100))/100;
meanmagz = sum(magz(1:100))/100;

%Subtartct
ax = accx;
ay = accy;
az = accz;

gx=gyrox;
gy=gyroy;
gz=gyroz;

mx=magx;
my=magy;
mz=magz;

%we will only focus on yaw
Ts=1/20;   %delta t from the Data rate

%take intial angle for gyroscope from the accelerometer
%anggyr(1)=atan2(accx(1),sqrt((accy(1)*accy(1)) + (accz(1)*accz(1))))*(180/pi);

%compute drift in a loop%
 yaw_raw(1) = atan2(my(1),mx(1));
for i=2:length(gz)
    if i<= length(gz)
        yaw_raw(i) = yaw_raw(i-1) + gz(i)*Ts; %integrationg the gyroscope
         if  (yaw_raw(i)<0)
              yaw_raw(i)=yaw_raw(i)+(2*pi);
        end
   end
    yaw_raw_deg(i)=rad2deg(yaw_raw(i));
end

plot(yaw_raw_deg, '-');
%hold on;
yaw = yaw_raw;
alphp = 0.9;

yaw_hp(1) = atan2(my(1),mx(1)); ;
for i=2:length(gz)
    yaw_hp(i)= (alphp*yaw_hp(i-1) + alphp*(yaw(i)-yaw(i-1)));
         if  (yaw_hp(i)<0)
            yaw_hp(i)=yaw_hp(i)+(2*pi);
         end
     yaw_hp_deg(i)=rad2deg(yaw_hp(i));
end


%plot(yaw_hp_deg, 'k-');
% legend('Angle using raw gyro','Angle passed through HPF');
% xlabel('Data Sample');
% ylabel('Angles');
% title('Drift Correction');


