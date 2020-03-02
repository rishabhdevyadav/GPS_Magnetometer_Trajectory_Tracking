clear all; close all;

%%%%% Data read from the file %%%%%
data=xlsread('imu_ros2.xlsx');
%save imu_ros_mat2.mat data
accx=data(1:end,2);
accy=data(1:end,3);
accz=data(1:end,4);

gyrox=data(1:end,5);
gyroy=data(1:end,6);
gyroz=data(1:end,7);

magx=data(1:end,8);
magy=data(1:end,9);
magz=data(1:end,10);

cam=data(1:end,12);
for i=1:length(cam)
   cam(i)=-cam(i)+305;
    if cam(i)<0
        cam(i)=cam(i)+360;
    end    
end
for i=5:length(cam)
    n=int32(i*3524/35236); 
    cam_new(n)= cam(i);
end
%subplot(2,2,1)
plot((cam_new),'-g');
title('cam vs magneto vs compliment')
hold on;

%Calibarte
% meanaccx = sum(accx(1:100))/100;
% meanaccy = sum(accy(1:100))/100;
% meanaccz = sum(accz(1:100))/100;
% 
% meangyrox = sum(gyrox(1:100))/100;
% meangyroy = sum(gyroy(1:100))/100;
% meangyroz = sum(gyroz(1:100))/100;
% 
% meanmagx = sum(magx(1:100))/100;
% meanmagy = sum(magy(1:100))/100;
% meanmagz = sum(magz(1:500))/500;

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

%    pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
%    roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));
%    // yaw from mag
%    float Yh = (magY * cos(roll)) - (magZ * sin(roll));
%    float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch));
%    yaw =  atan2(Yh, Xh);
%     roll = roll*57.3;
%     pitch = pitch*57.3;
%     yaw = yaw*57.3;

% for i=1:length(gz)
%    pitch(i) = atan2 (ay(i) ,( sqrt ((ax(i) * ax(i)) + (az(i) * az(i)))));
%    roll(i) = atan2(-ax(i) ,( sqrt((ay(i) * ay(i)) + (az(i) * az(i)))));
%    %yaw from mag
%    Yh(i) = (my(i) * cos(roll(i))) - (mz(i) * sin(roll(i)));
%    Xh(i) = (mx(i) * cos(pitch(i)))+(my(i) * sin(roll(i))*sin(pitch(i))) + (mz(i) * cos(roll(i)) * sin(pitch(i)));
%    yaw(i) =  atan2(Yh(i), Xh(i));
%      if  (yaw(i)<0)
%             yaw(i)=yaw(i)+(2*pi);
%      end    
%     roll(i) = roll(i)*57.3;
%     pitch(i) = pitch(i)*57.3;
%     yaw(i) = yaw(i)*57.3;   
% end
%plot(yaw, 'r-');

%%%%direct division
for i=1:length(mz)
    if i<= length(mz)
        bearing(i) = atan2(my(i),mx(i)); 
        if  (bearing(i)<0)
            bearing(i)=bearing(i)+(2*pi);
        end
    end
    bearing_deg(i)=radtodeg(bearing(i));
end
%plot(bearing_deg, 'k-');

% alplp=0.5;
% bearing_deg_lpf(1)=bearing_deg(1);
% for i=2:length(gz)
%     bearing_deg_lpf(i) = alplp*bearing_deg(i)+((1-alplp)*bearing_deg_lpf(i-1));
% end
% %plot(bearing_deg_lpf, 'b-')


Ts=1/5.5;
yaw_raw(1) = atan2(my(1),mx(1));
yaw_raw(2) = atan2(my(2),mx(2));
for i=3:length(gz)
    if i<= length(gz)
        %yaw_raw(i) = yaw_raw(i-1) + gz(i)*Ts; %step integrationg the gyroscope
        yaw_raw(i) = yaw_raw(i-1) + ((gz(i-2)+gz(i-1))/2)*Ts; %trapezoidal integration
        %for trapezoidal start i with 3;
         if  (yaw_raw(i)<0)
              yaw_raw(i)=yaw_raw(i)+(2*pi);
         end
         if  (yaw_raw(i)>deg2rad(360))
              yaw_raw(i)=yaw_raw(i)-(2*pi);
        end
    end
    yaw_raw_deg(1)=rad2deg(yaw_raw(1));
    yaw_raw_deg(i)=rad2deg(yaw_raw(i));
end
%plot(yaw_raw_deg, 'g-')


% 
% alphp = 0.85;
% yaw_hp(1) = bearing_deg_lpf(1); 
% for i=2:length(gz)
%     yaw_hp(i)= (alphp*yaw_hp(i-1) + alphp*(yaw_raw_deg(i)-yaw_raw_deg(i-1)));
%           if  (yaw_hp(i)<0)
%              yaw_hp(i)=yaw_hp(i)+360;
%           end
%           if (yaw_hp(i)>360)
%              yaw_hp(i)=yaw_hp(i)-360;
%           end
% end
% %plot(yaw_hp, '-')
% 
% 
% alpha_complimentary=0.9;
% for i=1:length(gz)
% yaw_complimentary(i) =((1 - alpha_complimentary) * yaw_hp(i))   + (alpha_complimentary * bearing_deg(i));
% end
% %plot(yaw_complimentary, 'b-')
% 
%  alpha_complimentary_filt=0.85;
%  for i=1:length(gz)
%  yaw_complimentary_filt(i)   = ((1 - alpha_complimentary_filt) * yaw_raw_deg(i))   + (alpha_complimentary_filt * bearing_deg_lpf(i));
%  end
% %plot(yaw_complimentary_filt, 'b-')



% for i=1:3520
% Error1(i)=abs(cam_new(i)-yaw_complimentary_filt(i));
% Error2(i)=abs(yaw_complimentary_filt(i)-cam_new(i));
% Error1_new(i)=min([Error1(i) Error2(i)]);
% Error2(i)=cam_new(i)-yaw(i);
% Error3(i)=yaw(i)-yaw_complimentary_filt(i);
% end
% subplot(2,2,2)
% plot(Error1_new);
% title('Error1: cam vs compliment')
% subplot(2,2,3)
% plot(Error2);
% title('Error2: cam vs magneto')
% subplot(2,2,4)
% plot(Error3);
% title('Error3: magneto vs compliment')

