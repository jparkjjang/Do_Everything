clear my_tello;

my_tello = ryze()

battery_level = my_tello.BatteryLevel;
disp(['Battery Level: ', num2str(battery_level)]);

%이륙
takeoff(my_tello); 
%1. roll축 제어
moveleft(my_tello, 2)
%2. yaw 축 제어
turn(my_tello, deg2rad(30)) 
%3. pitch축 제어
moveforward(my_tello, 1) 
%4. yaw축 제어
turn(my_tello, deg2rad(60)) 
%5. pitch축 제어
moveforward(my_tello, 1) 
%6. 사진 촬영
tello_cam = camera(my_tello)
img = snapshot(tello_cam);

imshow(img)

%7. yaw축 제어
turn(my_tello, deg2rad(-30))
%8. roll축 제어
moveright(my_tello, 1)

%착륙
land(my_tello);