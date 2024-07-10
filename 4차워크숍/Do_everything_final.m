clear all;
clc;


% 카메라 파라미터 설정
focalLength = [957.1698 963.1543];
principalPoint = [495.3551 394.4279];
imageSize = [720 960];
radialDistortion = [-0.0337 0.1373];
tangentialDistortion = [0.0058 0.0034];

% IntrinsicMatrix 구성
intrinsicMatrix = [focalLength(1), 0, principalPoint(1); 
                   0, focalLength(2), principalPoint(2); 
                   0, 0, 1]';

cameraParams = cameraParameters('IntrinsicMatrix', intrinsicMatrix, ...
                                'RadialDistortion', radialDistortion, ...
                                'TangentialDistortion', tangentialDistortion, ...
                                'ImageSize', imageSize);
% 드론 객체 생성 및 연결
droneObj = ryze();
disp(['Battery Level: ' num2str(droneObj.BatteryLevel) '%']);

% 카메라 객체 생성
global cameraObj;
cameraObj = camera(droneObj);
% 프리뷰 창 생성
figure;
hImage = imshow(snapshot(cameraObj)); % 초기 이미지를 보여줌
axis off;
title('Tello 드론 카메라 프리뷰');

% 프리뷰 활성화
preview(cameraObj, hImage);

% 사용자에게 드론을 이동시키라고 지시
% disp('드론을 이동시키고 "확인"을 누르세요. 현재 이미지 촬영 완료.');
% pause; % 사용자에게 드론을 이동시킬 시간을 줌


takeoff(droneObj);


function[x,y,z,t] = circle(radious, minr)
    global cameraObj;
    % 카메라 파라미터 설정
    focalLength = [957.1698 963.1543];
    principalPoint = [495.3551 394.4279];
    imageSize = [720 960];
    radialDistortion = [-0.0337 0.1373];
    tangentialDistortion = [0.0058 0.0034];
    
    % IntrinsicMatrix 구성
    intrinsicMatrix = [focalLength(1), 0, principalPoint(1); 
                       0, focalLength(2), principalPoint(2); 
                       0, 0, 1]';
    
    cameraParams = cameraParameters('IntrinsicMatrix', intrinsicMatrix, ...
                                    'RadialDistortion', radialDistortion, ...
                                    'TangentialDistortion', tangentialDistortion, ...
                                    'ImageSize', imageSize);

    % 사진 촬영
    frame = snapshot(cameraObj);
    
    % 보정된 이미지 생성
    image = undistortImage(frame, cameraParams);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % HSV 색 공간으로 변환
    hsvImage = rgb2hsv(image);
    
    % 푸른색 범위 설정 (HSV 값 기준)
    blueMask = (hsvImage(:,:,1) >= 0.5 & hsvImage(:,:,1) <= 0.7) & ...
               (hsvImage(:,:,2) >= 0.4 & hsvImage(:,:,2) <= 1.0) & ...
               (hsvImage(:,:,3) >= 0.4 & hsvImage(:,:,3) <= 1.0);
        
    % 마스크 적용
    filteredImage = bsxfun(@times, image, cast(blueMask, 'like', image));
    
    % 결과 표시
    % figure;
    % imshow(filteredImage);
    % title('Filtered Blue Regions');
    
    %---------------------------------------------------%
    
    % 이진화
    binaryImage = imbinarize(rgb2gray(filteredImage));
    
    % imshow(binaryImage)
    
    
    % 원 감지 (Hough 변환 사용)
    [centers, radii] = imfindcircles(binaryImage, [minr 1000], 'ObjectPolarity', 'dark', 'Sensitivity', 0.95);
    [maxValue, maxIndex] = max(radii);
    
    % 감지된 원 표시
    % imshow(image);
    % hold on;
    % viscircles(centers, radii, 'EdgeColor', 'b');
    % title('Detected Circle');
    
    % 원 중심 좌표
    t = 1;
    try
        circleCenter = centers(maxIndex, :);
    
    
    
        fprintf('Detected circle center: (%.2f, %.2f)\n', circleCenter(1), circleCenter(2));
        
        %-------------------------------------%
        % 카메라 파라미터 설정
        focalLength = [957.1698 963.1543];
        principalPoint = [495.3551 394.4279];
        imageSize = [720 960];
        radialDistortion = [-0.0337 0.1373];
        tangentialDistortion = [0.0058 0.0034];
        
        % IntrinsicMatrix 구성
        intrinsicMatrix = [focalLength(1), 0, principalPoint(1); 
                           0, focalLength(2), principalPoint(2); 
                           0, 0, 1]';
        
        cameraParams = cameraParameters('IntrinsicMatrix', intrinsicMatrix, ...
                                        'RadialDistortion', radialDistortion, ...
                                        'TangentialDistortion', tangentialDistortion, ...
                                        'ImageSize', imageSize);
        
        % 왜곡 보정된 이미지 좌표 계산
        undistortedPoints = undistortPoints(circleCenter, cameraParams);
        
        % 카메라 좌표계에서 원의 중심 좌표 계산
        % 원의 실제 지름: 0.57m
        actualDiameter = radious;
        actualRadius = actualDiameter / 2;
        
        % 카메라의 중심으로부터 원의 중심까지의 거리 계산
        z = focalLength(1) * actualRadius / radii(maxIndex);
        x = (undistortedPoints(1) - principalPoint(1)) * z / focalLength(1);
        y = (undistortedPoints(2) - principalPoint(2)) * z / focalLength(2);
        
        fprintf('3D coordinates of the circle center: (%.2f, %.2f, %.2f)\n', x, y, z);

    catch exception
        t = -1;
        x = 0;
        y = 0;
        z = 0;
        fprintf('3D coordinates of the circle center: fail!!!!!');
    end
end

function[rx, ry, rz,rt] = rr()
    global cameraObj;

    % 카메라 파라미터 설정
    focalLength = [957.1698 963.1543];
    principalPoint = [495.3551 394.4279];
    imageSize = [720 960];
    radialDistortion = [-0.0337 0.1373];
    tangentialDistortion = [0.0058 0.0034];
    
    % IntrinsicMatrix 구성
    intrinsicMatrix = [focalLength(1), 0, principalPoint(1); 
                       0, focalLength(2), principalPoint(2); 
                       0, 0, 1]';
    
    cameraParams = cameraParameters('IntrinsicMatrix', intrinsicMatrix, ...
                                    'RadialDistortion', radialDistortion, ...
                                    'TangentialDistortion', tangentialDistortion, ...
                                    'ImageSize', imageSize);

    % 사진 촬영
    image = snapshot(cameraObj);
    
    % HSV 색상 공간으로 변환
    hsvImage  = rgb2hsv(image);
    
    % 붉은 색 범위 설정
    lower_red1 = [0, 0.5, 0.5];
    upper_red1 = [0.05, 1, 1];
    lower_red2 = [0.95, 0.5, 0.5];
    upper_red2 = [1, 1, 1];
    
    % 마스크 생성
    mask1 = (hsvImage(:,:,1) >= lower_red1(1) & hsvImage(:,:,1) <= upper_red1(1)) & ...
            (hsvImage(:,:,2) >= lower_red1(2) & hsvImage(:,:,2) <= upper_red1(2)) & ...
            (hsvImage(:,:,3) >= lower_red1(3) & hsvImage(:,:,3) <= upper_red1(3));
    mask2 = (hsvImage(:,:,1) >= lower_red2(1) & hsvImage(:,:,1) <= upper_red2(1)) & ...
            (hsvImage(:,:,2) >= lower_red2(2) & hsvImage(:,:,2) <= upper_red2(2)) & ...
            (hsvImage(:,:,3) >= lower_red2(3) & hsvImage(:,:,3) <= upper_red2(3));
    redMask  = mask1 | mask2;
    
    
    %green
    % 초록색 범위 설정 (두 개의 허용치 적용)
    % lower_green1 = [0.25, 0.4, 0];  % 하한 범위
    % upper_green1 = [0.35, 1, 1];    % 상한 범위
    % tolerance1 = 0.05;              % 첫 번째 허용치
    % 
    % lower_green2 = [0.55, 0.4, 0];  % 하한 범위
    % upper_green2 = [0.65, 1, 1];    % 상한 범위
    % tolerance2 = 0.05;              % 두 번째 허용치
    % 
    % % 초록색 마스크 생성
    % mask1 = (hsvImage(:,:,1) >= (lower_green1(1) - tolerance1) & hsvImage(:,:,1) <= (upper_green1(1) + tolerance1)) & ...
    %         (hsvImage(:,:,2) >= lower_green1(2) & hsvImage(:,:,2) <= upper_green1(2)) & ...
    %         (hsvImage(:,:,3) >= lower_green1(3) & hsvImage(:,:,3) <= upper_green1(3));
    % mask2 = (hsvImage(:,:,1) >= (lower_green2(1) - tolerance2) & hsvImage(:,:,1) <= (upper_green2(1) + tolerance2)) & ...
    %         (hsvImage(:,:,2) >= lower_green2(2) & hsvImage(:,:,2) <= upper_green2(2)) & ...
    %         (hsvImage(:,:,3) >= lower_green2(3) & hsvImage(:,:,3) <= upper_green2(3));
    % redMask = mask1 | mask2;
    
    % % 보라색 범위 설정 (두 개의 허용치 적용)
    % lower_purple1 = [0.65, 0.25, 0.5];    % 하한 범위
    % upper_purple1 = [0.75, 1, 1];         % 상한 범위
    % tolerance1 = 0.05;                    % 첫 번째 허용치
    % 
    % lower_purple2 = [0.85, 0.25, 0.5];    % 하한 범위
    % upper_purple2 = [0.95, 1, 1];         % 상한 범위
    % tolerance2 = 0.05;                    % 두 번째 허용치
    % 
    % % 보라색 마스크 생성
    % mask1 = (hsvImage(:,:,1) >= (lower_purple1(1) - tolerance1) & hsvImage(:,:,1) <= (upper_purple1(1) + tolerance1)) & ...
    %         (hsvImage(:,:,2) >= lower_purple1(2) & hsvImage(:,:,2) <= upper_purple1(2)) & ...
    %         (hsvImage(:,:,3) >= lower_purple1(3) & hsvImage(:,:,3) <= upper_purple1(3));
    % mask2 = (hsvImage(:,:,1) >= (lower_purple2(1) - tolerance2) & hsvImage(:,:,1) <= (upper_purple2(1) + tolerance2)) & ...
    %         (hsvImage(:,:,2) >= lower_purple2(2) & hsvImage(:,:,2) <= upper_purple2(2)) & ...
    %         (hsvImage(:,:,3) >= lower_purple2(3) & hsvImage(:,:,3) <= upper_purple2(3));
    % redMask = mask1 | mask2;
    
    % 결과 표시

    % figure;
    % imshow(redMask);
    % title('Red Square Mask');
    rt = 1;
    try
        % 바이너리 마스크에서 영역 속성을 추출합니다.
        statsRed = regionprops(redMask, 'Area', 'Centroid', 'BoundingBox');
        
        % 가장 큰 사각형 찾기
        if ~isempty(statsRed)
            % 모든 사각형의 면적 계산
            areas = [statsRed.Area];
            
            % 가장 큰 면적의 인덱스 찾기
            [~, maxIdx] = max(areas);
            
            % 가장 큰 사각형의 중심 좌표
            redCenter = statsRed(maxIdx).Centroid;
            fprintf('Detected red square center: (%.2f, %.2f)\n', redCenter(1), redCenter(2));
        end
        
        % 왜곡 보정된 이미지 좌표 계산
        if exist('redCenter', 'var')
            undistortedRedCenter = undistortPoints(redCenter, cameraParams);
        
            % 카메라 좌표계에서 정사각형의 중심 좌표 계산
            % 정사각형 한 변의 실제 길이: 0.09m
            actualSideLength = 0.09;
            actualHalfDiagonal = sqrt(2) * actualSideLength / 2;
        
            % 가장 큰 사각형의 BoundingBox를 사용하여 중심 좌표 계산
            rz = focalLength(1) * actualHalfDiagonal / (sqrt(sum((statsRed(maxIdx).BoundingBox(3:4) / 2).^2)));
            rx = (undistortedRedCenter(1) - principalPoint(1)) * rz / focalLength(1);
            ry = (undistortedRedCenter(2) - principalPoint(2)) * rz / focalLength(2);
            fprintf('3D coordinates of the red square center: (%.2f, %.2f, %.2f)\n', rx, ry, rz);
        end

    catch exception
        rt = -1;
        rx = 0;
        ry = 0;
        rz = 0;
    end
end
moveup(droneObj,'Distance',0.4);

%첫 인식
[x, y, z, t] = circle(0.57, 50);

while t == -1
    [x, y, z, t] = circle(0.57, 50);
    pause(0.5);
end


[x, y, z, t] = circle(0.57, 50);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.2
    moveleft(droneObj,'Distance',abs(x));
end

% if (0.10 < x) && (x < 0.20)
%     moveright(droneObj, 'Distance',x + 0.20);
%     moveleft(droneObj,'Distance',0.20);
% elseif (x > -0.2) && (x<-0.1)
%     moveleft(droneObj,'Distance',abs(x) + 0.20);
%     moveright(droneObj,'Distance',0.20);
% end
movedown(droneObj,'Distance',0.2);
moveforward(droneObj,'Distance', 1.8 ,'Speed', 1.0);




[rx,ry,rz,rt] = rr();
for i = 1:3
    if rx > 0.20
        moveright(droneObj, 'Distance',rx);
    elseif rx < -0.21
        moveleft(droneObj,'Distance',-rx);
    end
    moveforward(droneObj,'Distance',0.4);
    [rx,ry,rz, rt] = rr();
end

if rx > 0.20
    moveright(droneObj, 'Distance',rx);
elseif rx < -0.21
    moveleft(droneObj,'Distance',-rx);
end

%------------------

turn(droneObj, deg2rad(130));

[x y z t] = circle(0.46, 40);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 1.5);

[x y z t] = circle(0.46, 80);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 1.0);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.7);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.5);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.5);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.3);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.3);

%---------------------------------------
turn(droneObj, deg2rad(-130));

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 1.2);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.6);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.5);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.4);
%-----------------------------
turn(droneObj,deg2rad(215))

[x y z t] = circle(0.52);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 1.5);

[x y z t] = circle(0.52);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 0.8);

[x y z t] = circle(0.46);
if x > 0.20
    moveright(droneObj, 'Distance',x);
elseif x < -0.21
    moveleft(droneObj,'Distance',-x);
end
moveforward(droneObj, 'Distance', 1.2);

[rx,ry,rz, rt] = rr();
if rx > 0.20
    moveright(droneObj, 'Distance',rx);
elseif rx < -0.21
    moveleft(droneObj,'Distance',-rx);
end

[rx,ry,rz, rt] = rr();
if  rz - 0.75 <0.2
    land(droneObj);
end

moveforward(droneObj,'Distance',rz-0.75)
[rx,ry,rz, rt] = rr();
if rx > 0.20
    moveright(droneObj, 'Distance',rx);
elseif rx < -0.21
    moveleft(droneObj,'Distance',-rx);
end
land(droneObj);

    % 
    % if y >= 0.20
    %     movedown(droneObj, 'Distance',y);
    % elseif y <= -0.2
    %     moveup(droneObj,'Distance',abs(y));
    % end


moveback(droneObj, 'Distance', 0.75 - rz);

land(droneObj);
