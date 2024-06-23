% 이미지 파일 경로
imageFiles = {"C:\Pro_gramming\dron AI\hw3\사진1.jpg", "C:\Pro_gramming\dron AI\hw3\사진2.jpg", "C:\Pro_gramming\dron AI\hw3\사진3.jpg", "C:\Pro_gramming\dron AI\hw3\사진4.jpg", "C:\Pro_gramming\dron AI\hw3\사진5.jpg"};

for i = 1:length(imageFiles)

    image = imread(imageFiles{i});

    % Convert the image to HSV color space
    hsvImage = rgb2hsv(image);
    
    % Define thresholds for the green color in the HSV space
    greenThreshLow = [0.25, 0.40, 0.20]; % Lower threshold for green [Hue, Saturation, Value]
    greenThreshHigh = [0.45, 1.00, 1.00]; % Upper threshold for green [Hue, Saturation, Value]
    
    % Create a binary mask for the green regions
    greenMask = (hsvImage(:,:,1) >= greenThreshLow(1)) & (hsvImage(:,:,1) <= greenThreshHigh(1)) & ...
                (hsvImage(:,:,2) >= greenThreshLow(2)) & (hsvImage(:,:,2) <= greenThreshHigh(2)) & ...
                (hsvImage(:,:,3) >= greenThreshLow(3)) & (hsvImage(:,:,3) <= greenThreshHigh(3));
    
    % Remove noise using morphological operations
    greenMask = imopen(greenMask, strel('rectangle', [5, 5]));
    greenMask = imclose(greenMask, strel('rectangle', [15, 15]));
    
    % Find the boundaries of the green regions
    props = regionprops(greenMask, 'BoundingBox', 'Centroid');
    
    % Find the bounding box of the largest green region
    if ~isempty(props)
        [~, largestIdx] = max(cellfun(@(x) x(3)*x(4), {props.BoundingBox}));
        boundingBox = props(largestIdx).BoundingBox;
        
        % Invert the green mask to find the holes
        invertedMask = ~greenMask;
        
        % Find the boundaries of the holes
        holeProps = regionprops(invertedMask, 'BoundingBox', 'Centroid');
        
        % Find the largest hole within the green region
        holeBoundingBox = [];
        for j = 1:length(holeProps)
            if holeProps(j).BoundingBox(1) > boundingBox(1) && ...
               holeProps(j).BoundingBox(1) + holeProps(j).BoundingBox(3) < boundingBox(1) + boundingBox(3) && ...
               holeProps(j).BoundingBox(2) > boundingBox(2) && ...
               holeProps(j).BoundingBox(2) + holeProps(j).BoundingBox(4) < boundingBox(2) + boundingBox(4)
                holeBoundingBox = holeProps(j).BoundingBox;
                break;
            end
        end
        
        if ~isempty(holeBoundingBox)
            % Calculate the center of the hole
            holeCenter = [holeBoundingBox(1) + holeBoundingBox(3)/2, holeBoundingBox(2) + holeBoundingBox(4)/2];
        else
            holeCenter = [NaN, NaN];
            warning('No hole detected.');
        end
    else
        holeCenter = [NaN, NaN];
        warning('No green region detected.');
    end
    
    % Print the coordinates of the hole's center
    fprintf('Center coordinates of the hole: (%.2f, %.2f)\n', holeCenter(1), holeCenter(2));
    
    % 새로운 figure를 열고 이미지와 중심점을 표시
    figure;
    imshow(image);
    hold on;
    if ~isnan(holeCenter(1))
        plot(holeCenter(1), holeCenter(2), 'r+', 'MarkerSize', 30, 'LineWidth', 2);
        title(sprintf('Center of the hole: (%.2f, %.2f)', holeCenter(1), holeCenter(2)));
        
    else
        title('No hole detected.');
    end
    hold off;
end
