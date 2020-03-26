%MAIN   Main function of MATLAB DriveU Traffic Light Dataset wrapper. This
%   program shows examples how to load the dataset images, labels and
%   calibration
%
%   Authors:    Julian Müller, Andreas Fregin
%   E-Mail:     julian-2.mueller@uni-ulm.de, andreas.fregin@daimler.com

close all;
clear all;
clc;
%% Parse label files

labeled_files = parseLabelYml('/scratch/fs2/DTLD_final/Essen_all.yml');
%% Load all calibration matrices

intrinsic = parseCalibYml('/home/muejul3/driveu-dataset-parsing/Calibration/intrinsic_left.yml');
distortion = parseCalibYml('/home/muejul3/driveu-dataset-parsing/Calibration/distortion_left.yml');
projection = parseCalibYml('/home/muejul3/driveu-dataset-parsing/Calibration/projection_left.yml');
rectification = parseCalibYml('/home/muejul3/driveu-dataset-parsing/Calibration/rectification_left.yml');


%% Load and display labeled images and disparity image

h= figure;
set(h, 'Position', [100, 100, 1000, 1000]);

for i=1:length(labeled_files)
    clf;
    % Get color image
    hold off;
    image =getImage(labeled_files(i));
    cla;
    hold on;
    subplot(2,1,1), imshow(image);
    axis equal;
    title('DriveU Color Image With Annotations')
        
    % Draw labels
    for j = 1:length(labeled_files(i).objects)
       rectangle('Position', [labeled_files(i).objects(j).x labeled_files(i).objects(j).y labeled_files(i).objects(j).w labeled_files(i).objects(j).h], 'LineWidth',2, 'EdgeColor',colorFromClassId(labeled_files(i).objects(j).class_id));
       hold on;
    end
    
    % Get disparity image
    image_disp_double = getDisparityImage(labeled_files(i));

    % Draw disparity image
    subplot(2,1,2), imshow(image_disp_double, [min(image_disp_double(:)),max(image_disp_double(:))]);
    axis equal;
    title('DriveU Disparity Image Illustrated as ColorMap')

    colormap(gca,jet); 
    %colorbar
    pause (.01);
    
    
    
end