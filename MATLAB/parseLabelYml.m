function [labeled_files] = parseLabelYml(path)
%PARSELABELYML The function returns a struct of labeled images and vehicle
%data
%
%   [labeled_files] = parseLabelYml(path) returns a struct of labeled
%   images with object and vehicle data
%
%
%   The structure is as follows:
%       labeled_filse[]   
%       ----objects[]
%           ----x           label upper left corner (pixels), hotizontal axis
%           ----y           label upper left corner (pixels), vertical axis
%           ----w           label width (pixels)
%           ----h           label height (pixels)
%           ----class_id    label class identity (explanation see below)
%           ----unique_id   label unique identity (each TL has one unique id)
%           ----track_id    label track id, each TL instance has identical
%                           track id over multiple image frames
%       ----path            image path (left camera image)
%       ----disp_path       disparity image path
%       ----timestamp       time stamp of camera image
%       ----velocity        velocity of the vehicle (meters/second)
%       ----yaw_rate        yaw rate of vehicle (rad/second)
%       ----longitude       GPS longitude (°)
%       ----latitude        GPS latitude (°)

%        Class-ID Meaning:
% 
%        CLASSES:
% 
%        FIRST DIGIT 
% 
%        0xxxx    not relevant traffic light
%        1xxxx    relevant traffic light
%        2xxxx    not relevant traffic light, best_sight on this one
%        3xxxx    relevant traffic light, best_sight on this one
%        4xxxx    occluded
% 
%        x1xxx    traffic light horizontal
%        x2xxx    traffic light vertical
%        x3xxx    traffic light horizontal with no frame (erdnuss-ampel)
%        x4xxx    traffic light vertical with no frame (erdnuss-ampel)
%        x5xxx       
%        x6xxx    horizontal bus/tram traffic light 
%        x7xxx    vertical bus/Tram traffic light
%        x8xxx    horizontal bus/tram traffic light with no frame
%        x9xxx    vertical bus/tram traffic light with no frame 
%  
%        xx1xx    single light
%        xx2xx    dual light
%        xx3xx    triple light
%        xx4xx    quatro light
%        xx5xx    ...
%        [...]
% 
%        xxx0x    lights off
%        xxx1x    red
%        xxx2x    yellow
%        xxx3x    red-yellow
%        xxx4x    green
%        xxx5x    white
% 
%        xxxx0    no light mask
%        xxxx1    arrow_straight
%        xxxx2    arrow_left
%        xxxx3    arrow_left+straight
%        xxxx4    arrow_right
%        xxxx5    arrow_right+straight
%        xxxx7    arrow_left+right+straight    
%        xxxx8    'Pedestrian' mask in light
%        xxxx9    'Bike' mask in Light
% 
%        Examples:
%
%        2314    vertical triple light, red, arrow_right
%        2128    signle yellow warn light with pedestrian mask
%        2340    normal Green TL
%        2310    normal Red TL
%        2320    normal Yellow TL
%        2330    normal Red-Yellow TL
%
%   Authors:    Julian Müller, Andreas Fregin
%   E-Mail:     julian-2.mueller@uni-ulm.de, andreas.fregin@daimler.com

    fid = fopen(path);
    tline = fgetl(fid);

    labeled_files = [];
    i=0;
    
    while ischar(tline)

        if (tline(1) == '-' )

            tline = fgetl(fid);
            j=1;
            i=i+1;

            objects = [];

            while(tline(3) == '-')
                string = strsplit(tline);
                labeled_files(i).objects(j).x = str2num(string{1,4})
                labeled_files(i).objects(j).y = str2num(string{1,6})
                labeled_files(i).objects(j).w = str2num(string{1,8})
                labeled_files(i).objects(j).h = str2num(string{1,10})
                labeled_files(i).objects(j).class_id = str2num(string{1,12})
                labeled_files(i).objects(j).unique_id = str2num(string{1,14})
                labeled_files(i).objects(j).track_id = string{1,16}(1:end-1)
                j=j+1;
                tline = fgetl(fid);
            end
        end


        if (tline(2:8) == ' path: ')
            string = strsplit(tline);
            labeled_files(i).path = string{1,3};
        end

        if (tline(2:13) == ' disp_path: ')
            string = strsplit(tline);
            labeled_files(i).disp_path = string{1,3};
        end

        if (tline(2:14) == ' time_stamp: ')
            string = strsplit(tline);
            labeled_files(i).timestamp = str2num(string{1,3});
        end

        if (tline(2:12) == ' velocity: ')
            string = strsplit(tline);
            labeled_files(i).velocity = str2num(string{1,3});
        end

        if (tline(2:12) == ' yaw_rate: ')
            string = strsplit(tline);
            labeled_files(i).yaw_rate = str2num(string{1,3});
        end

        if (tline(2:13) == ' longitude: ')
            string = strsplit(tline);
            labeled_files(i).longitude = str2num(string{1,3});
        end


        if (tline(2:12) == ' latitude: ')
            string = strsplit(tline);
            labeled_files(i).latitude = str2num(string{1,3});
        end

        tline = fgetl(fid);


    end
end


