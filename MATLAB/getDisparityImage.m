function [image_disp] = getDisparityImage(labeled_file)
%GETDISPARITYIMAGE This function loads the disparity image from
%   DriveUObject disp_path.
%
%   [image_disp_float] = getDisparityImage(labeled_file) returns a
%   1024x440 pixel double image
%
%   Disparity images are saved with a resolution of 1024x440 pixels. Compared to the original image
%   a binning factor of 2 is used in x and y direction, respectively. In
%   consequence, 1024x440 *2 = 2048*880, 144 pixels of the original image
%   do not have a disparity value. For the remaining color image pixels,
%   the corresponding pixel can be found by applying a binning factor of x
%   to the rectified (!) image.
%
%   Authors:    Julian Müller, Andreas Fregin
%   E-Mail:     julian-2.mueller@uni-ulm.de, andreas.fregin@daimler.com

    % type uint16
    image_disp = imread(labeled_file.disp_path);
    % type double
    image_disp_double = [];
    
    image_disp(image_disp == 65535) = 0;
    image_disp = bitand(image_disp,4095);    
    image_disp = im2double(image_disp);
    image_disp = image_disp * 65625;
    image_disp = image_disp* double(1./16.);
    
% %     for row = 1:length(image_disp(:,1))
%         for col = 1:length(image_disp(1,:))
%             
%             % 65535 = invalid
%             if (image_disp(row,col) == 65535)
%                 image_disp_double(row,col) = 0.0;
%             else
%                 % Magic you dont have do understand ;)
%                 image_disp_double(row,col) = double(bitand(image_disp(row,col),4095)) * 1./16.;
%             end
%             
%         end
%     end
% 

end

