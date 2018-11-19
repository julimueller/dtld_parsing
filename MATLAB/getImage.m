function [image] = getImage(labeled_file)
%GETIMAGE This function loads a DriveU-Image from path
%
%   [image] = getImage(labeled_file) returns a 2048x1024 pixel uint8 image
%
%   The camera images are saved in 12 bit raw format. They are debayered in
%   converted to 8 Bit here.

%   Authors:    Julian Müller, Andreas Fregin
%   E-Mail:     julian-2.mueller@uni-ulm.de, andreas.fregin@daimler.com

    image = imread(labeled_file.path);
    image = demosaic(image, 'grbg');
    image = uint8(image/16);
end

