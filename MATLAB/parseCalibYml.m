function [data] = parseCalibYml(path)
%PARSECALIBYML This function returns a matrix loaded from path
%  
%   [data] = parseCalibYml(path)
%
%   All calibration files (intrinsic_left.yml, ...) can be loaded using this
%   function
%
%                               [fx  0 cx]
%   Intrinsic Matrix:       K = [ 0 fy cy]
%                               [ 0  0  1]
%
%                               [r_11 r_12 r_13 tx]
%   Extrinsic Matrix:       E = [r_21 r_22 r_23 ty]
%                               [r_31 r_32 r_33 tz]
%
%                               [fx 0 cx tx]
%   Projection Matrix:      P = [0 fy cy ty]
%                               [0  0  0  1]
%
%                               [r_11 r_12 r_13] 
%   Rectification Matrix:   R = [r_21 r_22 r_23]
%                               [r_32 r_32 r_33]
%
%   Distortion Matrix       D = [k1 k2 p1 p2 k3]
%
%   Authors:    Julian Müller, Andreas Fregin
%   E-Mail:     julian-2.mueller@uni-ulm.de, andreas.fregin@daimler.com


fid = fopen(path);
tline = fgetl(fid);
    
while ischar(tline)

    if (strfind(tline,'rows'))
       [~,rows] = strtok(tline,' ');
       rows = str2num(rows);
    end
    
    if (strfind(tline,'cols'))
       [~,cols] = strtok(tline,' ');
       cols = str2num(cols);
    end

    if (strfind(tline,'data'))
       [~,data] = strtok(tline,' ');
       data = str2num(data);
       data = reshape(data, cols, rows).';
    end
    
    tline = fgetl(fid);
end

fclose(fid);

end

