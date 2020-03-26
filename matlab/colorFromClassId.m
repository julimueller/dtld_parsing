function [color] = colorFromClassId(class_id)
%COLORFROMCLASSID This function returns a state-color from the class
%identity
%
%   [color] = colorFromClassId(class_id) returns an rgb color
%
%   We check the 4th digit to get the color (state) of the traffic light.
%   Needed for visualization
%
%   Authors:    Julian Müller, Andreas Fregin
%   E-Mail:     julian-2.mueller@uni-ulm.de, andreas.fregin@daimler.com

class_id_str = num2str(class_id);
color_id = class_id_str(end-1);

if (color_id == '1')
    color = [1 0 0];

elseif (color_id == '2')
    color = [1 1 0];
    
elseif (color_id == '3')
    color = [1 0.5 0];
    
elseif (color_id == '4')
    color = [0 1 0];
    
else
    color = [1 1 1];
    
end

end

