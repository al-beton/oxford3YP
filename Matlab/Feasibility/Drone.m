classdef Drone < handle
    % Drone class which searches in a defined route
    properties
        location = 0+0j;        %location not used in this code revision
        phase = 0;              %phase not used either
        route;          %route that drone follows
    end
       
    methods
        function obj = Drone(location, route)   %allow creation of drone objct with defined location and route
            if (nargin > 0)
                obj.location = location;
                obj.route = route;
            end
        end
        function stepPhase(obj)                 %not used in this code revision
            obj.phase = obj.phase + 1;
        end
    end
end
