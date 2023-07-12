classdef Node
    properties
        position
        g
        h
        f
        parent
    end
    
    methods
        function obj = Node(position, g, h, parent)
            obj.position = position;
            obj.g = g;
            obj.h = h;
            obj.f = g + h;
            obj.parent = parent;
        end
    end
end
