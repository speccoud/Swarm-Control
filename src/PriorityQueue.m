classdef PriorityQueue
    properties
        queue
    end
    
    methods
        function obj = PriorityQueue()
            obj.queue = [];
        end
        
        function enqueue(obj, node)
            % Enqueue node based on f-score
            if isempty(obj.queue)
                obj.queue = node;
            else
                insert_index = find([obj.queue.f] >= node.f, 1);
                if isempty(insert_index)
                    obj.queue(end+1) = node;
                else
                    obj.queue = [obj.queue(1:insert_index-1), node, obj.queue(insert_index:end)];
                end
            end
        end
        
        function node = dequeue(obj)
            % Dequeue node with lowest f-score
            node = obj.queue(1);
            obj.queue = obj.queue(2:end);
        end
        
        function empty = isEmpty(obj)
            % Check if the queue is empty
            empty = isempty(obj.queue);
        end
    end
end
