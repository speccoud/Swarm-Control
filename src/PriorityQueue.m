classdef PriorityQueue < handle
    properties (Access = private)
        queue % The priority queue as a cell array of [element, priority] pairs
    end
    
    methods
        function obj = PriorityQueue()
            % Constructor
            obj.queue = {};
        end
        
        function insert(obj, element, priority)
            % Insert an element with a given priority
            obj.queue{end + 1} = {element, priority};
            obj.bubbleUp(length(obj.queue));
        end
        
        function element = pop(obj)
            % Remove and return the element with the highest priority
            if isempty(obj.queue)
                error('Priority queue is empty');
            end
            
            element = obj.queue{1}{1};
            obj.queue{1} = obj.queue{end};
            obj.queue(end) = [];
            obj.bubbleDown(1);
        end
        
        function isEmpty = isEmpty(obj)
            % Check if the priority queue is empty
            isEmpty = isempty(obj.queue);
        end
    end
    
    methods (Access = private)
        function bubbleUp(obj, index)
            % Bubble up an element in the queue to maintain the heap property
            while index > 1
                parentIndex = floor(index / 2);
                if obj.queue{index}{2} >= obj.queue{parentIndex}{2}
                    break;
                end
                temp = obj.queue{index};
                obj.queue{index} = obj.queue{parentIndex};
                obj.queue{parentIndex} = temp;
                index = parentIndex;
            end
        end
        
        function bubbleDown(obj, index)
            % Bubble down an element in the queue to maintain the heap property
            while true
                leftChildIndex = 2 * index;
                rightChildIndex = 2 * index + 1;
                smallestIndex = index;
                
                if leftChildIndex <= length(obj.queue) && obj.queue{leftChildIndex}{2} < obj.queue{smallestIndex}{2}
                    smallestIndex = leftChildIndex;
                end
                
                if rightChildIndex <= length(obj.queue) && obj.queue{rightChildIndex}{2} < obj.queue{smallestIndex}{2}
                    smallestIndex = rightChildIndex;
                end
                
                if smallestIndex == index
                    break;
                end
                
                temp = obj.queue{index};
                obj.queue{index} = obj.queue{smallestIndex};
                obj.queue{smallestIndex} = temp;
                index = smallestIndex;
            end
        end
    end
end
