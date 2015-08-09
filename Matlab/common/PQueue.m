classdef PQueue<handle
    %PQUEUE Summary of this class goes here
    %   Detailed explanation goes here
    
    % define the class properties
    properties
        U
    end
    
    % define the class methods
    methods
        % constructor
        function pq = PQueue()
            if nargin == 0
                pq.U = [];
            end
        end
                
        % get element with lowest priority
        function s = Top(pq)
            if isempty(pq.U)
                disp('Queue is empty, no element to return');
                s = [];
                return;
            end
            
            s = TopElement(pq);
        end
        
        % get key with lowest priority
        function k = TopKey(pq)
            if isempty(pq.U)
                [r,~] = size(pq.U);
                k = inf(1,r-1);
                return;
            end
            
            k = TopKeyValue(pq);
        end
        
        % pop off vertex with lowest priority
        function s = Pop(pq)
            if isempty(pq.U)
                disp('Queue is empty, no element to pop');
                s = [];
                return;
            end
            
            [s,idx] = PopElement(pq);
            pq.U(:,idx) = [];
        end
        
        % insert element into queue
        function Insert(pq, s, k)
           pq.U = [pq.U [s; k']];
        end
        
        % update element priority
        function Update(pq, s, k)
            idx = find(pq.U(1,:) == s);
            pq.U(2:end, idx) = k';
        end
        
        % remove element from queue
        function Remove(pq, s)
            idx = find(pq.U(1,:) == s);
            pq.U(:, idx) = [];
        end
        
        % determine if element is already in queue
        function f = Member(pq,s)
            f = ismember(s, pq.U(1,:));
        end
        
        % get elements in queue
        function vec = GetElements(pq)
            vec = pq.U(1,:);
        end
            
        % print a representation to the console
        function disp(pq)
            disp('priority queue');
            if ~isempty(pq.U)
                disp('U =');
                disp(pq.U);
            end
        end
        
    end
    
end

function s = TopElement(pq)
    keys = pq.U(2:end,:);
    keys = keys';

    [~, idx] = sortrows(keys);

    s = pq.U(1,idx(1));
end

function k = TopKeyValue(pq)
    keys = pq.U(2:end,:);
    keys = keys';

    [sorted_keys, ~] = sortrows(keys);

    k = sorted_keys(1,:);
end

function [s,idx] = PopElement(pq)
    keys = pq.U(2:end,:);
    keys = flipud(keys');
    N = size(keys,1);

    [~, idx] = sortrows(keys);

    s = pq.U(1,N-idx(1)+1);
    idx = N-idx(1)+1;
end
