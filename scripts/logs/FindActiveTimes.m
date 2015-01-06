function [start_times, end_times] = FindActiveTimes(time, data, threshold)
    % Given a time sequence and data, finds the start and end times
    % of the data's interesting regions.  For example, you can find the
    % times the throttle is on:
    %
    % <pre>
    % [throttle_start, throttle_end] = FindActiveTimes(u.logtime,
    %   u.throttle, 1700)
    % </pre>
    %
    % @param time a time sequence
    % @param data a data sequence
    % @param thresold a single value above which the data is considered
    %   interesting.
    %
    % @retval start_times array of times (in seconds) where intersting
    %   regions start
    % @retval end_times array of times (in seconds) where interesting
    %   regions end
    
    % discover throttle starts
    ind = find(data > threshold);
    if (isempty(ind))
        % can't find spots with high throttle
        warning('Cannot find any time with data above thresold')
        start_times = [];
        end_times = [];
        return;
    else
        
        % break into sections if there are multiple runs
        
        
        dind = diff(ind);
        
        
        dt = mean(diff(time));
        
        sec_dividing_runs = 1;
        
        diff_inds = find(dind > sec_dividing_runs/dt);
        
        % now we have the loation in the diff of the indicies
        
        % the indicies show where things end and start
        
        end_array = [diff_inds; length(ind)];
        start_array = [1; diff_inds + 1];
        
        
        start_times = time(ind(start_array));
        end_times = time(ind(end_array));
        
    end
    
    
    

end