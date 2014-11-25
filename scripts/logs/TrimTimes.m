function [trimmed_data, trimmed_time] = TrimTimes(start_time, end_time, time_array, data_array)
  % Trims data to start and end times
  %
  % @param start_time time to start
  % @param end_time time to end
  % @param time_array arry with timestamps
  % @param data_array data to trim
  %
  % @retval trimmed_data data trimmed to specified times
  % @retval trimmed_time time array trimmed to specified times
  
  
  % sanity checks
  if (start_time < time_array(1))
    error('Start time is before first input time.');
  end
  
  if (end_time > time_array(end))
    error('End time is after last input time.');
  end
  
  % find the index of the start time
  
  [~, index_start] = min(abs(time_array - start_time));
  [~, index_end] = min(abs(time_array - end_time));
  
  trimmed_data = data_array(index_start:index_end);
  trimmed_time = time_array(index_start:index_end);
  
  
end