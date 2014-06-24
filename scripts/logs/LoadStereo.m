function stereo = LoadStereo(stereo_values)
% Loads stereo values from a log

% stereo
% #stereo  <class 'lcmt_stereo.lcmt_stereo'> :
% #[
% #1- timestamp
% #2- number_of_points
% #3- frame_number
% #4- video_number
% #5- x(0)
% #5- y(0)
% #5- z(0)
% #5- log_timestamp
% #]

  stereo.utime = stereo_values(:,1);
  stereo.number_of_points = stereo_values(:,2);
  stereo.frame_number = stereo_values(:,3);
  stereo.video_number = stereo_values(:,4);

  if (size(stereo_values, 2) < 6)
    % if we got no stereo hits, the array won't be initialized
    stereo.x = [];
    stereo.y = [];
    stereo.z = [];
    stereo.logtime = stereo_values(:,5);
  else
    
    % init variables
    stereo.x = zeros(length(stereo.frame_number), size(stereo_values,2));
    stereo.y = zeros(length(stereo.frame_number), size(stereo_values,2));
    stereo.z = zeros(length(stereo.frame_number), size(stereo_values,2));
    
    % parse the nubmer of points
    for i=1:length(stereo.number_of_points)
      
      this_number = stereo.number_of_points(i);
      
      if (this_number > 0)
      
        positions = 0:this_number;

        offset_index = 5;

        stereo.x(i,1:this_number) = stereo_values(i, offset_index:this_number+offset_index-1);

        offset_index = offset_index + this_number;
        stereo.y(i,1:this_number) = stereo_values(i, offset_index:this_number+offset_index-1);

        offset_index = offset_index + this_number;
        stereo.z(i,1:this_number) = stereo_values(i, offset_index:this_number+offset_index-1);

        stereo.logtime(i) = stereo_values(i, this_number*3+5);
      else
        
        stereo.logtime(i) = stereo_values(i, 5);
        
      end
    end
    
    
     
  end

end


