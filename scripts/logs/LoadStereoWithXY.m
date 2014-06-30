function stereo_with_xy = LoadStereoWithXY(stereo_with_xy_values)
% Loads stereo values from a log

% #stereo-octomap  <class 'lcmt_stereo_with_xy.lcmt_stereo_with_xy'> :
% #[
% #1- timestamp
% #2- number_of_points
% #3- frame_number
% #4- video_number
% #5- x(0)
% #5- y(0)
% #5- z(0)
% #5- frame_x(0)
% #5- frame_y(0)
% #5- log_timestamp
% #]


  stereo_with_xy.utime = stereo_with_xy_values(:,1);
  stereo_with_xy.number_of_points = stereo_with_xy_values(:,2);
  stereo_with_xy.frame_number = stereo_with_xy_values(:,3);
  stereo_with_xy.video_number = stereo_with_xy_values(:,4);

  if (size(stereo_with_xy_values, 2) < 6)
    % if we got no stereo hits, the array won't be initialized
    stereo_with_xy.x = [];
    stereo_with_xy.y = [];
    stereo_with_xy.z = [];
    stereo_with_xy.frame_x = [];
    stereo_with_xy.frame_y = [];
    stereo_with_xy.logtime = stereo_with_xy_values(:,5);
  else
    
    % init variables
    stereo_with_xy.x = zeros(length(stereo_with_xy.frame_number), size(stereo_with_xy_values,2));
    stereo_with_xy.y = zeros(length(stereo_with_xy.frame_number), size(stereo_with_xy_values,2));
    stereo_with_xy.z = zeros(length(stereo_with_xy.frame_number), size(stereo_with_xy_values,2));
    
    stereo_with_xy.frame_x = zeros(length(stereo_with_xy.frame_number), size(stereo_with_xy_values,2));
    stereo_with_xy.frame_y = zeros(length(stereo_with_xy.frame_number), size(stereo_with_xy_values,2));
    
    
    % parse the nubmer of points
    for i=1:length(stereo_with_xy.number_of_points)
      
      this_number = stereo_with_xy.number_of_points(i);
      
      if (this_number > 0)
      
        positions = 0:this_number;

        offset_index = 5;

        stereo_with_xy.x(i,1:this_number) = stereo_with_xy_values(i, offset_index:this_number+offset_index-1);

        offset_index = offset_index + this_number;
        stereo_with_xy.y(i,1:this_number) = stereo_with_xy_values(i, offset_index:this_number+offset_index-1);

        offset_index = offset_index + this_number;
        stereo_with_xy.z(i,1:this_number) = stereo_with_xy_values(i, offset_index:this_number+offset_index-1);
        
        offset_index = offset_index + this_number;
        stereo_with_xy.frame_x(i,1:this_number) = stereo_with_xy_values(i, offset_index:this_number+offset_index-1);
        
        offset_index = offset_index + this_number;
        stereo_with_xy.frame_y(i,1:this_number) = stereo_with_xy_values(i, offset_index:this_number+offset_index-1);

        stereo_with_xy.logtime(i) = stereo_with_xy_values(i, this_number*5+5);
      else
        
        stereo_with_xy.logtime(i) = stereo_with_xy_values(i, 5);
        
      end
    end
    
    
     
  end

end


