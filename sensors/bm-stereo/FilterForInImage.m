function stereo_filtered = FilterForInImage(stereo_octomap, left_bound, right_bound, top_bound, bottom_bound)
  % Fitlers points for being in the image, with bounds.  NOTE these bounds
  % are likely in UNRECTIFIED image space.
  %
  % @param stereo_octomap structure from loadDeltawing
  % @param bm_stereo bm stereo structure
  % @param left_bound pixel bound on the left side for in-the-image
  % @param right_bound pixel bound on the right side
  % @param upper_bound pixel bound on the top
  % @param lower_bound pixel bound on the bottom
  %
  % @retval stereo_filtered stereo_octomap like structure, but filtered to
  % only contain points valid for bm-stereo
  %
  
  num_frames = size(stereo_octomap.x, 1);
  max_points = size(stereo_octomap.x, 2);
  
  stereo_filtered.x = [[]];
  stereo_filtered.y = [];
  stereo_filtered.z = [];
  
  stereo_filtered.frame_x = [];
  stereo_filtered.frame_y = [];

  for i = 1 : num_frames
    
    num_points = stereo_octomap.number_of_points(i);
    valid_points = [];
    
    for j = 1 : num_points
      
      this_frame_point = [ stereo_octomap.frame_x(i, j) stereo_octomap.frame_y(i, j) ];
      
      if (this_frame_point(1) < 0 || this_frame_point(1) < left_bound ...
          || this_frame_point(1) > right_bound || this_frame_point(2) > bottom_bound ...
          || this_frame_point(2) < top_bound)
        % not a valid point
      else
        
        valid_points = [ valid_points j ];
        
        
      end
    end
    
    % now done with this frame
    
    stereo_filtered.utime(i) = stereo_octomap.utime(i);
    stereo_filtered.number_of_points(i) = length(valid_points);
    stereo_filtered.frame_number(i) = stereo_octomap.frame_number(i);
    stereo_filtered.video_number(i) = stereo_octomap.video_number(i);
    stereo_filtered.logtime(i) = stereo_octomap.logtime(i);
    
    
    if ~isempty(valid_points)
      stereo_filtered.x(i,:) = [ stereo_octomap.x(i,valid_points) zeros(1, max_points - length(valid_points)) ];
      stereo_filtered.y(i,:) = [ stereo_octomap.y(i,valid_points) zeros(1, max_points - length(valid_points)) ];
      stereo_filtered.z(i,:) = [ stereo_octomap.z(i,valid_points) zeros(1, max_points - length(valid_points)) ];

      stereo_filtered.frame_x(i,:) = [ stereo_octomap.frame_x(i,valid_points) zeros(1, max_points - length(valid_points)) ];
      stereo_filtered.frame_y(i,:) = [ stereo_octomap.frame_y(i,valid_points) zeros(1, max_points - length(valid_points)) ];
    else
      stereo_filtered.x(i,:) = zeros(1, max_points);
      stereo_filtered.y(i,:) = zeros(1, max_points);
      stereo_filtered.z(i,:) = zeros(1, max_points);

      stereo_filtered.frame_x(i,:) = zeros(1, max_points);
      stereo_filtered.frame_y(i,:) = zeros(1, max_points);
      
    end
    
  end
  

end