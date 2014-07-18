function bm_stereo_boxed = ProcessBoundingBoxBmStereo(bm_stereo, bbox)

  % loop through bbox
  
  % figure out the maximum number of points
  row_length = size(bm_stereo.x, 2);
  
  bm_stereo_boxed.utime = bm_stereo.utime;
  bm_stereo_boxed.frame_number = bm_stereo.frame_number;
  bm_stereo_boxed.video_number = bm_stereo.video_number;
  bm_stereo_boxed.logtime = bm_stereo.logtime;
  
  for i = 1 : length(bm_stereo.frame_number)
    
    % find the frame number in bbox
    
    
    bbox_index = find(bbox.frame_number - bm_stereo.frame_number(i) == 0);
    
    if length(bbox_index) > 1
      error(['Duplicate frame number in bbox: ' num2str(bm_stereo.frame_number(i))]);
    elseif isempty(bbox_index)
      
      bm_stereo_boxed.x(i, :) = zeros(1, row_length);
      bm_stereo_boxed.y(i, :) = zeros(1, row_length);
      bm_stereo_boxed.z(i, :) = zeros(1, row_length);
      
      bm_stereo_boxed.number_of_points(i) = 0;
      
     
    else
    
      % now we have the index, so zap anything not in the right spot

      this_x = bm_stereo.x(i, :);
      this_y = bm_stereo.y(i, :);
      this_z = bm_stereo.z(i, :);

      new_x = this_x(bbox.valid_bm{bbox_index} + 1);
      new_y = this_y(bbox.valid_bm{bbox_index} + 1);
      new_z = this_z(bbox.valid_bm{bbox_index} + 1);
      
      % pad new_x with zeros
      new_x = [new_x zeros(1, row_length - size(new_x, 2))];
      new_y = [new_y zeros(1, row_length - size(new_y, 2))];
      new_z = [new_z zeros(1, row_length - size(new_z, 2))];
      
      bm_stereo_boxed.number_of_points(i) = length(new_x);

      bm_stereo_boxed.x(i, :) = new_x;
      bm_stereo_boxed.y(i, :) = new_y;
      bm_stereo_boxed.z(i, :) = new_z;
      
    end
    
  end


end