function bounding_box = LoadBoundingBox(csv_path)
% Loads bounding box from a csv like pass1.csv produced by the HUD
% when passed the -b option

  csv_vals = csvread(csv_path);
  
  last_frame_number = -1;
  num = 1;
  
  for i = 1 : size(csv_vals, 1)
  
    % check for duplicate frame numbers
    this_frame_number = csv_vals(i, 2);

    
    valid_inds_temp = csv_vals(i, 7:end);
    % check for only zeros
    if any(valid_inds_temp)
      % there are some non-zero elements
      if (valid_inds_temp(1) == 0)
        % 0 is a valid ind
        zero_add = 0;
      else
        % 0 is not a valid ind
        zero_add = [];
      end
      
      valid_inds = [zero_add csv_vals(i, find(csv_vals(i, 7:end))+6)];
      
    else
      valid_inds = [];
    end
    
    
    if (this_frame_number == last_frame_number)
      
      % duplicate frame number, merge them!
      
      disp(['Combining lines for frame ' num2str(last_frame_number)]);
      
      bounding_box.valid_bm{num - 1} = unique([ bounding_box.valid_bm{num - 1} valid_inds ]);
      
    else
      bounding_box.video_number(num) = csv_vals(i, 1);
      
      bounding_box.frame_number(num) = this_frame_number;
      
      bounding_box.x1(num) = csv_vals(i, 3);
      bounding_box.y1(num) = csv_vals(i, 4);
      
      bounding_box.x2(num) = csv_vals(i, 5);
      bounding_box.y2(num) = csv_vals(i, 6);
      
      bounding_box.valid_bm{num} = valid_inds;
      
      num = num + 1;
      
    end
    
    last_frame_number = this_frame_number;
    
  end
  
end


