function distances = SmallestDistance(from_x, from_y, from_z, to_x, to_y, to_z, min_sense_dist, max_sense_dist)
  % Computes the minimum distance from (from_x, from_y, from_z) to the
  %   cloest point in (to_x, to_y, to_z)
  %
  % @param from_x array of points sensed from stereo... aka
  %   stereo_octomap.stereo.x
  % @param from_y array of points sensed from stereo... aka
  %   stereo_octomap.stereo.y
  % @param from_z array of points sensed from stereo... aka
  %   stereo_octomap.stereo.z
  
  % @param to_x array of points sensed from stereo... aka
  %   bm_stereo.x
  % @param to_y array of points sensed from stereo... aka
  %   bm_stereo.y
  % @param to_z array of points sensed from stereo... aka
  %   bm_stereo.z
  %
  % @param min_sense_dist z-distance at which to ignore from_* points
  % @param max_sense_dist maximum z-distance at which to compute data for from_* points
  %
  % @retval distances array of minimum distances at each pont
  
  
  no_match_dist = 10;
  
  distances = [];
  
  counter = 0;
  
  if (~isequal(size(from_x), size(from_y)) || ~isequal(size(from_x), size(from_z)))
    error('x1/y1/z1 size mismatch.');
  end
  
  if (~isequal(size(to_x), size(to_y)) || ~isequal(size(to_x), size(to_z)))
    error('x2/y2/z2 size mismatch.');
  end
  
  if (size(from_x,1) ~= size(to_x,1))
    error('Different number of frames for x1 and x2.');
  end
  
  num_frames = size(from_x, 1);
  max_points = size(from_x, 2);
  
  max_truth_est_points = size(to_x, 2);
  
  for i = 1 : num_frames
    fprintf(['\nProcessing frame ' num2str(i) ' / ' num2str(num_frames)]);
    
    for j = 1 : max_points
      
      this_point = [ from_x(i, j) from_y(i, j) from_z(i, j) ];
      
      
      % check for no more points
      
      if (this_point(3) == 0)
        % we're done
        break;
      elseif (this_point(3) > min_sense_dist && this_point(3) < max_sense_dist)
        
        distances(i, j) = no_match_dist;

        % determine the minimum distance to other points
        for k = 1 : max_truth_est_points

          this_truth_est_point = [ to_x(i, k) to_y(i, k) to_z(i, k) ];

          if (this_truth_est_point(3) == 0)
            break;
          end

          this_dist = sum((this_point - this_truth_est_point).^2).^0.5;

          if (this_dist < distances(i, j))
            distances(i, j) = this_dist;
          end

        end
      else
        distances(i, j) = 0;
      end
      
      counter = counter + 1;
      if (distances(i, j) == no_match_dist)
        fprintf('x');
      else
        fprintf('.');
      end
      if (mod(j,80) == 0)
        fprintf('\n');
      end
      
      
    end
    
  end
  
  fprintf('\n\nProcessed %d points.\n', counter);

end