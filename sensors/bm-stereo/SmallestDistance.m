function distances = SmallestDistance(sensedx, sensedy, sensedz, truth_estx, truth_esty, truth_estz, min_sense_dist)
  % Computes the distance to the cloest point in sensed to ground_truth_est
  %
  % @param sensedx array of points sensed from stereo... aka
  %   stereo_octomap.stereo.x
  % @param sensedy array of points sensed from stereo... aka
  %   stereo_octomap.stereo.y
  % @param sensedz array of points sensed from stereo... aka
  %   stereo_octomap.stereo.z
  
  % @param truth_estx array of points sensed from stereo... aka
  %   bm_stereo.x
  % @param truth_esty array of points sensed from stereo... aka
  %   bm_stereo.y
  % @param truth_estz array of points sensed from stereo... aka
  %   bm_stereo.z
  %
  % @param min_sense_dist z-distance at which to ignore sensed points
  %
  % @retval distances array of minimum distances at each pont
  
  distances = [];
  
  if (~isequal(size(sensedx), size(sensedy)) || ~isequal(size(sensedx), size(sensedz)))
    error('sensed size mismatch.');
  end
  
  if (~isequal(size(truth_estx), size(truth_esty)) || ~isequal(size(truth_estx), size(truth_estz)))
    error('sensed size mismatch.');
  end
  
  num_frames = size(sensedx, 1);
  max_points = size(sensedx, 2);
  
  max_truth_est_points = size(truth_estx, 2);
  
  for i = 1 : num_frames
    fprintf(['\nProcessing frame ' num2str(i) ' / ' num2str(num_frames)]);
    
    for j = 1 : max_points
      
      this_point = [ sensedx(i, j) sensedy(i, j) sensedz(i, j) ];
      
      % check for no more points
      
      if (this_point(3) == 0)
        % we're done
        break;
      elseif (this_point(3) > min_sense_dist)

        distances(i, j) = 1e8;

        % determine the minimum distance to other points
        for k = 1 : max_truth_est_points

          this_truth_est_point = [ truth_estx(i, k) truth_esty(i, k) truth_estz(i, k) ];

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
      
      
      if (distances(i, j) == 1e8)
        fprintf('x');
      else
        fprintf('.');
      end
      if (mod(j,80) == 0)
        fprintf('\n');
      end
      
      
    end
    
    
    
    
  end
  
  

end