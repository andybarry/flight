function stereo_octomap = TrimStereoOctomap(start_time, end_time, stereo_octomap_in)

  stereo_octomap.utime = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.utime);
  
  stereo_octomap.number_of_points = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.number_of_points);
  
  stereo_octomap.frame_number = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.frame_number);
  stereo_octomap.video_number = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.video_number);

  stereo_octomap.x = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.x);  
  stereo_octomap.y = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.y);
  stereo_octomap.z = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.z);
  
  stereo_octomap.frame_x = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.frame_x);
  stereo_octomap.frame_y = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.frame_y);
  
  
  stereo_octomap.logtime = TrimTimes(start_time, end_time, stereo_octomap_in.logtime, stereo_octomap_in.logtime);

end