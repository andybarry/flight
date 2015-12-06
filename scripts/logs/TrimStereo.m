function stereo = TrimStereo(start_time, end_time, stereo_in)

  if end_time > stereo_in.logtime(end)
    end_time = stereo_in.logtime(end);
  end

  stereo.utime = TrimTimes(start_time, end_time, stereo_in.logtime, stereo_in.utime);
  
  stereo.number_of_points = TrimTimes(start_time, end_time, stereo_in.logtime, stereo_in.number_of_points);
  
  stereo.frame_number = TrimTimes(start_time, end_time, stereo_in.logtime, stereo_in.frame_number);
  stereo.video_number = TrimTimes(start_time, end_time, stereo_in.logtime, stereo_in.video_number);

  stereo.x = TrimTimes(start_time, end_time, stereo_in.logtime, stereo_in.x);  
  stereo.y = TrimTimes(start_time, end_time, stereo_in.logtime, stereo_in.y);
  stereo.z = TrimTimes(start_time, end_time, stereo_in.logtime, stereo_in.z);
  
  
  stereo.logtime = TrimTimes(start_time, end_time, stereo_in.logtime, stereo_in.logtime);

end