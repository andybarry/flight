function fpga_struct = LoadFpgaStereo(fpga_csv_filename)
  % Loads fpga stereo csv file
  %
  % @param fpga_csv_filename filename of the csv file without header line
  %
  % @retval fpga_struct structure with the following entries:
  % <pre>
  %   fpga.frame: frame number
  %   fpga.time_us: device time
  %   fpga.plane_time: timestamp from airplane
  % </pre>


  %fpga_csv_filename = ''/home/abarry/MIT/RLG/logs/2014-10-10-fpga-flight1/boston_flight_logs/awesome1/log_for_matlab.csv';


  fpga_raw = tdfread(fpga_csv_filename, ',');


  fpga_struct.frame = fpga_raw.frame;
  fpga_struct.time_us = fpga_raw.time_us;
  fpga_struct.plane_time = fpga_raw.vicon_time_us
  
  
end