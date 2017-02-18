flight
======

Flight code for MIT CSAIL [Robot Locomotion Group](https://groups.csail.mit.edu/locomotion/index.html) flying-through-forests project

This code powers:

* [Self-flying drone dips, darts and dives through trees at 30 mph](http://www.csail.mit.edu/drone_flies_through_forest_at_30_mph)
  * [video](https://www.youtube.com/watch?v=_qah8oIzCwk)
  * [thesis (pdf)](http://groups.csail.mit.edu/robotics-center/public_papers/Barry16.pdf)
  
  [![Drone Autonomously Avoiding Obstacles at 30 MPH](http://img.youtube.com/vi/_qah8oIzCwk/0.jpg)](https://www.youtube.com/watch?v=_qah8oIzCwk)

  * [techincal video](https://www.youtube.com/watch?v=iksfHQkkq88)
  
  [![Pushbroom Stereo for High-Speed Obstacle Avoidance (technical video)](http://img.youtube.com/vi/iksfHQkkq88/0.jpg)](https://www.youtube.com/watch?v=iksfHQkkq88)

* *FPGA vs. pushbroom stereo vision for MAVs*:
  * [paper (pdf)](http://groups.csail.mit.edu/robotics-center/public_papers/Barry15a.pdf)

* *Pushbroom stereo for high-speed navigation in cluttered environments*:
  * [paper (pdf)](http://groups.csail.mit.edu/robotics-center/public_papers/Barry15.pdf)
  * [video](https://www.youtube.com/watch?v=cZE01bJIgvQ)

  [![Pushbroom stereo for high-speed navigation in cluttered environments](http://img.youtube.com/vi/cZE01bJIgvQ/0.jpg)](https://www.youtube.com/watch?v=cZE01bJIgvQ)


* *Fast and Accurate Knife-Edge Maneuvers for Autonomous Aircraft*:
  * [video](https://www.youtube.com/watch?v=voN9CCmzxYk)
  * [abstract (pdf)](http://groups.csail.mit.edu/robotics-center/public_papers/Barry14.pdf)
  * [thesis (pdf)](http://groups.csail.mit.edu/robotics-center/public_papers/Barry12a.pdf)

Pushbroom stereo files are here:
  * [Main class](https://github.com/andybarry/flight/blob/master/sensors/stereo/pushbroom-stereo.hpp) [[cpp](https://github.com/andybarry/flight/blob/master/sensors/stereo/pushbroom-stereo.cpp)]
  * [Camera capture, recording management, debugging UI, etc.](https://github.com/andybarry/flight/blob/master/sensors/stereo/pushbroom-stereo-main.hpp) [[cpp](https://github.com/andybarry/flight/blob/master/sensors/stereo/pushbroom-stereo-main.cpp)]
  
Stereo image and synced sensor data:
 * [Datasets](https://drive.google.com/drive/folders/0B504R2Bs0QpzcVlBRXhsREVLMFE)

See also:
  * [Full parts list (PDF)](https://github.com/andybarry/flight-cad/blob/master/TBSC/PartsList.pdf)
  * [Drake toolbox](http://drake.mit.edu) for planning, control, and analysis
  * [Simulation code](https://github.com/andybarry/simflight)
  * [CAD](https://github.com/andybarry/flight-cad/tree/master/TBSC)
  * [Pronto state estimator](https://github.com/ipab-slmc/pronto-distro)

Contact <abarry@csail.mit.edu> / [abarry.org](http://abarry.org) for more information.
