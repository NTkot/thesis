# Detection and mapping of road surface damage using sensors and smart devices

Α well-developed road network is essential for the efficient movement of people and goods. Therefore, maintenance of road networks is paramount to ensure their continued functionality and the economic and social wellbeing they support. When it comes to road maintenance, a necessary condition for optimal resource allocation is the mapping of the road condition. 

In this work, an integrated system for road surface damage detection is presented. Τhe detection of anomalies is based on the vibrations received by a moving vehicle the moment it passes on top of a defect. The vibrations are sensed by an inertia measurement unit (IMU), while the geographic location of the damage is determined using a GPS sensor. 

The system is based on an embedded device designed as part of this diploma thesis which was used in real-life conditions to collect data and conduct experiments. For the development of the related software, the ROS2 framework was used. 

The implementation addresses several challenges of a real-world scenario, such as the existence of noise in the sensor signals and the random orientation in which the device is placed on the vehicles. A heuristic algorithmic approach was developed to identify road imperfections through the IMU measurements. The analysis on hand-labeled data showed that a F1-Measure score of ≈ 0.83 is achieved when it comes to identifying anomalies in the signal. 

Finally, a logic for storing road anomalies in a database is presented that clusters its entries to extract a better estimate of the characteristics of a road defect that is met more than once in the data.

## Repo structure

