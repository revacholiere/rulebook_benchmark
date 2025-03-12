A benchmarking tool to evaluate Scenic scenarios with respect to a user-defined rulebook.

Scenario             |  Violation Plot
:-------------------------:|:-------------------------:
![](https://github.com/revacholiere/rulebook_benchmark/blob/main/example/close_vru.gif)  |  ![](https://github.com/revacholiere/rulebook_benchmark/blob/main/example/close_vru.png)
![](https://github.com/revacholiere/rulebook_benchmark/blob/main/example/pedestrian_collision.gif) | ![](https://github.com/revacholiere/rulebook_benchmark/blob/main/example/pedestrian_collision.png)
![](https://github.com/revacholiere/rulebook_benchmark/blob/main/example/vehicle_collision.gif) | ![](https://github.com/revacholiere/rulebook_benchmark/blob/main/example/vehicle_collision.png)

**Scenario 1**: 
- Ego vehicle refuses to slow down for the pedestrian in close proximity of its path, violating the VRU acknowledgement rule.
- Ego vehicle passes by the pedestrian standing on the sidewalk, violating the VRU off-road clearance rule.

**Scenario 2**:
- Ego vehicle first refuses to slow down for the pedestrian on its path, violating the VRU acknowledgement rule.
- Ego vehicle comes too close to the pedestrian crossing the road, violating the VRU on-road clearance rule.
- Ego vehicle hits the pedestrian, violating the VRU collision rule.
- At the end of the simulation, the ego vehicle drives on the sidewalk, violating the stay in the drivable area rule.

**Scenario 3**:
- Ego vehicle first hits the adversary vehicle in front of it, violating the vehicle collision rule.
- Ego vehicle keeps accelerating into the adversary vehicle, causing the violation score to increase further.
- Ego vehicle gets out of the drivable area, violating the stay in the drivable area rule.


(Red and blue rectangles are the ego vehicle and adversary vehicles respectively, while the purple dots are pedestrians.)
