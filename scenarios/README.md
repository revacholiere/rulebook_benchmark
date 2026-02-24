Scenarios
===

We collect two types of scenarios in this benchmark: **common scenarios** and **near-accident scenarios**. The former represent common behaviors of vehicles (e.g., lane change, turns, etc.) and serve as building blocks for more complicated scenarios. The latter reconstruct scenarios from DMV crash reports and serve as critical traffic situations.

Common Scenarios
---

Near-Accident Scenarios
---
### LLM-Assisted Scenic Code Generation

We provide an LLM-assisted pipeline to generate executable Scenic code directly from DMV crash reports written in natural language. Our prompting strategy guides the LLM by outlining the core structure of a Scenic program and providing several few-shot examples that map natural language crash reports to Scenic code. For complete implementation details, please refer to `scenicnl.py`.

To execute the generation pipeline, follow these steps:
1. Configure the API and Model: Create a file named `api.txt`. Insert your Google Gemini API key on the first line, and specify the target AI model (e.g., gemini-2.5-flash) on the second line.
2. Prepare the Input: Create a text file containing the natural language description of your target scenario.
3. Execute the Script: Run the generation script using the following command:
```bash
python scenicnl.py api.txt <input_file_path> <output_file_path>
```
where <input_file_path> is the path to the file containing your natural language description (e.g., `scenario.txt`), and <output_file_path> is the desired output path for the generated Scenic program (e.g., `scenario.scenic`).

### Example Scenarios

We provide 27 example near-accident scenarios located in the `crash/` directory. The crash reports for these examples are curated from the "hard" tier of the [ScenicNL dataset](https://github.com/KE7/ScenarioNL-CA-AV-Crash). Additional raw reports can be accessed through the [California DMV Autonomous Vehicle Collision Reports](https://www.dmv.ca.gov/portal/vehicle-industry-services/autonomous-vehicles/autonomous-vehicle-collision-reports/) database.


Driving Policies
---
To test different driving policies for the scenario, we introduce a parameter `POLICY` in each Scenic file. Currently, we support three policies:
1. `'built_in'`: Use the behaviors defined in the Scenic files to control the ego vehicle. The behaviors are basically rule-based planners with PID controllers.
2. `'metadrive_ppo'`: Use the MetaDrive PPO agent to control the ego vehicle (see `src/evaluation/` for more details). We assume the trajectory of the ego vehicle is given. Below is an example of how to set the ego's behavior to `MetaDrivePPOPolicyBehavior`.
```scenic
from metadrive_expert import MetaDrivePPOPolicyCar, MetaDrivePPOPolicyBehavior, MetaDrivePPOUpdateState
ego = new MetaDrivePPOPolicyCar at egoSpawnPt,
    with blueprint MODEL,
    with behavior MetaDrivePPOPolicyBehavior(egoTrajectory)
require monitor MetaDrivePPOUpdateState()
```
3. `'ppo_with_built_in'`: Sometimes if the ego's trajectory is not available, we may need to use rule-based planners to guide the ego vehicle, while using the MetaDrive PPO agent to perform the control. Here is an example, where the MetaDrive PPO agent only focuses on the lane following parts:
```scenic
behavior EgoPPOBehavior():
    try:
        do MetaDrivePPOFollowLaneBehavior()
    interrupt when withinDistanceToAnyObjs(self, BYPASS_DIST[0]):
        fasterLaneSec = self.laneSection.fasterLane
        do LaneChangeBehavior(
                laneSectionToSwitch=fasterLaneSec,
                target_speed=globalParameters.EGO_SPEED)
        do MetaDrivePPOFollowLaneBehavior() \
            until (distance to adversary) > BYPASS_DIST[1] and (apparent heading of adversary) > 1.57
        slowerLaneSec = self.laneSection.slowerLane
        do LaneChangeBehavior(
                laneSectionToSwitch=slowerLaneSec,
                target_speed=globalParameters.EGO_SPEED)
        do MetaDrivePPOFollowLaneBehavior() for TERM_TIME seconds
        terminate 
```
