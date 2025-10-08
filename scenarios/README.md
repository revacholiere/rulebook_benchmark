Scenarios
===

We collect two types of scenarios in this benchmark: **basic maneuver scenarios** and **near-accident scenarios**. The former represent basic behaviors of vehicles (e.g., lane change, turns, etc.) and serve as building blocks for more complicated scenarios. The latter reconstruct scenarios from DMV crash reports and serve as critical traffic situations.

Creating a Scenic Scenario
---
We use `crash/crash_waymo_august_12_2019.scenic` as an example. To test different driving policies for the scenario, we introduce a parameter `POLICY` in each Scenic file. Currently, the value of the parameter could be either `'build_in'` or `'metadrive_ppo'`. If `POLICY` is set to  `'build_in'`, the ego behavior defined in the Scenic file is used to control the ego vehicle. Otherwise, if `POLICY` is set to `'metadrive_ppo'`, the MetaDrive PPO agent is used (see `src/agents/` for more details). The ego's behavior can be defined using the following code:

```python
from metadrive_expert import MetaDrivePPOPolicyCar, MetaDrivePPOPolicyBehavior, MetaDrivePPOUpdateState
ego = new MetaDrivePPOPolicyCar at egoSpawnPt,
    with blueprint MODEL,
    with behavior MetaDrivePPOPolicyBehavior(egoTrajectory)
require monitor MetaDrivePPOUpdateState()
```

Basic Maneuver Scenarios
---

Near-Accident Scenarios
---
### LLM-Assisted Scenic Code Generation

We provide an LLM-assisted flow to generate Scenic code from DMV crash reports written in natural language. In the prompt, we guide the LLM by providing the typical structure of a Scenic program and several crash report-Scenic code example pairs. See `scenicnl.py` for more details. To run the flow, users need to paste their Google Gemini API keys in `scenicnl.py`.
