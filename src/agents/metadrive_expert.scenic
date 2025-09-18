model scenic.simulators.metadrive.model
from metadrive_expert_agent import MetaDrivePolicyAgent, MetaDrivePolicyAction

METADRIVE_ACTOR = [None]
TRAJECTORY = [None]

class ExpertPolicyCar(Car):
    actor: None
    controller: None

    def update_actor(self):
        self.actor = METADRIVE_ACTOR[-1]
        self.trajectory = TRAJECTORY[-1]

    def startDynamicSimulation(self):
        self.controller = MetaDrivePolicyAgent()

behavior ExpertPolicyBehavior(egoTrajectory):
    action = MetaDrivePolicyAction()
    TRAJECTORY.append(egoTrajectory)
    take action
    while True:
        take action

monitor UpdateState():
    while True:
        METADRIVE_ACTOR.append(ego.metaDriveActor)
        wait