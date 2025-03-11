from rulebook_benchmark.rule_functions import *

class Rulebook:
    def __init__(self, rules):
        self.rules = rules
    
    def apply(self, realization):
        violations = []
        violation_histories = []
        for rule in self.rules:
            violation, violation_history = rule(realization)
            violations.append(violation)
            violation_histories.append(violation_history)
            
        return violations, violation_histories
    
    def __call__(self, realization):
        return self.apply(realization)
    
    @property
    def rule_names(self):
        return [rule.name for rule in self.rules]
    
    #TODO: add/remove rule function
    

    
    
class Rule:
    def __init__(self, rule_function, name, description, args={}):
        self.rule = rule_function
        self.name = name
        self.description = description
        self.args = args
    
    def __call__(self, realization):
        return self.rule(realization, **self.args)
    


vru_collision_rule = Rule(rule_function=rule_vru_collision, name="VRU Collision", description="No collision with VRUs.")
vehicle_collision_rule = Rule(rule_function=rule_vehicle_collision, name="Vehicle Collision", description="No collision with vehicles.", args={"start_index": 0, "end_index": None})
stay_in_drivable_area_rule = Rule(rule_function=rule_stay_in_drivable_area, name="Stay in Drivable Area", description="Ego vehicle has to stay in drivable area.", args={"start_index": 0, "end_index": None})
vru_clearance_on_road_rule = Rule(rule_function=vru_clearance_on_road, name="VRU Clearance on road", description="Ego vehicle has to keep a minimum distance to VRUs on road.", args={"start_index": 0, "end_index": None})
vru_clearance_off_road_rule = Rule(rule_function=vru_clearance_off_road, name="VRU Clearance off road", description="Ego vehicle has to keep a minimum distance to VRUs off road.", args={"start_index": 0, "end_index": None})
vru_acknowledgement_rule = Rule(rule_function=vru_acknowledgement, name="VRU Acknowledgement", description="Ego vehicle has to slow down if a VRU is on its planned trajectory.", args={"start_index": 0, "end_index": None})

default_rulebook = Rulebook([vru_collision_rule, vehicle_collision_rule, stay_in_drivable_area_rule, vru_clearance_on_road_rule, vru_clearance_off_road_rule, vru_acknowledgement_rule])

