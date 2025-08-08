import sys
import os
import json
sys.path.append(os.path.abspath(".."))
from rulebook_benchmark.rule_functions import (
    f1, f2, f3, f4, f5, f6, f7, f8, f9, f11_a, f11_b, f12_a, f12_b, f13_a, f13_b, f15, f17, f18
)
from reasonable_crowd.parse_trajectory import parse_trajectory
from reasonable_crowd.parse_map import parse_map
from rulebook_benchmark.process_trajectory import process_trajectory
from rulebook_benchmark.rulebook import Rulebook
from rulebook_benchmark.rulebook import Relation

path_to_reasonable_crowd = "../../../Reasonable-Crowd"
map_directory = path_to_reasonable_crowd + '/maps'
trajectory_directory = path_to_reasonable_crowd + '/trajectories'

network_U = parse_map(map_directory, 'U')
network_S = parse_map(map_directory, 'S')

output_directory = 'outputs'
output_file = os.path.join(output_directory, 'results_scenic.txt')

# Collect evaluation results
f = open(output_file, 'w')
def get_rule_violations(realization):
    results = {}

    results['vru_collision'] = f1(realization)
    results['vehicle_collision'] = f2(realization)
    results['drivable_area'] = f3(realization)
    results['vru_ttc'] = f4(realization)
    results['vru_acknowledgement'] = f5(realization)
    results['vehicle_ttc'] = f6(realization)
    results['correct_side'] = f7(realization)
    results['vru_offroad'] = f8(realization)
    results['vru_onroad'] = f9(realization)
    results['front_clearance_buffer'] = f11_a(realization)
    results['left_clearance_buffer'] = f12_a(realization)
    results['right_clearance_buffer'] = f13_a(realization)
    results['speed_limit'] = f15(realization)
    results['lane_keeping'] = f17(realization)
    results['lane_centering'] = f18(realization)
    
    return results

count = 0
for filename in os.listdir(trajectory_directory):
    if not filename.endswith('.json'):
        continue
    traj_path = os.path.join(trajectory_directory, filename)
    if filename.startswith('U'):
        print(f"Evaluating trajectory {filename} on map U")
        realization = parse_trajectory(traj_path, step_size=100000)
        realization.network = network_U
    else:
        print(f"Evaluating trajectory {filename} on map S")
        realization = parse_trajectory(traj_path, step_size=100000)
        realization.network = network_S
    results = get_rule_violations(realization)
    f.write(f"{filename}")
    for _, result in results.items():
        f.write(f" {result.total_violation}")
    f.write("\n")

f.close()

# Compare with human annotations
rb = Rulebook(rule_file="reasonable_crowd_rule_functions.py", rulebook_file="reasonable_crowd_3.graph")

results = {}
with open(output_file, 'r') as f:
    for line in f:
        parts = line.strip().split()
        filename = parts[0].split(".")[0]
        results[filename] = list(map(float, parts[1:]))
f.close()

correct = 0
total = 0
weighted_correct = 0
weighted_total = 0

human_euqal_count = 0
human_non_equal_count = 0
model_equal_count = 0
model_non_equal_count = 0
model_non_comparable_count = 0

annotation_file = os.path.join(path_to_reasonable_crowd, "annotations/annotations.json")
with open(annotation_file, 'r') as f:
    data = json.load(f)
f.close()

for scenario, annotations in data.items():
    evaluated_pairs = set()
    for pair, votes in annotations.items():
        t1, t2 = pair.split(" ;; ")
        reverse_pair = f"{t2} ;; {t1}"
        reverse_votes = annotations.get(reverse_pair, [])
        if pair in evaluated_pairs or reverse_pair in evaluated_pairs:
            continue
        if not (t1 in results and t2 in results):
            continue
        evaluated_pairs.add(pair)
        evaluated_pairs.add(reverse_pair)
        
        votes_1 = len(votes)
        votes_2 = len(reverse_votes)
        votes_total = votes_1 + votes_2
        
        if votes_1 > votes_2:
            human_pref = Relation.LARGER
            human_non_equal_count += 1
        elif votes_1 < votes_2:
            human_pref = Relation.SMALLER
            human_non_equal_count += 1
        else:
            human_pref = Relation.EQUAL
            human_euqal_count += 1
        
        model_pref = rb.compare_trajectories((t1, results), (t2, results))
        if model_pref == Relation.EQUAL:
            model_equal_count += 1
        elif model_pref == Relation.NONCOMPARABLE:
            model_non_comparable_count += 1
        else:
            model_non_equal_count += 1
        
        if model_pref == Relation.EQUAL or model_pref == Relation.NONCOMPARABLE or human_pref == Relation.EQUAL:
            continue
        is_correct = (human_pref == model_pref)
        correct += int(is_correct)
        total += 1
        if not is_correct:
            print(f"Error in {scenario}: {pair} - Model: {model_pref}, Human: {human_pref}")
        weighted_correct += votes_1 if is_correct and human_pref == Relation.LARGER else 0
        weighted_correct += votes_2 if is_correct and human_pref == Relation.SMALLER else 0
        weighted_total += max(votes_1, votes_2)
        
print(total, correct)

print("Accuracy:")
print("Pairwise accuracy:", correct / total if total > 0 else 0)
print("Weighted accuracy:", weighted_correct / weighted_total if weighted_total > 0 else 0)
print("Total pairs:", total)
print("Total weighted pairs:", weighted_total)

print("\nStatistics:")
print("Model discernment:", model_non_equal_count / (model_equal_count + model_non_equal_count + model_non_comparable_count))
print("Human discernment:", human_non_equal_count / (human_euqal_count + human_non_equal_count))
print("Human equal count:", human_euqal_count)
print("Human non-equal count:", human_non_equal_count)
print("Model equal count:", model_equal_count)
print("Model non-equal count:", model_non_equal_count)
print("Model non-comparable count:", model_non_comparable_count)
