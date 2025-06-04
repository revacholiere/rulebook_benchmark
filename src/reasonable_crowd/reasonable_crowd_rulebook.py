import sys
import os
import json
sys.path.append(os.path.abspath(".."))
from rulebook_benchmark.rulebook import Rulebook
from rulebook_benchmark.rulebook import Relation

rb = Rulebook(rule_file="reasonable_crowd_rule_functions.py", rulebook_file="reasonable_crowd_2.graph")

result_path = "outputs/results_1_.8_14_.3_.8_1.txt"
results = {}
with open(result_path, 'r') as f:
    for line in f:
        parts = line.strip().split()
        filename = parts[0]
        results[filename] = list(map(float, parts[1:]))
f.close()

path_to_reasonable_crowd = "../../../Reasonable-Crowd"
annotation_file = os.path.join(path_to_reasonable_crowd, "annotations/annotations.json")
with open(annotation_file, 'r') as f:
    data = json.load(f)
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
        
print("Accracy:")
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

rb.visualize_rulebook(output_file_name='outputs/rulebook_visualization_1.png')
