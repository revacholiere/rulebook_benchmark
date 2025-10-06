import sys
import os
import json

from reasonable_crowd.reasonable_crowd_inplaceRulebook import InPlaceRulebook
sys.path.append(os.path.abspath(".."))
from reasonable_crowd.parse_trajectory import parse_trajectory
from reasonable_crowd.parse_map import parse_map
from rulebook_benchmark.process_trajectory import process_trajectory
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed
import os
import networkx as nx
import pickle
from rulebook_benchmark.realization import VariableHandler
from rulebook_benchmark.rule_functions import RuleEngine
from rulebook_benchmark.rule_functions import (
    f1, f2, f3, f4, f5, f6, f7, f8, f9, f11, f12, f13, f15, f17, f18, Result
)

from IPython.display import display, HTML
from pprint import pprint
from rulebook_benchmark.rulebook import Rulebook
from networkx import topological_sort
from rulebook_benchmark.rulebook import Relation
from tqdm import tqdm
import cProfile, pstats
path_to_reasonable_crowd = "../../../Reasonable-Crowd"
map_directory = path_to_reasonable_crowd + '/maps'
trajectory_directory = path_to_reasonable_crowd + '/trajectories'

network_U = parse_map(map_directory, 'U')
network_S = parse_map(map_directory, 'S')

output_directory = 'outputs'
output_file = os.path.join(output_directory, 'results_scenic.txt')

from reasonable_crowd.dataset import load_all_trajectories






# check if trajectories.pkl exists
if os.path.exists(os.path.join(output_directory, 'trajectories.pkl')):
    with open(os.path.join(output_directory, 'trajectories.pkl'), 'rb') as f:
        trajectories = pickle.load(f)
else:
    profiler = cProfile.Profile()
    profiler.enable()
    trajectories = load_all_trajectories(
        trajectory_directory,
        network_U,
        network_S,
        step_size=100000,
    max_workers=8   # adjust depending on your CPU
)
    with open(os.path.join(output_directory, 'trajectories.pkl'), 'wb') as f:
        pickle.dump(trajectories, f)
        
        
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats("cumtime")
    stats.print_stats(30)  # top 30 slowest calls
    
    
    
trajectories_dict = {}
for filename, realization in trajectories:
    trajectories_dict[filename[:-5]] = realization  # remove .json extension
    
    


def get_rule_violations(realization):

    handler = VariableHandler(realization)
    rule_engine = RuleEngine(ruleset)

    # Evaluate rules
    results = rule_engine.evaluate(handler)

    return results





rb = Rulebook(rule_file="reasonable_crowd_rule_functions.py", rulebook_file="reasonable_crowd_4.graph")
rb.verbosity = 0
sorted_rule_ids = list(topological_sort(rb.priority_graph))

ruleset = {"vru_collision": f1, "vehicle_collision": f2, "drivable_area": f3, "vru_ttc": f4, "vru_acknowledgement": f5, "vehicle_ttc": f6, "correct_side": f7, "vru_offroad": f8, "vru_onroad": f9, "front_clearance_buffer": f11, "left_clearance_buffer": f12, "right_clearance_buffer": f13, "speed_limit": f15, "lane_keeping": f17, "lane_centering": f18}
rule_id_to_name = {1: "vru_collision", 2: "vehicle_collision", 3: "drivable_area", 4: "vru_ttc", 5: "vru_acknowledgement", 6: "vehicle_ttc", 7:"correct_side", 8: "vru_offroad", 9: "vru_onroad", 11: "front_clearance_buffer", 12: "left_clearance_buffer", 13: "right_clearance_buffer", 15: "speed_limit", 17: "lane_keeping", 18: "lane_centering"}
rule_name_to_id = {v: k for k, v in rule_id_to_name.items()}
rule_id_to_rule = {rule_id: ruleset[rule_name] for rule_id, rule_name in rule_id_to_name.items()}

idx_to_rule_id = list(rule_id_to_name.keys())
rule_id_to_idx = {rule_id: idx for idx, rule_id in enumerate(idx_to_rule_id)}
sorted_rule_ids = list(topological_sort(rb.priority_graph))
rule_names = list(rule_id_to_name.values())

results = {}
dataset = []


with open(output_file, 'r') as f:
    for line in f:
        parts = line.strip().split()
        filename = parts[0].split(".")[0]
        results[filename] = list(map(float, parts[1:]))
f.close()



annotation_file = os.path.join(path_to_reasonable_crowd, "annotations/annotations.json")
with open(annotation_file, 'r') as f:
    data = json.load(f)
f.close()

#print(len(data.items()))
for scenario, annotations in data.items():
    #print(len(annotations.items()))
    evaluated_pairs = set()
    for pair, votes in list(annotations.items()):
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

            dataset.append((t1,t2, 0))
        elif votes_1 < votes_2:
            human_pref = Relation.SMALLER

            dataset.append((t1,t2, 1))
        else:
            human_pref = Relation.EQUAL
        




dataset = np.array(dataset)
print("Dataset size:", len(dataset))






# Count number of '0's and '1's in the third column of dataset
num_zeros = np.sum(dataset[:, 2] == '0')
num_ones = np.sum(dataset[:, 2] == '1')

print(f"Number of 0s: {num_zeros}")
print(f"Number of 1s: {num_ones}")


        
        
            
def evaluate_rulebook(rulebook, arr):
    evaluations = {}
    pbar = tqdm(total=len(trajectories_dict), desc="scoring trajectories", leave=False)
    for name, traj in trajectories_dict.items():
        evaluations[name] = rulebook.evaluate(traj)
        pbar.update(1)
    pbar.close()
    
    
    correct = 0
    incorrect = 0
    equal = 0
    incomparable = 0
    total = len(arr)
    pbar = tqdm(total=len(arr), desc="Evaluating rulebook", leave=False)
    for t1, t2, label in arr:
        model_pref = rulebook.compare_results(evaluations[t1], evaluations[t2])
        label = int(label)
        if model_pref == label:
            correct += 1
        if model_pref != label:
            incorrect += 1
        if model_pref == 2:
            equal += 1
        if model_pref == -1:
            incomparable += 1
        pbar.update(1)
    pbar.close()
    return correct, incorrect, equal, incomparable, total
import time

def evaluate_rulebook_with_cache(rulebook, arr, rule_parameter_result_dict):

    #    pbar.update(1)
    #pbar.close()
    
    
    correct = 0
    incorrect = 0
    equal = 0
    incomparable = 0
    total = len(arr)
    #pbar = tqdm(total=len(arr), desc="Evaluating rulebook", leave=False)
    e = 0
    for t1, t2, label in arr:
        e += 1
        # time the comparison
        #print("before comparison", e)
        evaluation1 = rulebook.evaluate_with_cache(rule_parameter_result_dict, t1)
        evaluation2 = rulebook.evaluate_with_cache(rule_parameter_result_dict, t2)
        model_pref = rulebook.compare_results(evaluation1, evaluation2)
        #print("after comparison", e)
        label = int(label)
        if model_pref == label:
            correct += 1
        if model_pref != label:
            incorrect += 1
        if model_pref == 2:
            equal += 1
        if model_pref == -1:
            incomparable += 1
        #pbar.update(1)
    #pbar.close()
    return correct, incorrect, equal, incomparable, total
    
    
rbook = InPlaceRulebook(rb.priority_graph, rule_id_to_rule)



def get_rule_params(rule_id, ruleset, rule_id_to_params, rule_id_to_name):
    """
    Get the current parameters for a given rule.
    """
    rule_name = rule_id_to_name[rule_id]
    rule_fn = ruleset[rule_name]
    param_names = rule_id_to_params[rule_id]
    return {p: rule_fn.parameters[p] for p in param_names}


def set_rule_params(rule_id, param_values, ruleset, rule_id_to_params, rule_id_to_name):
    """
    Set new parameters for a given rule.
    """
    rule_name = rule_id_to_name[rule_id]
    rule_fn = ruleset[rule_name]
    param_names = rule_id_to_params[rule_id]

    for p in param_names:
        if p in param_values:
            rule_fn.parameters[p] = param_values[p]




def evaluate_rule(rule, arr):
    correct = 0
    remaining = []
    # use tqdm
    pbar = tqdm(total=len(arr), desc="Evaluating rules", leave=False)
    for item in arr:
        t1, t2, label = item
        trajectory1 = trajectories_dict[t1]
        trajectory2 = trajectories_dict[t2]
        handler1 = VariableHandler(trajectory1)
        handler2 = VariableHandler(trajectory2)
        r1 = rule.evaluate(handler1)
        r2 = rule.evaluate(handler2)

        if r1 < r2 and label == '0':
            correct += 1
        elif r1 > r2 and label == '1':
            correct += 1
        else:
            remaining.append(item)
        pbar.update(1)
    pbar.close()
    return correct, np.array(remaining)


def evaluate_rule_with_cache(rule, arr, rule_parameter_result_dict, rule_id):
    evaluations = {}
    

    pbar = tqdm(total=len(trajectories_dict), desc="scoring trajectories",
               leave=False)
    for name, traj in trajectories_dict.items():
        evaluations[name] = rule.evaluate_with_cache(
            VariableHandler(traj), rule_parameter_result_dict, name, rule_id)
        
        pbar.update(1)
    pbar.close()
    
    correct = 0
    remaining = []
    
    pbar = tqdm(total=len(arr), desc="Evaluating rules", leave=False)
    for item in arr:
        t1, t2, label = item
        r1 = evaluations[t1]
        r2 = evaluations[t2]

        if r1 < r2 and label == '0':
            correct += 1
        elif r1 > r2 and label == '1':
            correct += 1
        else:
            remaining.append(item)
        pbar.update(1)
    pbar.close()
    return correct, np.array(remaining)



import itertools

def optimize_rules_grid(
    rbook,
    rule_id_to_params,
    rule_id_to_name,
    rule_id_to_values,
    all_samples,
    verbose=True,
    rule_parameter_result_dict=None
):
    """
    Grid-search optimizer for rule parameters in priority order.
    - ruleset: dict(rule_name -> rule_object)
    - priority_order: list of rule_ids (highest -> lowest)
    - rule_id_to_params: dict rule_id -> [param_name, ...]
    - rule_id_to_name: dict rule_id -> rule_name
    - rule_id_to_values: {rule_id: {param_name: [list of candidate values]}}
    - evaluate_rule: callable(rule, samples) -> (correct_count, remaining_samples)
    - all_samples: array of tuples (t1, t2, label)
    - rulebook: Rulebook instance (must implement get_rule_relation)
    """
    best_config = {}
    rule_id_to_samples = {}
    first_node = rbook.root_node
    rule_id_to_samples[first_node] = all_samples
    priority_order = list(nx.topological_sort(rbook.in_place_priority_graph))
    priority_graph = rbook.in_place_priority_graph
    
    pbar = tqdm(total=len(priority_order), desc="Grid tuning rules", leave=False)
    for i, rule_id in enumerate(priority_order):
        # show rule id 
        pbar.set_description(f"Optimizing rule {rule_id})")
        current_samples = rule_id_to_samples[rule_id]
        rule_name = rule_id_to_name[rule_id]
        rule = ruleset[rule_name]

        if verbose:
            print(f"\n[optimize_rules_grid] Evaluating {rule_name} (id={rule_id})")

        param_names = rule_id_to_params.get(rule_id, [])
        value_lists = rule_id_to_values.get(rule_id, {})

        # Case 1: rule has tunable params → grid search
        if param_names and value_lists:
            best_rule_score = -1
            best_rule_params = None
            best_remaining = None

            for values in itertools.product(*(value_lists[p] for p in param_names)):
                trial_params = dict(zip(param_names, values))
                set_rule_params(rule_id, trial_params, ruleset, rule_id_to_params, rule_id_to_name)

                score, remaining = evaluate_rule_with_cache(rule, current_samples, rule_parameter_result_dict, rule_id)
                if score > best_rule_score:
                    best_rule_score = score
                    best_rule_params = trial_params
                    best_remaining = remaining

            # Commit best params
            set_rule_params(rule_id, best_rule_params, ruleset, rule_id_to_params, rule_id_to_name)
            best_config[rule_id] = best_rule_params
            remaining = best_remaining

            if verbose:
                print(f"  Best params: {best_rule_params}, score={best_rule_score}, remaining={len(remaining)}")

        # Case 2: no tunable params → just evaluate
        else:
            score, remaining = evaluate_rule_with_cache(rule, current_samples, rule_parameter_result_dict, rule_id)

            if verbose:
                print(f"  Fixed params, score={score}, remaining={len(remaining)}")

        neighbors = list(priority_graph.neighbors(rule_id))
        for neighbor in neighbors:
            if rule_id_to_samples.get(neighbor) is None:
                rule_id_to_samples[neighbor] = remaining
            else:
                rule_id_to_samples[neighbor] = np.unique(np.vstack((rule_id_to_samples[neighbor], remaining)), axis=0)

        pbar.update(1)

    pbar.close()

    if verbose:
        print("\n[optimize_rules_grid] Optimization finished.")
        print("Best config by rule id:")
        for rid, params in best_config.items():
            print(f"  {rid} -> {params}")

    return best_config





def optimize_rulebook_grid_bruteforce(rulebook, dataset, rule_id_to_params, rule_id_to_values, rule_parameter_result_dict=None, verbose=0):
    """
    Brute-force optimizer: tries all parameter combinations for all rules at once.

    - rulebook: Rulebook object (must expose in_place_priority_graph)
    - dataset: evaluation samples
    - rule_id_to_params: dict rule_id -> [param_name, ...]
    - rule_id_to_values: dict rule_id -> {param_name: [candidate_values]}
    """
    graph = rulebook.in_place_priority_graph

    # Collect parameter search space
    search_space = []
    param_keys = []  # list of (rule_id, param_name)
    if rule_parameter_result_dict is None:
        rule_parameter_result_dict = {}

    for rule_id, param_names in rule_id_to_params.items():
        value_lists = rule_id_to_values.get(rule_id, {})
        for p in param_names:
            if p not in value_lists:
                raise ValueError(f"No candidate values provided for {p} in rule {rule_id}")
            search_space.append(value_lists[p])
            param_keys.append((rule_id, p))

    total_combos = 1
    for vals in search_space:
        total_combos *= len(vals)

    #if verbose:
    #    print(f"[optimize_rulebook_grid_bruteforce] Searching {total_combos} total combinations...")

    best_config = {}
    best_score = 0
    #if verbose:
    #    print(f"  Initial score = {best_score:.6f}")

    # Progress bar around the Cartesian product
    iterator = itertools.product(*search_space)

    pbar = tqdm(iterator, total=total_combos, desc="Grid Search", leave=False)

    # Try all combinations
    max_time = 0
    c = 0
    for combo in iterator:
        c += 1
        trial_config = {}
        for (rule_id, p), v in zip(param_keys, combo):
            trial_config.setdefault(rule_id, {})[p] = v

        # Apply this configuration to the rulebook
        for rule_id, params in trial_config.items():
            current_rule = graph.nodes[rule_id]['rule']
            current_rule.parameters.update(params)

        # time the evaluation


        score, incorrect, equal, incomparable, total = evaluate_rulebook_with_cache(rulebook, dataset, rule_parameter_result_dict)


        if verbose >= 2:
            print(f"  Trial {trial_config} -> {score:.6f}")

        if score > best_score:
            best_score = score
            best_config = trial_config

        pbar.update(1)
    #if verbose:
    #    print("\n[optimize_rulebook_grid_bruteforce] Finished.")
    #    print(f"Best score: {best_score:.6f}")
    #    print("Best config:")
    #    for rid, params in best_config.items():
    #        print(f"  Rule {rid} -> {params}")

    return best_config, best_score


sorted_rule_ids = list(topological_sort(rb.priority_graph))
rule_id_to_params = {4: ["threshold"], 6: ["threshold"], 8: ["threshold"], 9: ["threshold"], 5: ["velocity", "threshold", "timesteps"], 11: ["threshold"], 12: ["threshold"], 13: ["threshold"], 18: ["buffer"]}
rule_id_to_values = {4: {"threshold": [0.3, 0.5, 0.8, 1]}, 6: {"threshold": [0.3, 0.5, 0.8, 1]}, 8: {"threshold": [0.5, 1, 1.5, 2]}, 9: {"threshold": [0.5 , 1, 1.5, 2]}, 5: {"velocity": [4], "threshold": [-1, -0.5, -0.2, 0], "timesteps": [30]}, 11: {"threshold": [0.5, 0.8, 1, 1.5]}, 12: {"threshold": [0.5, 0.8, 1, 1.5]}, 13: {"threshold": [0.5, 0.8, 1, 1.5]}, 18: {"buffer": [0.3, 0.5, 0.8]}}
tuning_file = os.path.join(output_directory, 'tuning.txt')


cache_path = os.path.join(output_directory, 'cache_dict.pkl')
if os.path.exists(cache_path):
    with open(cache_path, 'rb') as f:
        cache_dict = pickle.load(f)
else:
    cache_dict = {}


    best_config = optimize_rules_grid(
        rbook,
        rule_id_to_params,
        rule_id_to_name,
        rule_id_to_values,
        dataset,
        rule_parameter_result_dict=cache_dict,
        verbose=False   
    )

    with open(os.path.join(output_directory, 'cache_dict.pkl'), 'wb') as f:
        pickle.dump(cache_dict, f)


best_config, best_score = optimize_rulebook_grid_bruteforce(rbook, dataset, rule_id_to_params, rule_id_to_values, rule_parameter_result_dict=cache_dict)
with open(os.path.join(output_directory, 'best_config.pkl'), 'wb') as f:
    pickle.dump(best_config, f)

print(f"Best score: {best_score}")