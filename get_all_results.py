from reasonable_crowd.parse_map import parse_map
from reasonable_crowd.dataset import build_evaluation_dataset, get_trajectories, load_annotations
import os
from rulebook_benchmark.realization import VariableHandler
from rulebook_benchmark.rule_functions import RuleEngine
from rulebook_benchmark.rule_functions import (
    f1, f2, f3, f4, f5, f6, f7, f8, f9, f11, f12, f13, f15, f17, f18, Result
)
from rulebook_benchmark.rulebook import Rulebook
from reasonable_crowd.InPlaceRulebook import InPlaceRulebook
import numpy as np
import pandas as pd
from reasonable_crowd.optimization import cache_rule_evaluations, optimize_rulebook_grid_bruteforce_with_validation, simulated_annealing, simulated_annealing_with_validation, number_of_unique_rulebooks, find_scenario_rulebooks, group_rulebook, optimize_rulebook_greedy_by_priority
import pickle
from sklearn.model_selection import train_test_split
from reasonable_crowd.evaluation import evaluate_rulebook_with_cache
from sklearn.model_selection import KFold
from reasonable_crowd.visualization import plot_topological_graph, plot_two_rulebooks_side_by_side

SEED = 50
NUM_RUNS = 10


path_to_reasonable_crowd = "../Reasonable-Crowd"
map_directory = path_to_reasonable_crowd + '/maps'
trajectory_directory = path_to_reasonable_crowd + '/trajectories'

network_U = parse_map(map_directory, 'U')
network_S = parse_map(map_directory, 'S')

output_directory = 'outputs'
output_file = os.path.join(output_directory, 'results_scenic.txt')

print("Getting trajectories...")

trajectories = get_trajectories(output_directory, trajectory_directory, network_U, network_S)

trajectories_dict = {}
for filename, realization in trajectories:
    trajectories_dict[filename[:-5]] = realization  # remove .json extension

print("Loading annotations...")
data = load_annotations(path_to_reasonable_crowd)

print("Building evaluation dataset...")
X, y, y_votes, y_agreement = build_evaluation_dataset(data)
# create pandas dataframe
df = pd.DataFrame(columns=['X', 'y', 'votes', 'agreement'])
df['X'] = X
df['y'] = y
df['votes'] = y_votes
df['agreement'] = y_agreement

print(df.head())

rb = Rulebook(rule_file="src/reasonable_crowd/reasonable_crowd_rule_functions.py", rulebook_file="src/reasonable_crowd/reasonable_crowd_5.graph")
rule_id_to_rule = {1: f1, 2: f2, 3: f3, 4: f4, 5: f5, 6: f6, 7: f7, 8: f8, 9: f9, 11: f11, 12: f12, 13: f13, 15: f15, 17: f17, 18: f18}
rulebook = InPlaceRulebook(rb.priority_graph, rule_id_to_rule)


rule_id_to_params = {4: ["threshold"], 6: ["threshold"], 8: ["threshold"], 9: ["threshold"], 5: ["velocity", "threshold", "timesteps"], 11: ["threshold"], 12: ["threshold"], 13: ["threshold"], 18: ["buffer"]}
rule_id_to_values = {4: {"threshold": [0.6, 0.8, 1, 1.2]}, 6: {"threshold": [0.6, 0.8, 1, 1.2]}, 8: {"threshold": [0.5, 1, 1.5, 2]}, 9: {"threshold": [0.5 , 1, 1.5, 2]}, 5: {"velocity": [3, 4, 5], "threshold": [-1.5, -1, -0.5], "timesteps": [20, 30, 40]}, 11: {"threshold": [0.4, 0.8, 1.2, 1.6]}, 12: {"threshold": [0.4, 0.8, 1.2, 1.6]}, 13: {"threshold": [0.4, 0.8, 1.2, 1.6]}, 18: {"buffer": [0.3, 0.5, 0.7]}}

if os.path.exists(os.path.join(output_directory, 'tuning_cache.pkl')):    
    print("Loading cached rule evaluations...")
    with open(os.path.join(output_directory, 'tuning_cache.pkl'), 'rb') as f:
        cache_dict = pickle.load(f)
else:
    print("No cached rule evaluations found. Starting with empty cache.")
    print("Saving default rulebook parameters...")
    default_params = {}
    for rule_id, rule in rule_id_to_rule.items():
        default_params[rule_id] = rule.parameters.copy()
        print(f"Rule {rule_id} default parameters: {rule.parameters}")
    
    cache_dict = {}
    cache_rule_evaluations(rulebook, rule_id_to_params, rule_id_to_values, X, y, cache_dict, trajectories_dict)
    pickle.dump(cache_dict, open(os.path.join(output_directory, 'tuning_cache.pkl'), 'wb'))
    
    print("Restoring default rulebook parameters...")
    for rule_id, params in default_params.items():
        rule_id_to_rule[rule_id].parameters.update(params)
        

groups = [[1, 2], [3, 7], [8, 9, 11, 12, 13], [17, 18, 15], [4, 5, 6]]
name_to_group = {"safety-critical": groups[0], "operation-limit": groups[1], "safety-enhancing": groups[2], "predictability": groups[3], "precautionary": groups[4]}
group_to_name = {tuple(value): key for key, value in name_to_group.items()}
rulebook = group_rulebook(rulebook, groups, keep_relations=True)


base_result = evaluate_rulebook_with_cache(
    rulebook,
    X,
    y,
    y_votes,
    cache_dict,
    trajectories_dict)
print()
print("Base Rulebook Results:")
print("----------------------")
print("Correct:", base_result[0])
print("Equal:", base_result[1])
print("Incomparable:", base_result[2])
print("Total:", base_result[3])
print("Accuracy:", base_result[4])
print("Weighted Accuracy:", base_result[5])
print("Accuracy out of predictions:", base_result[0]/(base_result[3]-base_result[2]) if base_result[3]-base_result[2]>0 else 0.0)
print("\n")



accuracy_list = []
weighted_accuracy_list = []
correct_list = []



for run in range(NUM_RUNS):
    # Shuffle df

    # Prepare data
    X = df['X'].tolist()
    y = df['y'].tolist()
    votes = df['votes'].tolist()

    kf = KFold(n_splits=5, shuffle=True, random_state=SEED)

    accuracy_list = []
    weighted_accuracy_list = []
    correct_list = []
    fold = 0

    for train_index, test_index in kf.split(X):
        X_train = [X[i] for i in train_index]
        y_train = [y[i] for i in train_index]
        votes_train = [votes[i] for i in train_index]

        X_test = [X[i] for i in test_index]
        y_test = [y[i] for i in test_index]
        votes_test = [votes[i] for i in test_index]

            
        # if cached best config for this fold exists, load it
        if os.path.exists(os.path.join(output_directory, f'greedy_best_config_seed_{SEED}_run_{run}_fold_{fold}.pkl')):
            best_config = pickle.load(open(os.path.join(output_directory, f'greedy_best_config_seed_{SEED}_run_{run}_fold_{fold}.pkl'), 'rb'))
        else:
        # Optimize rulebook on this fold
            best_config, best_score = optimize_rulebook_greedy_by_priority(
                rulebook,
                training_data=X_train,
                training_labels=y_train,
                training_votes=votes_train,
                rule_id_to_params=rule_id_to_params,
                rule_id_to_values=rule_id_to_values,
                trajectories_dict=trajectories_dict,
                rule_parameter_result_dict=cache_dict
            )
        
        # load config

        

        # Apply best config to the rulebook
        for rule_id, params in best_config.items():
            rule = rule_id_to_rule[rule_id]
            rule.parameters.update(params)
        
        #Save best config for this fold to a file
        with open(os.path.join(output_directory, f'greedy_best_config_seed_{SEED}_run_{run}_fold_{fold}.pkl'), 'wb') as f:
            pickle.dump(best_config, f)
        
        

        # Evaluate on test fold
        correct, equal, incomparable, total, accuracy, weighted_accuracy, reasons, predictions = evaluate_rulebook_with_cache(
            rulebook,
            X_test,
            y_test,
            votes_test,
            cache_dict,
            trajectories_dict
        )

        correct_list.append(correct)
        accuracy_list.append(accuracy)
        weighted_accuracy_list.append(weighted_accuracy)
        

        fold += 1

    
    SEED += 1



# Report averaged results
avg_correct = np.mean(correct_list)
avg_accuracy = np.mean(accuracy_list)
std_dev_accuracy = np.std(accuracy_list)
avg_weighted_accuracy = np.mean(weighted_accuracy_list)
std_dev_weighted_accuracy = np.std(weighted_accuracy_list)


print(f"5-Fold Cross-Validation Results for run {run}, seed {SEED}:")
print("----------------------")
print("Average Correct:", avg_correct)
print("Average Accuracy:", avg_accuracy)
print("Average Weighted Accuracy:", avg_weighted_accuracy)
print("Std Dev Accuracy:", std_dev_accuracy)
print("Std Dev Weighted Accuracy:", std_dev_weighted_accuracy)

print("\n")

# count reasons
# drop None values
reasons = [reason for reason in reasons if reason is not None]
np.unique(reasons, return_counts=True)
print("Reason Counts:", dict(zip(*np.unique(reasons, return_counts=True))))


print("\n")

