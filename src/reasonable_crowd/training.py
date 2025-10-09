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
from reasonable_crowd.optimization import cache_rule_evaluations, optimize_rulebook_grid_bruteforce_with_validation, simulated_annealing, simulated_annealing_with_validation, number_of_unique_rulebooks, find_scenario_rulebooks
import pickle
from sklearn.model_selection import train_test_split
from reasonable_crowd.evaluation import evaluate_rulebook_with_cache
from sklearn.model_selection import KFold

path_to_reasonable_crowd = "../../../Reasonable-Crowd"
map_directory = path_to_reasonable_crowd + '/maps'
trajectory_directory = path_to_reasonable_crowd + '/trajectories'

network_U = parse_map(map_directory, 'U')
network_S = parse_map(map_directory, 'S')

output_directory = 'outputs'
output_file = os.path.join(output_directory, 'results_scenic.txt')

trajectories = get_trajectories(output_directory, trajectory_directory, network_U, network_S)

trajectories_dict = {}
for filename, realization in trajectories:
    trajectories_dict[filename[:-5]] = realization  # remove .json extension

data = load_annotations(path_to_reasonable_crowd)

X, y, y_votes = build_evaluation_dataset(data)
# create pandas dataframe
df = pd.DataFrame(columns=['X', 'y', 'votes'])
df['X'] = X
df['y'] = y
df['votes'] = y_votes

print(df.head())

rb = Rulebook(rule_file="reasonable_crowd_rule_functions.py", rulebook_file="reasonable_crowd_4.graph")
rule_id_to_rule = {1: f1, 2: f2, 3: f3, 4: f4, 5: f5, 6: f6, 7: f7, 8: f8, 9: f9, 11: f11, 12: f12, 13: f13, 15: f15, 17: f17, 18: f18}
rulebook = InPlaceRulebook(rb.priority_graph, rule_id_to_rule)

rule_id_to_params = {4: ["threshold"], 6: ["threshold"], 8: ["threshold"], 9: ["threshold"], 5: ["velocity", "threshold", "timesteps"], 11: ["threshold"], 12: ["threshold"], 13: ["threshold"], 18: ["buffer"]}
rule_id_to_values = {4: {"threshold": [0.3, 0.5, 0.8, 1]}, 6: {"threshold": [0.3, 0.5, 0.8, 1]}, 8: {"threshold": [0.5, 1, 1.5, 2]}, 9: {"threshold": [0.5 , 1, 1.5, 2]}, 5: {"velocity": [4], "threshold": [-1, -0.5, -0.2, 0], "timesteps": [30]}, 11: {"threshold": [0.5, 0.8, 1, 1.5]}, 12: {"threshold": [0.5, 0.8, 1, 1.5]}, 13: {"threshold": [0.5, 0.8, 1, 1.5]}, 18: {"buffer": [0.3, 0.5, 0.8]}}

if os.path.exists(os.path.join(output_directory, 'tuning_cache.pkl')):
    print("Loading cached rule evaluations...")
    with open(os.path.join(output_directory, 'tuning_cache.pkl'), 'rb') as f:
        cache_dict = pickle.load(f)


else:
    print("No cached rule evaluations found. Starting with empty cache.")
    cache_dict = {}
    cache_rule_evaluations(rulebook, rule_id_to_params, rule_id_to_values, X, y, cache_dict, trajectories_dict)
    pickle.dump(cache_dict, open(os.path.join(output_directory, 'tuning_cache.pkl'), 'wb'))
#cache_rule_evaluations(rulebook, rule_id_to_params, rule_id_to_values, X, y, cache_dict, trajectories_dict)
#pickle.dump(cache_dict, open(os.path.join(output_directory, 'tuning_cache.pkl'), 'wb'))

""" train_df, test_df = train_test_split(df, test_size=0.2, random_state=42)
train_df, val_df = train_test_split(train_df, test_size=0.5, random_state=42)

print(len(train_df), len(val_df), len(test_df))

best_config, best_score, best_val_score = optimize_rulebook_grid_bruteforce_with_validation(
    rulebook,
    training_data=train_df['X'].tolist(),
    training_labels=train_df['y'].tolist(),
    training_votes=train_df['votes'].tolist(),
    validation_data=val_df['X'].tolist(),
    validation_labels=val_df['y'].tolist(),
    validation_votes=val_df['votes'].tolist(),
    rule_id_to_params=rule_id_to_params,
    rule_id_to_values=rule_id_to_values,
    trajectories_dict=trajectories_dict,
    rule_parameter_result_dict=cache_dict)

print("Best Config:", best_config)
print("Best Score:", best_score)
print("Best Validation Score:", best_val_score)

# get test score with best config
for rule_id, params in best_config.items():
    rule = rule_id_to_rule[rule_id]
    rule.parameters.update(params)

correct, equal, incomparable, total, accuracy, weighted_accuracy = evaluate_rulebook_with_cache(
    rulebook,
    test_df['X'].tolist(),
    test_df['y'].tolist(),
    test_df['votes'].tolist(),
    cache_dict,
    trajectories_dict)

print("Test Set Results:")
print("Correct:", correct)
print("Equal:", equal)
print("Incomparable:", incomparable)
print("Total:", total)
print("Accuracy:", accuracy)
print("Weighted Accuracy:", weighted_accuracy) """

# Shuffle df
df = df.sample(frac=1, random_state=42).reset_index(drop=True)

# Prepare data
X = df['X'].tolist()
y = df['y'].tolist()
votes = df['votes'].tolist()
""" 
kf = KFold(n_splits=5, shuffle=True, random_state=42)

accuracy_list = []
weighted_accuracy_list = []
correct_list = []
for train_index, test_index in kf.split(X):
    X_train = [X[i] for i in train_index]
    y_train = [y[i] for i in train_index]
    votes_train = [votes[i] for i in train_index]

    X_test = [X[i] for i in test_index]
    y_test = [y[i] for i in test_index]
    votes_test = [votes[i] for i in test_index]

    # Further split training into train/validation (15%)
    val_size = int(0.15 * len(X_train))
    X_val, y_val, votes_val = X_train[:val_size], y_train[:val_size], votes_train[:val_size]
    X_train, y_train, votes_train = X_train[val_size:], y_train[val_size:], votes_train[val_size:]

    # Optimize rulebook on this fold
    best_config, best_score, best_val_score = optimize_rulebook_grid_bruteforce_with_validation(
        rulebook,
        training_data=X_train,
        training_labels=y_train,
        training_votes=votes_train,
        validation_data=X_val,
        validation_labels=y_val,
        validation_votes=votes_val,
        rule_id_to_params=rule_id_to_params,
        rule_id_to_values=rule_id_to_values,
        trajectories_dict=trajectories_dict,
        rule_parameter_result_dict=cache_dict
    )

    # Apply best config to the rulebook
    for rule_id, params in best_config.items():
        rule = rule_id_to_rule[rule_id]
        rule.parameters.update(params)

    # Evaluate on test fold
    correct, equal, incomparable, total, accuracy, weighted_accuracy = evaluate_rulebook_with_cache(
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

# Report averaged results
avg_correct = np.mean(correct_list)
avg_accuracy = np.mean(accuracy_list)
avg_weighted_accuracy = np.mean(weighted_accuracy_list)

print("5-Fold Cross-Validation Results:")
print("Average Correct:", avg_correct)
print("Average Accuracy:", avg_accuracy)
print("Average Weighted Accuracy:", avg_weighted_accuracy) """

#num_rulebooks, correct, accuracy, unsatisfiable_samples = number_of_unique_rulebooks(rulebook, X, y, y_votes, cache_dict, trajectories_dict, seed = 43)

#print("Number of Unique Rulebooks:", num_rulebooks)
#print("Correct:", correct)
#print("Accuracy:", accuracy)
#print("Unsatisfiable Samples:", unsatisfiable_samples)
""" best_rb, best_score, best_val_score = simulated_annealing_with_validation(rulebook, 
    train_data=train_df['X'].tolist(),
    train_labels=train_df['y'].tolist(),
    train_votes=train_df['votes'].tolist(),
    val_data=val_df['X'].tolist(),
    val_labels=val_df['y'].tolist(),
    val_votes=val_df['votes'].tolist(),
    rule_parameter_result_dict=cache_dict,
    trajectories_dict=trajectories_dict,
    max_iter=10000,
    start_temp=300.0,
    alpha=0.999,
    seed=42)

print("Best Score after Simulated Annealing with Validation:", best_score)
print("Best Validation Score after Simulated Annealing with Validation:", best_val_score) """


find_scenario_rulebooks(rulebook, X, y, y_votes, cache_dict, trajectories_dict)