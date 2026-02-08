import os
import pickle

import numpy as np
import pandas as pd
from sklearn.model_selection import KFold

from reasonable_crowd.dataset import (build_evaluation_dataset,
                                      get_trajectories, load_annotations)
from reasonable_crowd.evaluation import evaluate_rulebook_with_cache
from reasonable_crowd.InPlaceRulebook import InPlaceRulebook
from reasonable_crowd.optimization import (
    brute_force_group_optimization, cache_rule_evaluations,
    find_scenario_groups, greedy_group_optimization, group_rulebook,
    optimize_rulebook_greedy_by_priority)
from reasonable_crowd.parse_map import parse_map
from rulebook_benchmark.rule_functions import (f1, f2, f3, f4, f5, f6, f7, f8,
                                               f9, f11, f12, f13, f15, f17,
                                               f18)
from rulebook_benchmark.rulebook import Rulebook

SEED = 50
NUM_RUNS = 10


path_to_reasonable_crowd = "../Reasonable-Crowd"
map_directory = path_to_reasonable_crowd + "/maps"
trajectory_directory = path_to_reasonable_crowd + "/trajectories"

network_U = parse_map(map_directory, "U")
network_S = parse_map(map_directory, "S")

output_directory = "outputs"
output_file = os.path.join(output_directory, "results_scenic.txt")

print("Getting trajectories...")

trajectories = get_trajectories(
    output_directory, trajectory_directory, network_U, network_S
)

trajectories_dict = {}
for filename, realization in trajectories:
    trajectories_dict[filename[:-5]] = realization  # remove .json extension

print("Loading annotations...")
data = load_annotations(path_to_reasonable_crowd)

print("Building evaluation dataset...")
X, y, y_votes, y_agreement = build_evaluation_dataset(data)
# create pandas dataframe
df = pd.DataFrame(columns=["X", "y", "votes", "agreement"])
df["X"] = X
df["y"] = y
df["votes"] = y_votes
df["agreement"] = y_agreement

print(df.head())

rb = Rulebook(
    rule_file="src/reasonable_crowd/reasonable_crowd_rule_functions.py",
    rulebook_file="src/reasonable_crowd/reasonable_crowd_5.graph",
)
rule_id_to_rule = {
    1: f1,
    2: f2,
    3: f3,
    4: f4,
    5: f5,
    6: f6,
    7: f7,
    8: f8,
    9: f9,
    11: f11,
    12: f12,
    13: f13,
    15: f15,
    17: f17,
    18: f18,
}
rulebook = InPlaceRulebook(rb.priority_graph, rule_id_to_rule)


rule_id_to_params = {
    4: ["threshold"],
    6: ["threshold"],
    8: ["threshold"],
    9: ["threshold"],
    5: ["velocity", "threshold", "timesteps"],
    11: ["threshold"],
    12: ["threshold"],
    13: ["threshold"],
    18: ["buffer"],
}
rule_id_to_values = {
    4: {"threshold": [0.6, 0.8, 1, 1.2]},
    6: {"threshold": [0.6, 0.8, 1, 1.2]},
    8: {"threshold": [0.5, 1, 1.5, 2]},
    9: {"threshold": [0.5, 1, 1.5, 2]},
    5: {
        "velocity": [3, 4, 5],
        "threshold": [-1.5, -1, -0.5],
        "timesteps": [20, 30, 40],
    },
    11: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    12: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    13: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    18: {"buffer": [0.3, 0.5, 0.7]},
}

default_params = {}
for rule_id, rule in rule_id_to_rule.items():
    default_params[rule_id] = rule.parameters.copy()
    print(f"Rule {rule_id} default parameters: {rule.parameters}")

if os.path.exists(os.path.join(output_directory, "tuning_cache.pkl")):
    print("Loading cached rule evaluations...")
    with open(os.path.join(output_directory, "tuning_cache.pkl"), "rb") as f:
        cache_dict = pickle.load(f)
else:
    print("No cached rule evaluations found. Starting with empty cache.")
    print("Saving default rulebook parameters...")
    cache_dict = {}
    cache_rule_evaluations(
        rulebook,
        rule_id_to_params,
        rule_id_to_values,
        X,
        y,
        cache_dict,
        trajectories_dict,
    )
    pickle.dump(
        cache_dict, open(os.path.join(output_directory, "tuning_cache.pkl"), "wb")
    )

    print("Restoring default rulebook parameters...")
    for rule_id, params in default_params.items():
        rule_id_to_rule[rule_id].parameters.update(params)


groups = [[1, 2], [3, 7], [8, 9, 11, 12, 13], [17, 18, 15], [4, 5, 6]]
name_to_group = {
    "safety-critical": groups[0],
    "operation-limit": groups[1],
    "safety-enhancing": groups[2],
    "predictability": groups[3],
    "precautionary": groups[4],
}
group_to_name = {tuple(value): key for key, value in name_to_group.items()}
rulebook = group_rulebook(rulebook, groups, keep_relations=True)


base_result = evaluate_rulebook_with_cache(
    rulebook, X, y, y_votes, cache_dict, trajectories_dict
)
print()
print("Base Rulebook Results:")
print("----------------------")
print("Correct:", base_result[0])
print("Equal:", base_result[1])
print("Incomparable:", base_result[2])
print("Total:", base_result[3])
print("Accuracy:", base_result[4])
print("Weighted Accuracy:", base_result[5])
print(
    "Accuracy out of predictions:",
    (
        base_result[0] / (base_result[3] - base_result[2])
        if base_result[3] - base_result[2] > 0
        else 0.0
    ),
)
print("\n")


accuracy_list = []
weighted_accuracy_list = []
correct_list = []


for run in range(NUM_RUNS):
    # Shuffle df

    # Prepare data
    X = df["X"].tolist()
    y = df["y"].tolist()
    votes = df["votes"].tolist()

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
        if os.path.exists(
            os.path.join(
                output_directory,
                f"greedy_best_config_seed_{SEED}_run_{run}_fold_{fold}.pkl",
            )
        ):
            best_config = pickle.load(
                open(
                    os.path.join(
                        output_directory,
                        f"greedy_best_config_seed_{SEED}_run_{run}_fold_{fold}.pkl",
                    ),
                    "rb",
                )
            )
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
                rule_parameter_result_dict=cache_dict,
            )

        # load config

        # Apply best config to the rulebook
        for rule_id, params in best_config.items():
            rule = rule_id_to_rule[rule_id]
            rule.parameters.update(params)

        # Save best config for this fold to a file
        with open(
            os.path.join(
                output_directory,
                f"greedy_best_config_seed_{SEED}_run_{run}_fold_{fold}.pkl",
            ),
            "wb",
        ) as f:
            pickle.dump(best_config, f)

        # Evaluate on test fold
        (
            correct,
            equal,
            incomparable,
            total,
            accuracy,
            weighted_accuracy,
            reasons,
            predictions,
        ) = evaluate_rulebook_with_cache(
            rulebook, X_test, y_test, votes_test, cache_dict, trajectories_dict
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


print("Restoring default rulebook parameters...")
for rule_id, params in default_params.items():
    rule_id_to_rule[rule_id].parameters.update(params)

print("Rule group optimization")
print("----------------------")
greedy_rulebooks, num_unique_rulebooks, correct, accuracy, scenario_to_samples = (
    find_scenario_groups(
        rulebook,
        X,
        y,
        y_votes,
        cache_dict,
        trajectories_dict,
        greedy_group_optimization,
        groups,
        max_iters=1,
        restricted=False,
        fixed_level_depth=0,
    )
)
# print results
print("Greedy Group Optimization (max_iters=1):")
print(f"Number of unique rulebooks found: {num_unique_rulebooks}")
print(f"Correct classifications: {correct} out of {len(y)}")
print(f"Accuracy: {accuracy:.4f}")
print()

greedy_rulebooks, num_unique_rulebooks, correct, accuracy, scenario_to_samples = (
    find_scenario_groups(
        rulebook,
        X,
        y,
        y_votes,
        cache_dict,
        trajectories_dict,
        greedy_group_optimization,
        groups,
        max_iters=2,
        restricted=False,
        fixed_level_depth=0,
    )
)
# print results
print("Greedy Group Optimization (max_iters=2):")
print(f"Number of unique rulebooks found: {num_unique_rulebooks}")
print(f"Correct classifications: {correct} out of {len(y)}")
print(f"Accuracy: {accuracy:.4f}")
print()

brute_force_rulebooks, num_unique_rulebooks, correct, accuracy, scenario_to_samples = (
    find_scenario_groups(
        rulebook,
        X,
        y,
        y_votes,
        cache_dict,
        trajectories_dict,
        brute_force_group_optimization,
        groups,
        fixed_level_depth=0,
    )
)
# print results
print("Brute Force Group Optimization:")
print(f"Number of unique rulebooks found: {num_unique_rulebooks}")
print(f"Correct classifications: {correct} out of {len(y)}")
print(f"Accuracy: {accuracy:.4f}")


print("\n")
print("Alternative rule definition comparison:")
print("----------------------")


from rulebook_benchmark.rule_functions import (f7_alt, f11_sum, f11_v, f12_sum,
                                               f12_v, f13_sum, f13_v)

rb = Rulebook(
    rule_file="reasonable_crowd_rule_functions.py",
    rulebook_file="reasonable_crowd_5.graph",
)
rule_id_to_rule = {
    1: f1,
    2: f2,
    3: f3,
    4: f4,
    5: f5,
    6: f6,
    7: f7,
    8: f8,
    9: f9,
    11: f11,
    12: f12,
    13: f13,
    15: f15,
    17: f17,
    18: f18,
}
rule_id_to_rule_alt = {
    1: f1,
    2: f2,
    3: f3,
    4: f4,
    5: f5,
    6: f6,
    7: f7,
    8: f8,
    9: f9,
    11: f11_v,
    12: f12_v,
    13: f13_v,
    15: f15,
    17: f17,
    18: f18,
}
rule_id_to_rule_side = {
    1: f1,
    2: f2,
    3: f3,
    4: f4,
    5: f5,
    6: f6,
    7: f7_alt,
    8: f8,
    9: f9,
    11: f11,
    12: f12,
    13: f13,
    15: f15,
    17: f17,
    18: f18,
}
rule_id_to_rule_sum = {
    1: f1,
    2: f2,
    3: f3,
    4: f4,
    5: f5,
    6: f6,
    7: f7,
    8: f8,
    9: f9,
    11: f11_sum,
    12: f12_sum,
    13: f13_sum,
    15: f15,
    17: f17,
    18: f18,
}

rulebook = InPlaceRulebook(rb.priority_graph, rule_id_to_rule)
rulebook_alt = InPlaceRulebook(rb.priority_graph, rule_id_to_rule_alt)
rulebook_side = InPlaceRulebook(rb.priority_graph, rule_id_to_rule_side)
rulebook_sum = InPlaceRulebook(rb.priority_graph, rule_id_to_rule_sum)
rule_id_to_params = {
    4: ["threshold"],
    6: ["threshold"],
    8: ["threshold"],
    9: ["threshold"],
    5: ["velocity", "threshold", "timesteps"],
    11: ["threshold"],
    12: ["threshold"],
    13: ["threshold"],
    18: ["buffer"],
}
rule_id_to_values = {
    4: {"threshold": [0.6, 0.8, 1, 1.2]},
    6: {"threshold": [0.6, 0.8, 1, 1.2]},
    8: {"threshold": [0.5, 1, 1.5, 2]},
    9: {"threshold": [0.5, 1, 1.5, 2]},
    5: {"velocity": [4], "threshold": [-1.5, -1, -0.5], "timesteps": [30]},
    11: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    12: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    13: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    18: {"buffer": [0.3, 0.5, 0.8]},
}


rule_id_to_params_alt = {
    4: ["threshold"],
    6: ["threshold"],
    8: ["threshold"],
    9: ["threshold"],
    5: ["velocity", "threshold", "timesteps"],
    11: ["threshold"],
    12: ["threshold"],
    13: ["threshold"],
    18: ["buffer"],
}
rule_id_to_values_alt = {
    4: {"threshold": [0.6, 0.8, 1, 1.2]},
    6: {"threshold": [0.6, 0.8, 1, 1.2]},
    8: {"threshold": [0.5, 1, 1.5, 2]},
    9: {"threshold": [0.5, 1, 1.5, 2]},
    5: {"velocity": [4], "threshold": [-1.5, -1, -0.5], "timesteps": [30]},
    11: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    12: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    13: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    18: {"buffer": [0.3, 0.5, 0.8]},
}

rule_id_to_params_side = {
    4: ["threshold"],
    6: ["threshold"],
    8: ["threshold"],
    9: ["threshold"],
    5: ["velocity", "threshold", "timesteps"],
    11: ["threshold"],
    12: ["threshold"],
    13: ["threshold"],
    18: ["buffer"],
    7: ["fine_grained"],
}
rule_id_to_values_side = {
    4: {"threshold": [0.6, 0.8, 1, 1.2]},
    6: {"threshold": [0.6, 0.8, 1, 1.2]},
    8: {"threshold": [0.5, 1, 1.5, 2]},
    9: {"threshold": [0.5, 1, 1.5, 2]},
    5: {"velocity": [4], "threshold": [-1.5, -1, -0.5], "timesteps": [30]},
    11: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    12: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    13: {"threshold": [0.4, 0.8, 1.2, 1.6]},
    18: {"buffer": [0.3, 0.5, 0.8]},
    7: {"fine_grained": [True, False]},
}


if os.path.exists(os.path.join(output_directory, "tuning_cache.pkl")):
    print("Loading cached rule evaluations...")
    with open(os.path.join(output_directory, "tuning_cache.pkl"), "rb") as f:
        cache_dict = pickle.load(f)
else:
    print("No cached rule evaluations found. Starting with empty cache.")
    cache_dict = {}
    cache_rule_evaluations(
        rulebook,
        rule_id_to_params,
        rule_id_to_values,
        X,
        y,
        cache_dict,
        trajectories_dict,
    )
    pickle.dump(
        cache_dict, open(os.path.join(output_directory, "tuning_cache.pkl"), "wb")
    )

alt_cache_dict = {}
sum_cache_dict = {}
side_cache_dict = {}

# copy tuning cache into alt tuning cache except for rules 11 12 13
for rule_id in cache_dict:
    if rule_id in [11, 12, 13]:
        continue
    alt_cache_dict[rule_id] = cache_dict[rule_id]
    sum_cache_dict[rule_id] = cache_dict[rule_id]

for rule_id in cache_dict:
    if rule_id == 7:
        continue
    side_cache_dict[rule_id] = cache_dict[rule_id]

if os.path.exists(os.path.join(output_directory, "alt_tuning_cache.pkl")):
    print("Loading cached rule evaluations...")
    with open(os.path.join(output_directory, "alt_tuning_cache.pkl"), "rb") as f:
        alt_cache_dict = pickle.load(f)
else:
    print("No cached rule evaluations found. Starting with empty cache.")
    cache_rule_evaluations(
        rulebook_alt,
        rule_id_to_params_alt,
        rule_id_to_values_alt,
        X,
        y,
        alt_cache_dict,
        trajectories_dict,
    )
    pickle.dump(
        alt_cache_dict,
        open(os.path.join(output_directory, "alt_tuning_cache.pkl"), "wb"),
    )


if os.path.exists(os.path.join(output_directory, "side_tuning_cache.pkl")):
    print("Loading cached rule evaluations...")
    with open(os.path.join(output_directory, "side_tuning_cache.pkl"), "rb") as f:
        side_cache_dict = pickle.load(f)
else:
    print("No cached rule evaluations found. Starting with empty cache.")
    cache_rule_evaluations(
        rulebook_side,
        rule_id_to_params_side,
        rule_id_to_values_side,
        X,
        y,
        side_cache_dict,
        trajectories_dict,
    )
    pickle.dump(
        side_cache_dict,
        open(os.path.join(output_directory, "side_tuning_cache.pkl"), "wb"),
    )


if os.path.exists(os.path.join(output_directory, "sum_tuning_cache.pkl")):
    print("Loading cached rule evaluations...")
    with open(os.path.join(output_directory, "sum_tuning_cache.pkl"), "rb") as f:
        sum_cache_dict = pickle.load(f)
else:
    print("No cached rule evaluations found. Starting with empty cache.")
    cache_rule_evaluations(
        rulebook_sum,
        rule_id_to_params,
        rule_id_to_values,
        X,
        y,
        sum_cache_dict,
        trajectories_dict,
    )
    pickle.dump(
        sum_cache_dict,
        open(os.path.join(output_directory, "sum_tuning_cache.pkl"), "wb"),
    )

groups = [[1, 2], [3, 7], [8, 9, 11, 12, 13], [17, 18, 15], [4, 6, 5]]
name_to_group = {
    "safety-critical": groups[0],
    "operation-limit": groups[1],
    "safety-enhancing": groups[2],
    "predictability": groups[3],
    "precautionary": groups[4],
}
group_to_name = {tuple(value): key for key, value in name_to_group.items()}
rulebook = group_rulebook(rulebook, groups, keep_relations=True)


base_result = evaluate_rulebook_with_cache(
    rulebook, X, y, y_votes, cache_dict, trajectories_dict
)
print("Base Rulebook Results:")
print("Correct:", base_result[0])
print("Equal:", base_result[1])
print("Incomparable:", base_result[2])
print("Total:", base_result[3])
print("Accuracy:", base_result[4])
print("Weighted Accuracy:", base_result[5])
print(
    "Accuracy out of predictions:",
    (
        base_result[0] / (base_result[3] - base_result[2])
        if base_result[3] - base_result[2] > 0
        else 0.0
    ),
)


base_result_alt = evaluate_rulebook_with_cache(
    rulebook_alt, X, y, y_votes, alt_cache_dict, trajectories_dict
)

print("Alternative Rulebook Results with front_angle=90")
print("Correct:", base_result_alt[0])
print("Equal:", base_result_alt[1])
print("Incomparable:", base_result_alt[2])
print("Total:", base_result_alt[3])
print("Accuracy:", base_result_alt[4])
print("Weighted Accuracy:", base_result_alt[5])
print(
    "Accuracy out of predictions:",
    (
        base_result_alt[0] / (base_result_alt[3] - base_result_alt[2])
        if base_result_alt[3] - base_result_alt[2] > 0
        else 0.0
    ),
)

base_result_sum = evaluate_rulebook_with_cache(
    rulebook_sum, X, y, y_votes, sum_cache_dict, trajectories_dict
)

print("Sum Rulebook Results")
print("Correct:", base_result_sum[0])
print("Equal:", base_result_sum[1])
print("Incomparable:", base_result_sum[2])
print("Total:", base_result_sum[3])
print("Accuracy:", base_result_sum[4])
print("Weighted Accuracy:", base_result_sum[5])
print(
    "Accuracy out of predictions:",
    (
        base_result_sum[0] / (base_result_sum[3] - base_result_sum[2])
        if base_result_sum[3] - base_result_sum[2] > 0
        else 0.0
    ),
)


base_result_side = evaluate_rulebook_with_cache(
    rulebook_side, X, y, y_votes, side_cache_dict, trajectories_dict
)


print("Side Rulebook Results")
print("Correct:", base_result_side[0])
print("Equal:", base_result_side[1])
print("Incomparable:", base_result_side[2])
print("Total:", base_result_side[3])
print("Accuracy:", base_result_side[4])
print("Weighted Accuracy:", base_result_side[5])
print(
    "Accuracy out of predictions:",
    (
        base_result_side[0] / (base_result_side[3] - base_result_side[2])
        if base_result_side[3] - base_result_side[2] > 0
        else 0.0
    ),
)

f7_alt.parameters["fine_grained"] = False

base_result_side_not_fg = evaluate_rulebook_with_cache(
    rulebook_side, X, y, y_votes, side_cache_dict, trajectories_dict
)

print("Side Rulebook Results with fine_grained=False")
print("Correct:", base_result_side_not_fg[0])
print("Equal:", base_result_side_not_fg[1])
print("Incomparable:", base_result_side_not_fg[2])
print("Total:", base_result_side_not_fg[3])
print("Accuracy:", base_result_side_not_fg[4])
print("Weighted Accuracy:", base_result_side_not_fg[5])
print(
    "Accuracy out of predictions:",
    (
        base_result_side_not_fg[0]
        / (base_result_side_not_fg[3] - base_result_side_not_fg[2])
        if base_result_side_not_fg[3] - base_result_side_not_fg[2] > 0
        else 0.0
    ),
)


def compare_preds(name, base_res, other_res):
    base_preds = base_res[7]
    other_preds = other_res[7]
    base_reasons = base_res[6]
    other_reasons = other_res[6]

    n = min(len(base_preds), len(other_preds))
    diffs_preds = [i for i in range(n) if base_preds[i] != other_preds[i]]
    diffs_reasons = [i for i in range(n) if base_reasons[i] != other_reasons[i]]
    diffs_reasons_only = [i for i in diffs_reasons if i not in diffs_preds]

    print(f"\n=== base vs {name} ===")
    print(f"Compared examples: {n}")
    print(
        f"Prediction changes: {len(diffs_preds)} ({(len(diffs_preds)/n*100) if n>0 else 0:.2f}%)"
    )
    print(
        f"Reason changes (any): {len(diffs_reasons)} ({(len(diffs_reasons)/n*100) if n>0 else 0:.2f}%)"
    )
    print(
        f"Reason changes with same prediction: {len(diffs_reasons_only)} ({(len(diffs_reasons_only)/n*100) if n>0 else 0:.2f}%)"
    )

    def reason_counters(indices):
        base_cnt = Counter()
        other_cnt = Counter()
        for i in indices:
            rb = base_reasons[i]
            ro = other_reasons[i]
            if isinstance(rb, (list, tuple)):
                base_cnt.update(rb)
            else:
                base_cnt.update([rb])
            if isinstance(ro, (list, tuple)):
                other_cnt.update(ro)
            else:
                other_cnt.update([ro])
        return base_cnt, other_cnt

    if diffs_preds:
        print("\n-- For examples where prediction changed --")
        base_reason_counts, other_reason_counts = reason_counters(diffs_preds)
        print("Counts of reasons in base (for differing predictions):")
        for reason, cnt in base_reason_counts.most_common():
            print(f"  {reason}: {cnt}")
        print("Counts of reasons in other (for differing predictions):")
        for reason, cnt in other_reason_counts.most_common():
            print(f"  {reason}: {cnt}")

    if diffs_reasons_only:
        print(
            "\n-- For examples where prediction stayed the same but reasons changed --"
        )
        base_reason_counts, other_reason_counts = reason_counters(diffs_reasons_only)
        print("Counts of reasons in base (for same-prediction differing reasons):")
        for reason, cnt in base_reason_counts.most_common():
            print(f"  {reason}: {cnt}")
        print("Counts of reasons in other (for same-prediction differing reasons):")
        for reason, cnt in other_reason_counts.most_common():
            print(f"  {reason}: {cnt}")


# Run comparisons
compare_preds("alternative", base_result, base_result_alt)
compare_preds("sum", base_result, base_result_sum)
compare_preds("side", base_result, base_result_side)
compare_preds("side_not_fine_grained", base_result, base_result_side_not_fg)
