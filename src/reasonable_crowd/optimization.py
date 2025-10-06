from reasonable_crowd.evaluation import evaluate_rule_with_cache, evaluate_rulebook_with_cache
import itertools
import networkx as nx
from tqdm import tqdm
import random
import math


def cache_rule_evaluations(
    rbook,
    rule_id_to_params,
    rule_id_to_values,
    X,
    y,
    rule_parameter_result_dict,
    trajectories_dict,
    verbose=False
):

    priority_order = list(nx.topological_sort(rbook.in_place_priority_graph))
    rule_id_to_rule = rbook.rule_id_to_rule
    pbar = tqdm(total=len(priority_order), desc="Caching rule evaluations", leave=False)

    for i, rule_id in enumerate(priority_order):

        if verbose:
            print(f"\n[optimize_rules_grid] Evaluating {rule_id} (id={rule_id})")

        param_names = rule_id_to_params.get(rule_id, [])
        value_lists = rule_id_to_values.get(rule_id, {})

        # Case 1: rule has tunable params → grid search
        if param_names and value_lists:
            for values in itertools.product(*(value_lists[p] for p in param_names)):
                trial_params = dict(zip(param_names, values))
                rule = rule_id_to_rule[rule_id]
                rule.parameters.update(trial_params)

                correct = evaluate_rule_with_cache(rule, X, y, rule_parameter_result_dict, rule_id, trajectories_dict)
                

        # Case 2: no tunable params → just evaluate
        else:
            rule = rule_id_to_rule[rule_id]
            correct = evaluate_rule_with_cache(rule, X, y, rule_parameter_result_dict, rule_id, trajectories_dict)

        pbar.update(1)

    pbar.close()


def optimize_rulebook_grid_bruteforce(rulebook, dataset, labels, votes, rule_id_to_params, rule_id_to_values, trajectories_dict, rule_parameter_result_dict=None, verbose=0):
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

    iterator = tqdm(iterator, total=total_combos, desc="Grid Search", leave=False, miniters=1000, mininterval=10)

    # Try all combinations
    for combo in iterator:
        trial_config = {}
        for (rule_id, p), v in zip(param_keys, combo):
            trial_config.setdefault(rule_id, {})[p] = v

        # Apply this configuration to the rulebook
        for rule_id, params in trial_config.items():
            current_rule = graph.nodes[rule_id]['rule']
            current_rule.parameters.update(params)

        # time the evaluation
        score, incorrect, equal, incomparable, total = evaluate_rulebook_with_cache(rulebook, dataset, labels, votes, rule_parameter_result_dict, trajectories_dict)


        if verbose >= 2:
            print(f"  Trial {trial_config} -> {score:.6f}")

        if score > best_score:
            best_score = score
            best_config = trial_config


    #if verbose:
    #    print("\n[optimize_rulebook_grid_bruteforce] Finished.")
    #    print(f"Best score: {best_score:.6f}")
    #    print("Best config:")
    #    for rid, params in best_config.items():
    #        print(f"  Rule {rid} -> {params}")

    return best_config, best_score






def optimize_rulebook_grid_bruteforce_with_validation(rulebook, training_data, training_labels, training_votes, validation_data, validation_labels, validation_votes, rule_id_to_params, rule_id_to_values, trajectories_dict, rule_parameter_result_dict=None, verbose=0):

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
    best_val_score = 0
    #if verbose:
    #    print(f"  Initial score = {best_score:.6f}")

    # Progress bar around the Cartesian product
    iterator = itertools.product(*search_space)

    iterator = tqdm(iterator, total=total_combos, desc="Grid Search", leave=False)

    # Try all combinations
    for combo in iterator:
        trial_config = {}
        for (rule_id, p), v in zip(param_keys, combo):
            trial_config.setdefault(rule_id, {})[p] = v

        # Apply this configuration to the rulebook
        for rule_id, params in trial_config.items():
            current_rule = graph.nodes[rule_id]['rule']
            current_rule.parameters.update(params)


        score = evaluate_rulebook_with_cache(rulebook, training_data, training_labels, training_votes, rule_parameter_result_dict, trajectories_dict)[0]
        val_score = evaluate_rulebook_with_cache(rulebook, validation_data, validation_labels, validation_votes, rule_parameter_result_dict, trajectories_dict)[0]


        if verbose >= 2:
            print(f"  Trial {trial_config} -> {score:.6f}")

        if score > best_score and val_score > best_val_score:
            best_score = score
            best_val_score = val_score
            best_config = trial_config
            iterator.set_description(f"New best: Train {best_score:.4f}, Val {best_val_score:.4f}")

    return best_config, best_score, best_val_score



def is_acyclic(graph):
    return nx.is_directed_acyclic_graph(graph)

def is_weakly_connected(graph):
    return nx.is_weakly_connected(graph)

def random_action(rulebook, max_attempts=10):
    """
    Perform a random modification on the rulebook's in_place_priority_graph.
    The action space includes adding, removing, or swapping edges.
    Returns a new rulebook with an acyclic in_place_priority_graph.
    If no valid action is found after max_attempts, returns the original rulebook.
    """
    for _ in range(max_attempts):
        new_rulebook = rulebook.copy()
        g = new_rulebook.in_place_priority_graph
        nodes = list(g.nodes)
        edges = list(g.edges)

        action_type = random.choice(["add", "remove", "swap"])

        if action_type == "add":
            u, v = random.sample(nodes, 2)
            if not g.has_edge(u, v):
                g.add_edge(u, v)
            else:
                continue  # resample if edge already exists

        elif action_type == "remove":
            if edges:
                u, v = random.choice(edges)
                g.remove_edge(u, v)
            else:
                continue  # resample if no edges to remove

        elif action_type == "swap":
            if len(nodes) >= 2:
                u, v = random.sample(nodes, 2)
                preds_u, succs_u = list(g.predecessors(u)), list(g.successors(u))
                preds_v, succs_v = list(g.predecessors(v)), list(g.successors(v))

                g.remove_node(u)
                g.remove_node(v)
                g.add_node(u)
                g.add_node(v)

                for p in preds_v:
                    if p != u:
                        g.add_edge(p, u)
                for s in succs_v:
                    if s != u:
                        g.add_edge(u, s)

                for p in preds_u:
                    if p != v:
                        g.add_edge(p, v)
                for s in succs_u:
                    if s != v:
                        g.add_edge(v, s)
            else:
                continue  # resample if not enough nodes

        # Validate acyclicity
        if is_acyclic(g) and is_weakly_connected(g):
            new_rulebook.in_place_priority_graph = g
            return new_rulebook

    # If no valid action found after max_attempts, return original rulebook
    return rulebook



def simulated_annealing(rulebook, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict, max_iter=1000, start_temp=10.0, alpha=0.995, seed=None):
    if seed is not None:
        random.seed(seed)
    
    current_rulebook = rulebook.copy()
    current_score = evaluate_rulebook_with_cache(current_rulebook, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict)[0]

    best = current_rulebook.copy()
    best_score = current_score
    print("Initial Score:", best_score)

    T = start_temp

    pbar = tqdm(total=max_iter, desc="Simulated Annealing", leave=False)

    for i in range(max_iter):

        candidate = random_action(current_rulebook)

        candidate_score = evaluate_rulebook_with_cache(candidate, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict)[0]

        delta = candidate_score - current_score

        if delta > 0 or random.random() < math.exp(min(0, max(-delta / T, -700))):
            current_rulebook = candidate
            current_score = candidate_score
            pbar.set_description(f"Current: {current_score:.4f}")

            if current_score > best_score:
                best = current_rulebook.copy()
                best_score = current_score
                pbar.set_description(f"New best: {best_score:.4f}")

        T *= alpha
        pbar.update(1)

    pbar.close()
    return best, best_score






def simulated_annealing_with_validation(rulebook, train_data, train_labels, train_votes, val_data, val_labels, val_votes, rule_parameter_result_dict, trajectories_dict, max_iter=1000, start_temp=10.0, alpha=0.995, seed=None):
    if seed is not None:
        random.seed(seed)
    
    current_rulebook = rulebook
    current_score = evaluate_rulebook_with_cache(current_rulebook, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict)[0]
    current_val_score = evaluate_rulebook_with_cache(current_rulebook, val_data, val_labels, val_votes, rule_parameter_result_dict, trajectories_dict)[0]
    
    best = current_rulebook
    best_score = current_score
    best_val_score = current_val_score

    T = start_temp

    pbar = tqdm(total=max_iter, desc="Simulated Annealing", leave=False)

    for i in range(max_iter):
        candidate = random_action(current_rulebook)
        candidate_score = evaluate_rulebook_with_cache(candidate, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict)[0]
        candidate_val_score = evaluate_rulebook_with_cache(candidate, val_data, val_labels, val_votes, rule_parameter_result_dict, trajectories_dict)[0]

        delta = candidate_score - current_score
        delta_val = candidate_val_score - current_val_score
        if (delta > 0 and delta_val > 0) or (random.random() < math.exp(min(0, max(-delta / T, -700))) and random.random() < math.exp(min(0, max(-delta_val / T, -700)))):
            current_rulebook = candidate
            current_score = candidate_score
            current_val_score = candidate_val_score

            if current_score > best_score and current_val_score > best_val_score:
                best = current_rulebook.copy()
                best_score = current_score
                best_val_score = current_val_score
                pbar.set_description(f"New best: Train {best_score:.4f}, Val {best_val_score:.4f}")
        pbar.update(1)
        T *= alpha

    pbar.close()
    return best, best_score, best_val_score





def simulated_annealing_single_sample(rulebook, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict, max_iter=1000, start_temp=10.0, alpha=0.995, seed=None):
    if seed is not None:
        random.seed(seed)

    
    current_rulebook = rulebook.copy()
    current_score = evaluate_rulebook_with_cache(current_rulebook, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict)[0]

    best = current_rulebook.copy()
    best_score = current_score
    if best_score > 0:
        return best, best_score
    

    T = start_temp

    pbar = tqdm(total=max_iter, desc="Simulated Annealing", leave=False)

    for i in range(max_iter):

        candidate = random_action(current_rulebook)

        candidate_score = evaluate_rulebook_with_cache(candidate, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict)[0]

        delta = candidate_score - current_score

        if delta > 0 or random.random() < math.exp(min(0, max(-delta / T, -700))):
            current_rulebook = candidate
            current_score = candidate_score
            pbar.set_description(f"Current: {current_score:.4f}")

            if current_score > best_score:
                best = current_rulebook.copy()
                best_score = current_score
                pbar.set_description(f"New best: {best_score:.4f}")
                return best, best_score

        T *= alpha
        pbar.update(1)

    pbar.close()
    return best, best_score


def number_of_unique_rulebooks(rulebook, train_data, train_labels, train_votes, rule_parameter_result_dict, trajectories_dict, seed=None):

    
    score = 0
    total = len(train_data)
    unique_rulebooks = []
    pbar = tqdm(total=total, desc="Finding Unique Rulebooks", leave=False)
    unsatisfiable_samples = []
    
    for i in range(total):
        sample = train_data[i:i+1]
        label = train_labels[i:i+1]
        votes = train_votes[i:i+1]
        
        #assert sc > 0
        rb, sc = simulated_annealing_single_sample(rulebook, sample, label, votes, rule_parameter_result_dict, trajectories_dict, max_iter=1000, start_temp=15.0, alpha=0.995, seed=seed)

    
        found = False
        if sc > 0:
            for existing_rb in unique_rulebooks:
                if nx.utils.graphs_equal(existing_rb.in_place_priority_graph, rb.in_place_priority_graph):

                    found = True
                    break

            if not found:
                unique_rulebooks.append(rb)
        else:
            unsatisfiable_samples.append((sample[0], votes[0]))
        
        score += sc
        pbar.update(1)
    pbar.close()
        
    return len(unique_rulebooks), score, score/total, unsatisfiable_samples