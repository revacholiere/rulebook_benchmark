from concurrent.futures import ThreadPoolExecutor, as_completed
from rulebook_benchmark.process_trajectory import process_trajectory
from reasonable_crowd.parse_trajectory import parse_trajectory
import os
from tqdm import tqdm
import pickle
import json
from rulebook_benchmark.rulebook import Relation
import numpy as np

def _parse_single_file(filename, trajectory_directory, network_U, network_S, step_size):
    traj_path = os.path.join(trajectory_directory, filename)
    if filename.startswith('U'):
        realization = parse_trajectory(traj_path, step_size=step_size)
        realization.network = network_U
    else:
        realization = parse_trajectory(traj_path, step_size=step_size)
        realization.network = network_S
        
    process_trajectory(realization)
    return filename, realization


def load_all_trajectories(trajectory_directory, network_U, network_S, step_size=100000, max_workers=8):
    trajectory_files = [f for f in os.listdir(trajectory_directory) if f.endswith('.json')]
    trajectories = []

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(_parse_single_file, f, trajectory_directory, network_U, network_S, step_size): f
            for f in trajectory_files
        }

        with tqdm(total=len(futures), leave=False) as pbar:
            for future in as_completed(futures):
                filename, realization = future.result()
                trajectories.append((filename, realization))
                pbar.set_description(f"Parsing {filename}")
                pbar.update(1)

    return trajectories



def get_trajectories(output_directory, trajectory_directory, network_U, network_S):
    # check if trajectories.pkl exists
    if os.path.exists(os.path.join(output_directory, 'trajectories.pkl')):
        print("Loading cached trajectories...")
        with open(os.path.join(output_directory, 'trajectories.pkl'), 'rb') as f:
            trajectories = pickle.load(f)
            
    else:
        print("Parsing trajectories from JSON files...")
        trajectories = load_all_trajectories(
            trajectory_directory,
            network_U,
            network_S,
            step_size=100000,
        max_workers=8   # adjust depending on your CPU
    )
        with open(os.path.join(output_directory, 'trajectories.pkl'), 'wb') as f:
            pickle.dump(trajectories, f)
            
    return trajectories


def load_annotations(path_to_reasonable_crowd):
    annotation_file = os.path.join(path_to_reasonable_crowd, "annotations/annotations.json")
    with open(annotation_file, 'r') as f:
        data = json.load(f)
    f.close()
    return data



def build_evaluation_dataset(data):

    X = []
    y = []
    y_votes = []
    for scenario, annotations in data.items():
        #print(len(annotations.items()))
        evaluated_pairs = set()
        for pair, votes in list(annotations.items()):
            t1, t2 = pair.split(" ;; ")
            reverse_pair = f"{t2} ;; {t1}"
            reverse_votes = annotations.get(reverse_pair, [])
            if pair in evaluated_pairs or reverse_pair in evaluated_pairs:
                continue
            evaluated_pairs.add(pair)
            evaluated_pairs.add(reverse_pair)
                
            votes_1 = len(votes)
            votes_2 = len(reverse_votes)
            
            if votes_1 > votes_2:
                human_pref = Relation.LARGER
            elif votes_1 < votes_2:
                human_pref = Relation.SMALLER
            else:
                human_pref = Relation.EQUAL
                
            if human_pref == Relation.EQUAL:
                continue # skip equal votes for now
                
            X.append((t1, t2))
            y.append(human_pref)
            y_votes.append((votes_1, votes_2))

    return X, y, y_votes
