#!/bin/bash

# List of scenarios
scenario_folder="../../scenarios"
scenarios=(
    "crash/crash_waymo_august_9_2019_1.scenic"
    "crash/crash_waymo_august_12_2019.scenic"
    crash/crash_apple_10232023.scenic
    crash/crash_apple_082321.scenic
    "nhtsa/nhtsa_intersection01.scenic"
    "nhtsa/nhtsa_bypassing01.scenic"
)

for scenario in "${scenarios[@]}"; do
    python run_evaluation.py \
        --config-name=eval.yaml \
        hydra.job.chdir=False \
        hydra.output_subdir=null \
        scenic.file_path="${scenario_folder}/${scenario}"
done

#python run_evaluation.py hydra.job.chdir=False hydra.output_subdir=null --config-name=eval.yaml