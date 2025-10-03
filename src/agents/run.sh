#!/bin/bash

# List of scenarios
scenarios=(
    "../../scenarios/nhtsa_intersection01/nhtsa_intersection01.scenic"
    "../../scenarios/nhtsa_intersection01/nhtsa_intersection01_metadriveppo.scenic"
    "../../scenarios/crash_waymo_august_9_2019_1/crash_waymo_august_9_2019_1.scenic"
    "../../scenarios/crash_waymo_august_9_2019_1/crash_waymo_august_9_2019_1_metadriveppo.scenic"
    "../../scenarios/crash_waymo_august_12_2019/crash_waymo_august_12_2019.scenic"
    "../../scenarios/crash_waymo_august_12_2019/crash_waymo_august_12_2019_metadriveppo.scenic"
)

for scenario in "${scenarios[@]}"; do
    python run_evaluation.py \
        --config-name=eval.yaml \
        hydra.job.chdir=False \
        hydra.output_subdir=null \
        scenic.file_path="$scenario"
done

#python run_evaluation.py hydra.job.chdir=False hydra.output_subdir=null --config-name=eval.yaml