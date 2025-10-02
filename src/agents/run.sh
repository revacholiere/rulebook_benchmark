#!/bin/bash

# List of scenarios
scenarios=(
    "../../scenarios/nhtsa_intersection01/nhtsa_intersection01.scenic"
    "../../scenarios/nhtsa_intersection01/nhtsa_intersection01_metadriveppo.scenic"
)

for scenario in "${scenarios[@]}"; do
    python run_evaluation.py \
        --config-name=eval.yaml \
        hydra.job.chdir=False \
        hydra.output_subdir=null \
        scenic.file_path="$scenario"
done

#python run_evaluation.py hydra.job.chdir=False hydra.output_subdir=null --config-name=eval.yaml