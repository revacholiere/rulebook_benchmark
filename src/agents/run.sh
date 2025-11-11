#!/bin/bash

# Using a list of scenarios
scenarios_without_vru=(
    "../../scenarios/crash/crash_apple_082321.scenic"
    "../../scenarios/crash/crash_apple_120621-pdf_gemini.scenic"
    "../../scenarios/crash/crash_apple_10232023.scenic"
    "../../scenarios/crash/crash_collision-report-zoox-november-22-2019-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_030623-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_031823-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_032223_2-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_032721-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_07212023-a-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_07302023-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_08082023-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_ol316_042522-pdf_gemini.scenic"
    "../../scenarios/crash/crash_gm-cruise-collision-report-april-10-2019-2-pdf_gemini.scenic"
    "../../scenarios/crash/crash_lyft_041421-pdf_gemini.scenic"
    "../../scenarios/crash/crash_lyft-collision-report-april-11-2019-pdf_gemini.scenic"
    "../../scenarios/crash/crash_waymo_010122-pdf_gemini.scenic"
    "../../scenarios/crash/crash_waymo_021721-pdf_gemini.scenic"
    "../../scenarios/crash/crash_waymo_040122-pdf_gemini.scenic"
    "../../scenarios/crash/crash_waymo_august_9_2019_1.scenic"
    "../../scenarios/crash/crash_waymo_august_12_2019.scenic"
    "../../scenarios/crash/crash_zoox_05112021-pdf_gemini.scenic"
)
scenarios_with_vru=(
    "../../scenarios/crash/crash_collision-report-zoox-june-11-2019-pdf_gemini.scenic"
    "../../scenarios/crash/crash_collision-report-zoox-june-19-2019-pdf_gemini.scenic"
    "../../scenarios/crash/crash_cruise_12112021-pdf_gemini.scenic"
    "../../scenarios/crash/crash_gm-cruise-collision-report-august-10-2019-pdf_gemini.scenic"
    "../../scenarios/crash/crash_waymo_022822_2-pdf_gemini.scenic"
    "../../scenarios/crash/crash_waymo_082121-pdf_gemini.scenic"
)
log_file="./outputs/collision_with_vru_ppo_eval.log"
for scenario in "${scenarios_with_vru[@]}"; do
    echo "Running scenario: ${scenario}"

    timeout --preserve-status --signal SIGINT 1200s \
    python run_evaluation.py \
        --config-name=eval.yaml \
        hydra.job.chdir=False \
        hydra.output_subdir=null \
        scenic.file_path="${scenario}" \
        hydra.job_logging.handlers.file.filename="${log_file}"

    exit_code=$?
    if [ $exit_code -eq 124 ]; then
        echo "⚠️ Scenario ${scenario} timed out after 1200s and was terminated." | tee -a "$log_file"
    elif [ $exit_code -ne 0 ]; then
        echo "❌ Scenario ${scenario} exited with error code ${exit_code}." | tee -a "$log_file"
    else
        echo "✅ Scenario ${scenario} completed successfully." | tee -a "$log_file"
    fi
    echo "---------------------------------------------" | tee -a "$log_file"
done

# Using an array of indices for representative scenarios
#indices=(
#    5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79 80 81 82 83 84 85 86 87 88 89 90 91 92 93 94 95 96 97 98 99 100
#)
## built-in 20 missing: 12, 28, 48, 61, 67, 71, 93
## ppo 20 missing: 4, 28, 41, 48, 51, 61, 67, 71, 75, 93
#log_file="./outputs/representative30_eval.log"
#base_path="../../scenarios/basic_gen"
#prefix="representative30_"
#extension=".scenic"
#
#for idx in "${indices[@]}"; do
#    scenario="${base_path}/${prefix}${idx}${extension}"
#    echo "Running scenario: ${scenario}"
#
#    timeout --preserve-status --signal SIGINT 1500s \
#    python run_evaluation.py \
#        --config-name=eval.yaml \
#        hydra.job.chdir=False \
#        hydra.output_subdir=null \
#        scenic.file_path="${scenario}" \
#        hydra.job_logging.handlers.file.filename="${log_file}"
#
#    exit_code=$?
#    if [ $exit_code -eq 124 ]; then
#        echo "⚠️ Scenario ${scenario} timed out after 1200s and was terminated." | tee -a "$log_file"
#    elif [ $exit_code -ne 0 ]; then
#        echo "❌ Scenario ${scenario} exited with error code ${exit_code}." | tee -a "$log_file"
#    else
#        echo "✅ Scenario ${scenario} completed successfully." | tee -a "$log_file"
#    fi
#    echo "---------------------------------------------" | tee -a "$log_file"
#done
