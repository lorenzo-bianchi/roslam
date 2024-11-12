#!/usr/bin/env bash

# Script to automate continuous sampling for CDC23.
#
# Roberto Masocco <robmasocco@gmail.com>
#
# March 13, 2023

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then set -o xtrace; fi

function usage {
  echo >&2 "Usage:"
  echo >&2 "    sampling.sh TOPICS_FILE OUTPUT_DIR"
  echo >&2 "TOPICS_FILE must be the path to a text file containing full names of the topics to sample, one by line."
  echo >&2 "OUTPUT_DIR must be the name of the directory to be created in ~/logs/ where to store the sampled data."
  echo >&2 "The script will start the recording process, which can be stopped with Ctrl+C."
}

if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
  usage
  exit 0
fi

# Check input arguments
if [[ $# -ne 2 ]]; then
  usage
  exit 1
fi

# Read the input file and get the topic names to record
TOPICS_FILE="$1"
TOPICS=()
while IFS= read -r line; do
  TOPICS+=("$line")
done < "$TOPICS_FILE"

# Read back the topics to record
echo "The following topics will be recorded:"
for topic in "${TOPICS[@]}"; do
  echo "$topic"
done
printf '\n'

# Check that we have topics to record
if [[ ${#TOPICS[@]} -eq 0 ]]; then
  echo >&2 "No topics to record!"
  exit 1
fi

# Start the ros2 bag record process
ros2 bag record \
  -o "/home/neo/workspace/logs/$2" \
  --max-cache-size 6000 \
  "${TOPICS[@]}"

# Print dimension of .db3 file
BAG_SIZE=$(du -sh "/home/neo/workspace/logs/$2/$2_0.db3" | cut -f1)
echo -e "\nBag size: $BAG_SIZE"