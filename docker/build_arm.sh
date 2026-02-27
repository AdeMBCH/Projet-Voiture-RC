#!/bin/bash

# Get the absolute path of the current script
CURRENT_DIR_PATH="$(dirname "$(realpath "$0")")/../"

mkdir -p "$CURRENT_DIR_PATH/.arm64/"

# Build the software in the arm64 container
docker run --platform linux/arm64 \
  -v $CURRENT_DIR_PATH:/home \
  -v $CURRENT_DIR_PATH/.arm64:/voiture2A-ros \
  --rm thomaslemezo/ros-arm64-robot \
  bash -c "/home/docker/run_docker_build.sh"

# Go to the directory
cd ../.arm64
