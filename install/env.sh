#!/usr/bin/env bash

# ArduPilot tools (sim_vehicle.py, etc.)
export PATH="$PATH:$HOME/ardupilot/Tools/autotest"

# Gazebo Classic plugin path (para que encuentre libArduPilotPlugin.so)
export GAZEBO_PLUGIN_PATH="$HOME/ardupilot_gazebo/build/lib:$GAZEBO_PLUGIN_PATH"

# Model path (para model://iris_with_ardupilot, etc.)
export GAZEBO_MODEL_PATH="$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH"

# (Opcional) Si tu world/modelos están en tu repo IR2136
# export GAZEBO_MODEL_PATH="$HOME/Documentos/GitHub/IR2136/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH"
