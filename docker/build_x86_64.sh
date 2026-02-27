#!/bin/bash

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja --event-handlers console_cohesion+
