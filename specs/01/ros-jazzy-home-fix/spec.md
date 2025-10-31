# ROS Jazzy $HOME Variable Fix

## Problem
The `ros_jazzy_user_snippet.Dockerfile` uses `$HOME` environment variable in ENV statements, but `$HOME` is not available during Docker build time when the user snippet runs. This causes permission errors when trying to create directories like `/ros_ws` instead of the intended `~/ros_ws`.

## Root Cause
The user snippet runs before the USER directive is processed, so `$HOME` is not set. Other extensions solve this by using `cliargs.get("user_home_dir")` in Python and passing the actual path to the Dockerfile template.

## Solution
1. Update the ROS Jazzy extension to get the container home directory from `cliargs.get("user_home_dir")`
2. Pass this path to the user snippet template via `empy_user_args`
3. Replace `$HOME` references in the user snippet with the actual path

## Expected Outcome
The ROS workspace directories will be created in the correct user home directory (e.g., `/home/user/ros_ws`) instead of failing to create `/ros_ws` due to permission errors.