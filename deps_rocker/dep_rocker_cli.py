# from deps_rocker.dependencies import Dependencies
import os


def launch_vscode():
    launch_vscode_cmd = """
set -e

export COMPOSE_DOCKER_CLI_BUILD=1
export DOCKER_BUILDKIT=1

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" #gets the current directory

#move to the project root folder
cd "$SCRIPT_DIR"/..

CONTAINER_NAME="$(basename "$(pwd)")-$(git rev-parse --abbrev-ref HEAD)"

echo "stopping existing container" "$CONTAINER_NAME" 
docker stop "$CONTAINER_NAME" || true 

CONTAINER_HEX=$(printf $CONTAINER_NAME | xxd -p | tr '\n' ' ' | sed 's/\\s//g' | tr -d ' ');

rocker --nvidia --x11 --user --pull --git --image-name "$CONTAINER_NAME" --name "$CONTAINER_NAME" --volume "${PWD}":/workspaces/"${CONTAINER_NAME}":Z --deps --oyr-run-arg " --detach" ubuntu:22.04

#this follows the same convention as if it were opened by a vscode devcontainer
code --folder-uri vscode-remote://attached-container+"$CONTAINER_HEX"/workspaces/"${CONTAINER_NAME}"
"""
    os.system(launch_vscode_cmd)
