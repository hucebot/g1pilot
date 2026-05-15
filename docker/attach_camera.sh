#!/usr/bin/env bash
# Attach a new bash shell to the running g1pilot_camera container.

CONTAINER=$(docker ps -qf "ancestor=g1pilot_camera:latest" | head -1)

if [ -z "$CONTAINER" ]; then
    echo "No running g1pilot_camera container found."
    exit 1
fi

docker exec -it "$CONTAINER" bash
