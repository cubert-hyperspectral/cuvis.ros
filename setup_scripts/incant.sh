#!/bin/bash
docker run --ipc=host --network host --privileged  --cap-add=NET_ADMIN -it --rm --workdir="/colcon_ws/src" -i cuvis_foxy