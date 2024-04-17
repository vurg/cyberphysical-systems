#!/bin/bash
# Pass the argument to the script
/opt/helloworld "$@"

# Keep the container alive
while true; do sleep 3600; done