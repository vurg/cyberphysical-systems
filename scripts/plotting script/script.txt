#!/bin/bash

# Enabling xhost for local GUI access from Docker containers
xhost +

# Directory containing the .rec files
REC_DIR="/tmp/rec_files"
# Path to the Gnuplot script
PLOT_SCRIPT="/tmp/plot_script.gp"

# Loop through all .rec files in the REC_DIR
for rec_file in ${REC_DIR}/*.rec; do
    echo "Processing file: $rec_file"

    # Extract filename for use in naming the output image
    base_name=$(basename "$rec_file" .rec)
    output_png="/tmp/${base_name}.png"

    # Start the decoder Docker container in the background
    sudo docker run --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp h264replay:v0.0.4 --cid=111 --verbose --name=img "${rec_file}" &
    decoder_pid=$!

    # Sleep briefly to ensure the first image is processed
    sleep 0.1

    # Start the microservice Docker container in the background
    docker run --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp anglecalculator:latest --cid=111 --name=img --width=640 --height=480 --verbose &
    microservice_pid=$!

    # Wait for the decoder process to finish
    wait $decoder_pid

    # Terminate our angle calculator
    kill $microservice_pid

    # Now that both processes are complete, run the Gnuplot script to generate the plot from plotting_data.csv
    gnuplot -e "filename='$output_png'" $PLOT_SCRIPT

    echo "Completed processing for $rec_file"
done

# Disable xhost for local GUI access from Docker containers
xhost -

echo "All files processed."
