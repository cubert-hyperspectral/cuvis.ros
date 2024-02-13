#!/bin/bash

# Define the content of jumbo.sh - CHANGE THE INTERFACE NAME!
JUMBO_CONTENT="interface_name=enp61s0\nip link set \$interface_name mtu 9000"

# Specify the destination path for jumbo.sh
DESTINATION_PATH="/etc/init.d/jumbo.sh"

# Copy jumbo.sh to the destination path
sudo cp <(echo -e "$JUMBO_CONTENT") "$DESTINATION_PATH"

# Set the appropriate permissions
sudo chmod 775 "$DESTINATION_PATH"

# Update the init.d configuration
sudo update-rc.d jumbo.sh defaults

echo "Installation completed successfully!"