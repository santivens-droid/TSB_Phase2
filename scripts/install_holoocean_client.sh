#!/bin/bash

# HoloOcean Client Installation Script
# This script removes any old HoloOcean client installation and installs the new one

# Remove old HoloOcean client installation
echo "Removing old HoloOcean client installation..."
pip3 uninstall -y holoocean

# Change to the holoocean_client directory
cd /ws/holoocean/client

# Install HoloOcean client
echo "Installing HoloOcean client..."
pip3 install .

# Return to workspace root
cd /ws
