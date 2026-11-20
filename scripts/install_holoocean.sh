#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing HoloOcean client and package..."

# Call the client installation script
"$SCRIPT_DIR/install_holoocean_client.sh"

# Call the package installation script
"$SCRIPT_DIR/install_holoocean_package.sh"

echo "HoloOcean installation complete!"