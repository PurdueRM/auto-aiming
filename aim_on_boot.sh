#!/bin/bash

SERVICE_NAME="ros2_aiming_service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"

# Paths
USER_HOME="/home/jason"
LOG_DIR="$USER_HOME/auto_aim_logs"
WRAPPER_SCRIPT="$USER_HOME/start_auto_aim.sh"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Create the wrapper script
create_wrapper_script() {
    echo "Creating the wrapper script at $WRAPPER_SCRIPT..."

    cat << 'EOF' > "$WRAPPER_SCRIPT"
#!/bin/bash

# Setup ROS environment
# source /home/jason/ros2-ws/install/setup.bash

source /opt/ros/humble/setup.bash
source /home/jason/ros2-ws/install/setup.sh

# Sleep to give system time to stabilize
sleep 5

# Create log directory and log file
LOG_DIR="/home/jason/auto_aim_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/$(date +'%Y-%m-%d_%H-%M-%S')_auto_aim_log.txt"

# Change to workspace and exec Python script
cd /home/jason/ros2-ws
exec ros2 launch prm_launch mv2control.py >> "$LOG_FILE" 2>&1
EOF

    chmod +x "$WRAPPER_SCRIPT"
    echo "Wrapper script created and made executable."
}

# Create the systemd service unit file
create_service() {
    echo "Creating the systemd service at $SERVICE_FILE..."

    cat <<EOL | sudo tee "$SERVICE_FILE" > /dev/null
[Unit]
Description=ROS 2 Auto Aiming Service
After=network.target

[Service]
ExecStart=$WRAPPER_SCRIPT
WorkingDirectory=$USER_HOME/ros2-ws
User=jason
Group=jason
Restart=always
Environment=ROS_DISTRO=humble
Environment=HOME=$USER_HOME
StandardOutput=append:$LOG_DIR/systemd_service.log
StandardError=append:$LOG_DIR/systemd_service.log

[Install]
WantedBy=multi-user.target
EOL

    sudo systemctl daemon-reload
    echo "Systemd service created and daemon reloaded."
}

# Enable and start the service
enable_service() {
    echo "Enabling and starting the service..."
    sudo systemctl enable "$SERVICE_NAME.service"
    sudo systemctl start "$SERVICE_NAME.service"
    echo "Service started and enabled."
}

# Disable and remove the service
disable_service() {
    if [ -f "$SERVICE_FILE" ]; then
        echo "Stopping and removing the service..."
        sudo systemctl stop "$SERVICE_NAME.service"
        sudo systemctl disable "$SERVICE_NAME.service"
        sudo rm "$SERVICE_FILE"
        sudo systemctl daemon-reload
        echo "Service stopped, disabled, and removed."
    else
        echo "Service does not exist. Nothing to disable."
    fi
}

# Main logic: parse arguments
if [[ "$1" == "--disable" ]]; then
    disable_service
else
    create_wrapper_script
    create_service
    enable_service
fi
