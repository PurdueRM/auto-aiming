#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SERVICE_USER="${SUDO_USER:-$(id -un)}"
SERVICE_HOME="$(getent passwd "$SERVICE_USER" | cut -d: -f6)"
LOG_DIR="$SERVICE_HOME/auto_aim_logs"
AUTO_AIM_SERVICE_NAME="ros2_auto_aim_service"
NAVIGATION_SERVICE_NAME="ros2_navigation_service"
AUTO_AIM_SERVICE_FILE="/etc/systemd/system/$AUTO_AIM_SERVICE_NAME.service"
NAVIGATION_SERVICE_FILE="/etc/systemd/system/$NAVIGATION_SERVICE_NAME.service"

usage() {
    echo "Usage: $0 [-a | -n | -d]"
    echo "  -a   enable auto-aiming service"
    echo "  -n   enable navigation watchdog service"
    echo "  -d   disable both services"
    echo "  -h   show this help"
}

create_wrapper_script() {
    local mode="$1"
    local wrapper_path="$2"

    echo "Creating the $mode wrapper script at $wrapper_path..."

    cat <<EOF > "$wrapper_path"
#!/bin/bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.sh"

sleep 5

LOG_DIR="$LOG_DIR"
mkdir -p "\$LOG_DIR"
LOG_FILE="\$LOG_DIR/\$(date +'%Y-%m-%d_%H-%M-%S')_${mode}_log.txt"

cd "$WORKSPACE_ROOT"

case "$mode" in
    auto_aim)
        exec ros2 launch prm_launch mv2control.py >> "\$LOG_FILE" 2>&1
        ;;
    navigation)
        exec python3 auto-aiming/src/prm_autobot_2023/nav.py >> "\$LOG_FILE" 2>&1
        ;;
    *)
        echo "Unknown mode: $mode" >&2
        exit 1
        ;;
esac
EOF

    chmod +x "$wrapper_path"
    echo "Wrapper script created and made executable."
}

create_service() {
    local service_name="$1"
    local service_file="$2"
    local wrapper_path="$3"
    local description="$4"

    echo "Creating the systemd service at $service_file..."

    cat <<EOL | sudo tee "$service_file" > /dev/null
[Unit]
Description=$description
After=network.target

[Service]
ExecStart=$wrapper_path
WorkingDirectory=$WORKSPACE_ROOT
User=$SERVICE_USER
Group=$(id -gn "$SERVICE_USER")
Restart=always
Environment=ROS_DISTRO=humble
Environment=HOME=$SERVICE_HOME
StandardOutput=append:$LOG_DIR/systemd_service.log
StandardError=append:$LOG_DIR/systemd_service.log

[Install]
WantedBy=multi-user.target
EOL

    sudo systemctl daemon-reload
    echo "Systemd service created and daemon reloaded."
    sudo systemctl enable "$service_name.service"
    sudo systemctl start "$service_name.service"
    echo "$description started and enabled."
}

disable_service() {
    local service_name="$1"
    local service_file="$2"

    if [ -f "$service_file" ]; then
        echo "Stopping and removing the $service_name service..."
        sudo systemctl stop "$service_name.service" || true
        sudo systemctl disable "$service_name.service" || true
        sudo rm -f "$service_file"
        sudo systemctl daemon-reload
        echo "Service stopped, disabled, and removed."
    else
        echo "Service $service_name does not exist. Nothing to disable."
    fi
}

enable_auto_aim() {
    mkdir -p "$LOG_DIR"
    local wrapper_path="$SERVICE_HOME/start_auto_aim.sh"
    create_wrapper_script "auto_aim" "$wrapper_path"
    create_service "$AUTO_AIM_SERVICE_NAME" "$AUTO_AIM_SERVICE_FILE" "$wrapper_path" "ROS 2 Auto Aiming Service"
}

enable_navigation() {
    mkdir -p "$LOG_DIR"
    local wrapper_path="$SERVICE_HOME/start_navigation.sh"
    create_wrapper_script "navigation" "$wrapper_path"
    create_service "$NAVIGATION_SERVICE_NAME" "$NAVIGATION_SERVICE_FILE" "$wrapper_path" "ROS 2 Navigation Service"
}

disable_all() {
    disable_service "$AUTO_AIM_SERVICE_NAME" "$AUTO_AIM_SERVICE_FILE"
    disable_service "$NAVIGATION_SERVICE_NAME" "$NAVIGATION_SERVICE_FILE"
}

if [[ $# -eq 0 ]]; then
    usage
    exit 1
fi

while getopts ":a:n:d:h" opt; do
    case "$opt" in
        a)
            enable_auto_aim
            exit 0
            ;;
        n)
            enable_navigation
            exit 0
            ;;
        d)
            disable_all
            exit 0
            ;;
        h)
            usage
            exit 0
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            exit 1
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
    esac
done

# Backward compatibility with the old flag name
if [[ "${1:-}" == "--disable" ]]; then
    disable_all
    exit 0
fi

usage
exit 1
