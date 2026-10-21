
#!/usr/bin/env bash
set -euo pipefail

CONTAINER="test"
SESSION="gen3_system"

# 改成 Docker 能访问的 Windows 主机 IP
WINDOWS_IP="192.168.1.11"
HTTP_PORT="8000"

COMMAND="${1:-start}"

container() {
    docker exec "$CONTAINER" "$@"
}

case "$COMMAND" in

start)
    echo "启动 Docker 容器..."
    docker start "$CONTAINER" >/dev/null

    if ! container bash -lc "command -v tmux" >/dev/null; then
        echo "错误：容器中没有 tmux"
        echo "请先运行："
        echo "docker exec -it $CONTAINER bash"
        echo "apt update && apt install -y tmux"
        exit 1
    fi

    if container tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "Gen3 系统已经启动。"
        exit 0
    fi

    # 在容器内创建统一的节点启动脚本
    container bash -c 'cat > /tmp/gen3_node.sh' <<'NODE'
#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /colcon_ws/install/setup.bash ]; then
    source /colcon_ws/install/setup.bash
fi

source /overlay_ws/install/setup.bash

case "$1" in

robot)
    exec ros2 launch kortex_bringup gen3.launch.py \
        robot_ip:=192.168.1.10 \
        use_fake_hardware:=true \
        launch_rviz:=false
    ;;

moveit)
    exec ros2 launch gen3_moveit_config move_group.launch.py
    ;;

rviz)
    exec ros2 launch gen3_moveit_config moveit_rviz.launch.py
    ;;

wifi)
    exec ros2 run esp_wifi_pub esp_wifi_publisher \
        --ros-args \
        -p host:="$2" \
        -p port:="$3" \
        -p path:=/data \
        -p hz:=10.0
    ;;

bridge)
    exec ros2 run gen3_target_control pose_bridge
    ;;

visualizer)
    exec ros2 run gen3_target_control target_visualizer
    ;;

controller)
    exec ros2 run gen3_target_control target_controller
    ;;

esac
NODE

    container chmod +x /tmp/gen3_node.sh

    echo "启动 Gen3 仿真..."
    container tmux new-session -d \
        -s "$SESSION" -n robot \
        "bash /tmp/gen3_node.sh robot"

    sleep 5

    echo "启动 MoveIt..."
    container tmux new-window \
        -t "$SESSION" -n moveit \
        "bash /tmp/gen3_node.sh moveit"

    sleep 5

    echo "启动 RViz..."
    container tmux new-window \
        -t "$SESSION" -n rviz \
        "bash /tmp/gen3_node.sh rviz"

    echo "启动 WiFi..."
    container tmux new-window \
        -t "$SESSION" -n wifi \
        "bash /tmp/gen3_node.sh wifi $WINDOWS_IP $HTTP_PORT"

    echo "启动 Pose Bridge..."
    container tmux new-window \
        -t "$SESSION" -n bridge \
        "bash /tmp/gen3_node.sh bridge"

    echo "启动 Target Visualizer..."
    container tmux new-window \
        -t "$SESSION" -n visualizer \
        "bash /tmp/gen3_node.sh visualizer"

    echo "启动 Target Controller..."
    container tmux new-window \
        -t "$SESSION" -n controller \
        "bash /tmp/gen3_node.sh controller"

    echo ""
    echo "ROS2 节点启动命令已发送！"
    echo "查看终端：./gen3.sh view"
    echo "查看状态：./gen3.sh status"
    echo "停止系统：./gen3.sh stop"
    ;;

view)
    container tmux attach-session -t "$SESSION"
    ;;

status)
    container tmux list-windows -t "$SESSION"
    ;;

stop)
    echo "停止 Gen3 系统..."
    if container tmux has-session -t "$SESSION" 2>/dev/null; then
        container tmux kill-session -t "$SESSION"
        echo "所有托管节点已停止。"
    else
        echo "系统没有运行。"
    fi
    ;;

*)
    echo "用法：$0 {start|view|status|stop}"
    exit 1
    ;;
esac
