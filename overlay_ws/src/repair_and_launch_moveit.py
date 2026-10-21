#!/usr/bin/env python3
"""Repair Kinova/MoveIt compatibility and no-gripper controller settings, build, launch.

Run INSIDE the ROS2 Humble Docker container:
    python3 /overlay_ws/src/repair_and_launch_moveit.py
To apply only repairs/build (without launching move_group):
    python3 /overlay_ws/src/repair_and_launch_moveit.py --repair-only

This is for the exact ros2_kortex layout observed in the project. The source
vendor sources live in /colcon_ws/src and may be lost when a container
is recreated. The custom gen3_moveit_config in /overlay_ws/src is persistent
when that directory is bind-mounted. Repairs are safe to re-run.
"""
import argparse
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path('/colcon_ws')
XACRO = ROOT / 'src/ros2_kortex/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro'
OMPL = ROOT / 'src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_moveit_config/config/ompl_planning.yaml'
MOVEIT_PACKAGE = 'kinova_gen3_7dof_robotiq_2f_85_moveit_config'
OVERLAY_ROOT = Path('/overlay_ws')
CUSTOM_PACKAGE = 'gen3_moveit_config'
CUSTOM_CONFIG = OVERLAY_ROOT / 'src' / CUSTOM_PACKAGE / 'config'


def backup_and_write(path, original, updated):
    if updated == original:
        print(f'[无需修改] {path}')
        return False
    backup = path.with_name(path.name + '.before_auto_fix.bak')
    if not backup.exists():
        shutil.copy2(path, backup)
        print(f'[已备份] {backup}')
    path.write_text(updated, encoding='utf-8')
    print(f'[已修复] {path}')
    return True


def repair_xacro():
    text = XACRO.read_text(encoding='utf-8')
    # Only modify parameters on the <xacro:robotiq_gripper ...> invocation,
    # not the outer macro's declared defaults or any other macros.
    pat = re.compile(r'<xacro:robotiq_gripper\b[^>]*>', re.DOTALL)
    matches = list(pat.finditer(text))
    if len(matches) != 1:
        raise RuntimeError(f'Expected one <xacro:robotiq_gripper> invocation; found {len(matches)}. Stopping.')
    match = matches[0]
    tag = match.group(0)
    tag = tag.replace('mock_sensor_commands=', 'fake_sensor_commands=')
    for name in ('sim_gazebo', 'isaac_joint_commands', 'isaac_joint_states'):
        tag = re.sub(r'\s+' + name + r'="[^"]*"', '', tag)
    updated = text[:match.start()] + tag + text[match.end():]
    # Validate *before* changing source.
    ET.fromstring(updated)
    return backup_and_write(XACRO, text, updated)


def repair_ompl():
    text = OMPL.read_text(encoding='utf-8')
    lines = text.splitlines(keepends=True)
    out = []
    changed = set()
    i = 0
    while i < len(lines):
        line = lines[i]
        m = re.match(r'^(request_adapters|response_adapters):\s*(?:#.*)?\r?\n$', line)
        if m:
            key = m.group(1)
            j = i + 1
            adapters = []
            while j < len(lines):
                item = re.match(r'^\s+-\s+([^#\r\n]+?)\s*(?:#.*)?\r?\n?$', lines[j])
                if item:
                    adapters.append(item.group(1).strip())
                    j += 1
                else:
                    break
            if adapters:
                out.append(f'{key}: "{" ".join(adapters)}"\n')
                changed.add(key)
                i = j
                continue
        out.append(line)
        i += 1
    updated = ''.join(out)
    # ROS environments normally supply PyYAML; verify YAML without otherwise
    # reformatting/comment-stripping this large planning configuration.
    try:
        import yaml
        data = yaml.safe_load(updated)
        for key in ('request_adapters', 'response_adapters'):
            if key in data and not isinstance(data[key], str):
                raise RuntimeError(f'{key} must be a string after repair, got {type(data[key]).__name__}')
    except ImportError:
        raise RuntimeError('PyYAML is missing: install python3-yaml before running the script')
    return backup_and_write(OMPL, text, updated)



def repair_custom_controllers():
    """Remove Robotiq controller entries from the persistent no-gripper package."""
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError('PyYAML is required (python3-yaml)') from exc

    if not CUSTOM_CONFIG.is_dir():
        print(f'[跳过] 找不到独立配置目录：{CUSTOM_CONFIG}')
        return False

    changed = False
    moveit_path = CUSTOM_CONFIG / 'moveit_controllers.yaml'
    ros2_path = CUSTOM_CONFIG / 'ros2_controllers.yaml'
    for path in (moveit_path, ros2_path):
        if not path.is_file():
            raise RuntimeError(f'找不到配置文件：{path}')

    text = moveit_path.read_text(encoding='utf-8')
    config = yaml.safe_load(text)
    if not isinstance(config, dict) or not isinstance(config.get('moveit_simple_controller_manager'), dict):
        raise RuntimeError(f'配置结构不符合预期：{moveit_path}')
    mgr = config['moveit_simple_controller_manager']
    names = mgr.get('controller_names')
    if not isinstance(names, list) or 'joint_trajectory_controller' not in names:
        raise RuntimeError('MoveIt 控制器配置缺少 joint_trajectory_controller，停止以避免误修改')
    modified = False
    if 'robotiq_gripper_controller' in names:
        mgr['controller_names'] = [name for name in names if name != 'robotiq_gripper_controller']
        modified = True
    if 'robotiq_gripper_controller' in mgr:
        mgr.pop('robotiq_gripper_controller')
        modified = True
    if modified:
        changed |= backup_and_write(moveit_path, text, yaml.safe_dump(config, sort_keys=False))
    else:
        print(f'[无需修改] {moveit_path}')

    text = ros2_path.read_text(encoding='utf-8')
    config = yaml.safe_load(text)
    if not isinstance(config, dict) or not isinstance(config.get('controller_manager'), dict):
        raise RuntimeError(f'配置结构不符合预期：{ros2_path}')
    params = config['controller_manager'].get('ros__parameters')
    if not isinstance(params, dict) or 'joint_trajectory_controller' not in params:
        raise RuntimeError('ros2_control 配置缺少 joint_trajectory_controller，停止以避免误修改')
    modified = False
    if 'robotiq_gripper_controller' in params:
        params.pop('robotiq_gripper_controller')
        modified = True
    if 'robotiq_gripper_controller' in config:
        config.pop('robotiq_gripper_controller')
        modified = True
    if modified:
        changed |= backup_and_write(ros2_path, text, yaml.safe_dump(config, sort_keys=False))
    else:
        print(f'[无需修改] {ros2_path}')
    return changed


def run_build():
    packages = ['kortex_description', MOVEIT_PACKAGE]
    cmd = ['colcon', 'build', '--packages-select', *packages]
    print('[构建] ' + ' '.join(cmd), flush=True)
    subprocess.run(cmd, cwd=str(ROOT), check=True)
    if (OVERLAY_ROOT / 'src' / CUSTOM_PACKAGE / 'package.xml').is_file():
        overlay_cmd = ['colcon', 'build', '--packages-select', CUSTOM_PACKAGE]
        print('[构建] 在 /overlay_ws 中编译独立配置包', flush=True)
        subprocess.run(overlay_cmd, cwd=str(OVERLAY_ROOT), check=True)


def launch():
    package = CUSTOM_PACKAGE if (OVERLAY_ROOT / 'src' / CUSTOM_PACKAGE / 'package.xml').is_file() else MOVEIT_PACKAGE
    cmd = ('source /opt/ros/humble/setup.bash && '
           'source /colcon_ws/install/setup.bash && '
           'source /overlay_ws/install/setup.bash && '
           f'exec ros2 launch {package} move_group.launch.py')
    print('[启动] MoveIt 2 move_group；Ctrl+C 退出', flush=True)
    subprocess.run(['bash', '-lc', cmd], check=True)


def main():
    parser = argparse.ArgumentParser(description='Fix and launch Kinova MoveIt 2 in Docker')
    parser.add_argument('--repair-only', action='store_true', help='Fix and build without starting MoveIt')
    args = parser.parse_args()
    for f in (XACRO, OMPL):
        if not f.is_file():
            parser.error(f'Expected source file not found: {f}; container/image layout may have changed')
    try:
        repair_xacro()
        repair_ompl()
        repair_custom_controllers()
        run_build()
        if not args.repair_only:
            launch()
    except KeyboardInterrupt:
        print('\n[退出] 用户中断')
        return 130
    except (RuntimeError, ET.ParseError, subprocess.CalledProcessError, OSError) as e:
        print(f'\n[失败] {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
