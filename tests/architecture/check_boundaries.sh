#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo_root"

failed=0

if rg -n '#include[[:space:]]*[<"](rclcpp|rclcpp_action|msgs)/' \
    master_ws/src/core master_ws/src/*/core exector_ws/src/*/core 2>/dev/null; then
  echo "ERROR: transport-neutral core code depends on ROS 2"
  failed=1
fi

if rg -n 'rclcpp::|create_(publisher|subscription|client|service)' \
    master_ws/src/core master_ws/src/*/core exector_ws/src/*/core 2>/dev/null; then
  echo "ERROR: ROS 2 runtime calls found in core code"
  failed=1
fi

if rg -n '<(LaunchModule|SuperviseModule|RoleGate)' config/behavior_trees; then
  echo "ERROR: process supervision leaked into a business behavior tree"
  failed=1
fi

if rg -n '"/(controller|mission|behaviors|episode)/' \
    master_ws/src/behavior/ros2 master_ws/src/controller master_ws/src/scene 2>/dev/null; then
  echo "ERROR: vehicle-local ROS graph names must be relative"
  failed=1
fi

while IFS= read -r manifest; do
  case "$manifest" in
    master_ws/src/core/framework/package.xml|master_ws/src/controller/package.xml|master_ws/src/platform/ros2/*/package.xml|master_ws/src/*/ros2/package.xml|master_ws/src/behavior/ros2/*/package.xml|exector_ws/src/*/ros2/package.xml|exector_ws/src/bt_plugins/package.xml|thirdparty/behaviortree_cpp/package.xml|thirdparty/vendor/*/package.xml)
      ;;
    *)
      echo "ERROR: ROS package is outside an approved ros2 directory: $manifest"
      failed=1
      ;;
  esac
done < <(find master_ws exector_ws thirdparty \
  -path '*/inactive' -prune -o -name package.xml -print | sort)

for config_file in config/executors/*.json config/transitions/*.json; do
  jq empty "$config_file"
done

for manifest in config/executors/phase_*.json; do
  if [[ $(jq '.libs | length' "$manifest") -lt 1 ]]; then
    echo "ERROR: phase manifest must declare at least one dynamic plugin: $manifest"
    failed=1
  fi
done

expected_phases=(standby approch onstation find track report refind)
for phase in "${expected_phases[@]}"; do
  [[ -f "config/executors/phase_${phase}.json" ]] || {
    echo "ERROR: missing phase manifest: $phase"
    failed=1
  }
  [[ -f "config/behavior_trees/phase_${phase}.xml" ]] || {
    echo "ERROR: missing phase tree: $phase"
    failed=1
  }
done

for shared_phase in approch find track refind; do
  manifest="config/executors/phase_${shared_phase}.json"
  if [[ $(jq '.libs | length' "$manifest") -lt 3 ]]; then
    echo "ERROR: shared phase must load multiple capability plugins: $manifest"
    failed=1
  fi
done

if rg -n 'registerCoreNodes' master_ws/src/behavior/ros2/app; then
  echo "ERROR: behavior executor must load phase nodes from manifest plugins"
  failed=1
fi

/usr/bin/python3 -c \
  'path="master_ws/src/platform/ros2/launch/launch/system.launch.py"; compile(open(path).read(), path, "exec")'

if ((failed != 0)); then
  exit 1
fi

echo "Architecture boundary checks passed"
