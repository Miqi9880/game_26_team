#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'USAGE'
Usage: run_ros_message_e2e.sh --workspace-root PATH --output-root NEW_PATH \
  --baseline FULL_MAIN_SHA [--ros-distro DISTRO] [--rounds N]

Builds and tests in isolated directories, then runs the complete C++ ROS
message-level E2E matrix. The output root must not already exist.
USAGE
}

workspace_root=""
output_root=""
baseline=""
ros_distro="${ROS_DISTRO:-humble}"
rounds=5

while (($# > 0)); do
  case "$1" in
    --workspace-root) workspace_root="${2:?missing workspace root}"; shift 2 ;;
    --output-root) output_root="${2:?missing output root}"; shift 2 ;;
    --baseline) baseline="${2:?missing baseline}"; shift 2 ;;
    --ros-distro) ros_distro="${2:?missing ROS distro}"; shift 2 ;;
    --rounds) rounds="${2:?missing rounds}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) printf 'unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$workspace_root" || -z "$output_root" || -z "$baseline" ]]; then
  usage >&2
  exit 2
fi
if [[ ! "$baseline" =~ ^[0-9a-fA-F]{40}$ ]]; then
  printf 'baseline must be a full Git SHA\n' >&2
  exit 2
fi
if [[ ! "$rounds" =~ ^[1-9][0-9]*$ ]]; then
  printf 'rounds must be positive\n' >&2
  exit 2
fi
if [[ -e "$output_root" ]]; then
  printf 'output root must not already exist: %s\n' "$output_root" >&2
  exit 2
fi
if [[ ! -f "/opt/ros/$ros_distro/setup.bash" ]]; then
  printf 'ROS setup is unavailable: /opt/ros/%s/setup.bash\n' "$ros_distro" >&2
  exit 2
fi
if [[ -n "$(git -C "$workspace_root" status --porcelain)" ]]; then
  printf 'workspace must be clean; preserve and commit reviewed changes first\n' >&2
  exit 2
fi

candidate="$(git -C "$workspace_root" rev-parse HEAD)"
git -C "$workspace_root" cat-file -e "$baseline^{commit}"
git -C "$workspace_root" merge-base --is-ancestor "$baseline" "$candidate"
mkdir -p "$output_root"

set +u
source "/opt/ros/$ros_distro/setup.bash"
set -u
export ROS2CLI_NO_DAEMON=1
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="$((100 + ($$ % 100)))"

colcon --log-base "$output_root/log/build" build \
  --base-paths "$workspace_root/src" \
  --build-base "$output_root/build" \
  --install-base "$output_root/install" \
  --cmake-args -DBUILD_TESTING=ON \
  --event-handlers console_direct+

set +u
source "$output_root/install/setup.bash"
set -u
colcon --log-base "$output_root/log/test" test \
  --base-paths "$workspace_root/src" \
  --build-base "$output_root/build" \
  --install-base "$output_root/install" \
  --event-handlers console_direct+
colcon test-result --test-result-base "$output_root/build" --verbose \
  >"$output_root/colcon-test-result.txt"
git -C "$workspace_root" diff --check >"$output_root/git-diff-check.txt"

runner="$output_root/install/auto_aim_ros_e2e/lib/auto_aim_ros_e2e/auto_aim_ros_e2e"
if [[ ! -x "$runner" ]]; then
  runner="$output_root/install/lib/auto_aim_ros_e2e/auto_aim_ros_e2e"
fi
if [[ ! -x "$runner" ]]; then
  printf 'installed E2E runner is missing\n' >&2
  exit 1
fi

"$runner" \
  --install-base "$output_root/install" \
  --output-dir "$output_root/report" \
  --baseline "$baseline" \
  --commit "$candidate" \
  --rounds "$rounds" \
  --seed 260033
