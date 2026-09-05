#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'USAGE'
Usage: run_release_smoke.sh --workspace-root PATH --output-root NEW_PATH \
  --baseline FULL_MAIN_SHA [--ros-distro DISTRO]

The output root must not exist. The script performs a non-symlink colcon build,
tests the installed tree, builds the standalone Orin preflight, and then runs
the C++ release smoke reporter in an isolated ROS domain.
USAGE
}

workspace_root=""
output_root=""
baseline=""
ros_distro="${ROS_DISTRO:-humble}"

while (($# > 0)); do
  case "$1" in
    --workspace-root)
      workspace_root="${2:?--workspace-root requires a value}"
      shift 2
      ;;
    --output-root)
      output_root="${2:?--output-root requires a value}"
      shift 2
      ;;
    --baseline)
      baseline="${2:?--baseline requires a value}"
      shift 2
      ;;
    --ros-distro)
      ros_distro="${2:?--ros-distro requires a value}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      printf 'run_release_smoke.sh: unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$workspace_root" || -z "$output_root" || -z "$baseline" ]]; then
  usage >&2
  exit 2
fi
if [[ ! "$baseline" =~ ^[0-9a-fA-F]{40}$ ]]; then
  printf 'run_release_smoke.sh: baseline must be a full Git SHA\n' >&2
  exit 2
fi
if [[ ! -d "$workspace_root/.git" && ! -f "$workspace_root/.git" ]]; then
  printf 'run_release_smoke.sh: workspace is not a Git worktree: %s\n' "$workspace_root" >&2
  exit 2
fi
if [[ -e "$output_root" ]]; then
  printf 'run_release_smoke.sh: output root must not already exist: %s\n' "$output_root" >&2
  exit 2
fi
if [[ ! -f "/opt/ros/$ros_distro/setup.bash" ]]; then
  printf 'run_release_smoke.sh: ROS setup is unavailable: /opt/ros/%s/setup.bash\n' "$ros_distro" >&2
  exit 2
fi

commit="$(git -C "$workspace_root" rev-parse HEAD)"
if [[ -n "$(git -C "$workspace_root" status --porcelain)" ]]; then
  printf 'run_release_smoke.sh: workspace must be clean\n' >&2
  exit 2
fi
if ! git -C "$workspace_root" cat-file -e "$baseline^{commit}"; then
  printf 'run_release_smoke.sh: baseline object is unavailable: %s\n' "$baseline" >&2
  exit 2
fi
if ! git -C "$workspace_root" merge-base --is-ancestor "$baseline" "$commit"; then
  printf 'run_release_smoke.sh: baseline is not an ancestor of commit\n' >&2
  exit 2
fi

mkdir -p "$output_root"
set +u
source "/opt/ros/$ros_distro/setup.bash"
set -u
export ROS2CLI_NO_DAEMON=1
export ROS_DOMAIN_ID="$((($$ % 100) + 1))"

rosdep_status="NOT_RUN"
if command -v rosdep >/dev/null 2>&1; then
  set +e
  rosdep check --from-paths "$workspace_root/src" --ignore-src \
    >"$output_root/rosdep-check.log" 2>&1
  rosdep_code=$?
  set -e
  if ((rosdep_code == 0)); then
    rosdep_status="PASS"
  elif grep -q "has not been initialized" "$output_root/rosdep-check.log"; then
    rosdep_status="UNAVAILABLE"
  else
    rosdep_status="FAIL"
  fi
else
  printf 'rosdep command unavailable\n' >"$output_root/rosdep-check.log"
  rosdep_status="UNAVAILABLE"
fi

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

cmake -S "$workspace_root/tools/orin_hardware_evidence" \
  -B "$output_root/orin-build" -DBUILD_TESTING=ON
cmake --build "$output_root/orin-build"
ctest --test-dir "$output_root/orin-build" --output-on-failure

runner="$output_root/install/auto_aim_release_smoke/lib/auto_aim_release_smoke/auto_aim_release_smoke"
if [[ ! -x "$runner" ]]; then
  runner="$output_root/install/lib/auto_aim_release_smoke/auto_aim_release_smoke"
fi
if [[ ! -x "$runner" ]]; then
  printf 'run_release_smoke.sh: installed smoke runner is missing\n' >&2
  exit 1
fi

"$runner" \
  --install-base "$output_root/install" \
  --output-dir "$output_root/report" \
  --baseline "$baseline" \
  --commit "$commit" \
  --orin-preflight "$output_root/orin-build/orin_environment_preflight" \
  --rosdep-status "$rosdep_status"
