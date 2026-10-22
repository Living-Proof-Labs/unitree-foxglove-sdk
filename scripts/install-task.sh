#!/usr/bin/env bash
# Pinned, checksum-verified Task installation, shared by CI and local setup.
set -euo pipefail
if [[ "$(uname -s)-$(uname -m)" != Linux-x86_64 ]]; then
  echo "This installer supports Linux x86_64; install Task 3.48.0 for your platform." >&2
  exit 64
fi
install_dir="${1:?usage: bash scripts/install-task.sh INSTALL_DIRECTORY}"
task_tmp="$(mktemp -d)"
trap 'rm -rf "$task_tmp"' EXIT
curl -fsSL --retry 3 https://github.com/go-task/task/releases/download/v3.48.0/task_linux_amd64.tar.gz -o "$task_tmp/task_linux_amd64.tar.gz"
(cd "$task_tmp" && echo 'f4bfc4eef1b2557b262f3cc0a79976a421885cb7b9e71cfe75568a2ebe4d7ae5  task_linux_amd64.tar.gz' | sha256sum -c -)
tar -xzf "$task_tmp/task_linux_amd64.tar.gz" -C "$task_tmp" task
install -Dm755 "$task_tmp/task" "$install_dir/task"
"$install_dir/task" --version
