#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"

fail=0
scanned=0
findings=()

add_finding() {
  local path="$1"
  local type="$2"

  findings+=("$path: $type")
  fail=1
}

is_allowed_agent_path() {
  local path="$1"

  case "$path" in
    agents/queue/.gitignore|agents/queue/.gitkeep|agents/queue/README.md)
      return 0
      ;;
    agents/archive/.gitignore|agents/archive/.gitkeep)
      return 0
      ;;
    agents/reports/.gitignore|agents/reports/.gitkeep)
      return 0
      ;;
    agents/tmp/.gitignore|agents/tmp/.gitkeep)
      return 0
      ;;
    agents/planner/proposals/.gitignore|agents/planner/proposals/.gitkeep)
      return 0
      ;;
    agents/planner/reports/.gitignore|agents/planner/reports/.gitkeep)
      return 0
      ;;
  esac

  return 1
}

is_agent_artifact_path() {
  local path="$1"

  case "$path" in
    agents/queue/*|agents/archive/*|agents/reports/*|agents/tmp/*|agents/planner/proposals/*|agents/planner/reports/*)
      ! is_allowed_agent_path "$path"
      return
      ;;
    .claude/*|.codex|.codex/*)
      return 0
      ;;
  esac

  return 1
}

is_allowed_hw_log_path_file() {
  local path="$1"

  case "$path" in
    CLAUDE.md|tools/run_hw_test.sh|.github/ISSUE_TEMPLATE/agent_followup.yml|.github/ISSUE_TEMPLATE/hardware_followup.yml)
      return 0
      ;;
  esac

  return 1
}

has_match() {
  local pattern="$1"
  local path="$2"

  LC_ALL=C grep -I -E -q -- "$pattern" "$path"
}

has_fixed_match() {
  local pattern="$1"
  local path="$2"

  LC_ALL=C grep -I -F -q -- "$pattern" "$path"
}

private_key_re='-----BEGIN ((OPENSSH|RSA|DSA|EC|ED25519|ENCRYPTED) )?PRIVATE KEY-----'
putty_private_key_re='^PuTTY-User-Key-File-[0-9]+: ssh-'
github_classic_token_re='gh[pousr]_[[:alnum:]_]{30,}'
github_fine_grained_token_re='github_pat_[[:alnum:]_]{50,}'
hw_log_prefix="/tmp/ultra""_hw_"

while IFS= read -r -d '' path; do
  [[ -f "$path" ]] || continue
  scanned=$((scanned + 1))

  if is_agent_artifact_path "$path"; then
    add_finding "$path" "local agent artifact or metadata"
  fi

  if has_match "$private_key_re" "$path" || has_match "$putty_private_key_re" "$path"; then
    add_finding "$path" "private key"
  fi

  if has_match "$github_classic_token_re" "$path" || has_match "$github_fine_grained_token_re" "$path"; then
    add_finding "$path" "GitHub token"
  fi

  if has_fixed_match "$hw_log_prefix" "$path" && ! is_allowed_hw_log_path_file "$path"; then
    add_finding "$path" "hardware log path"
  fi
done < <(git ls-files --cached --others --exclude-standard -z)

if [[ "$fail" -ne 0 ]]; then
  echo "Artifact/secret check failed. No secret values are printed below." >&2
  printf '%s\n' "${findings[@]}" | sort -u | sed 's/^/  - /' >&2
  exit 1
fi

echo "Artifact/secret check passed ($scanned files scanned)."
