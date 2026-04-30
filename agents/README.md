# ProjectUltra Agent Runner

This directory contains the repo-owned automation used to run Claude Code,
Codex, or another CLI agent on bounded engineering tasks.

The model is intentionally conservative:

- One task file in `agents/queue/` defines one unit of work.
- One worker creates one branch, runs one agent session, then runs gates.
- Reports and prompts are written under `agents/reports/` and `agents/tmp/`.
- Hardware tests are serialized with a lock so two agents cannot use the Mac/Pi audio path at the same time.
- Hardware sentinel runs write structured report-only evidence for the planner.
- Hardware proposal tasks are executed as runner-owned gates outside the model
  sandbox; the agent inspects/logs, while the maintained runner script owns
  SSH/audio access.
- Planner runs write proposed task files for human review; they do not edit
  modem source or auto-merge PRs.
- Planner proposal files are regenerated each planner pass, and publishing
  deduplicates against existing open planner issues by title.
- The planner suppresses new hardware proposals while any open hardware planner
  issue exists, so hardware work stays single-threaded at the issue level.
- The planner treats repeated valid hardware sentinel warnings as engineering
  work: it reads `metrics.tsv` and proposes a bounded repair task instead of
  endlessly asking for another sentinel rerun.
- Auto-commit and push are opt-in. Human review remains the merge gate.
- Queued and archived task files are ignored by default so prompts/log excerpts
  are not accidentally committed.

This is not meant to be an unrestricted shell daemon. Long-running agents are
useful only if the work is decomposed, reproducible, and measurable.

## Quick Start

Create a task:

```bash
cp agents/task_template.md agents/queue/001-my-task.md
```

Run one task with Claude Code or Codex in dry-run mode first:

```bash
AGENT_DRY_RUN=1 AGENT_CMD='claude -p' ./agents/run_next_task.sh
```

Run for real:

```bash
AGENT_CMD='claude -p' ./agents/run_next_task.sh
```

Run continuously from `tmux`:

```bash
AGENT_CMD='claude -p' ./agents/watchdog.sh
```

Run the planner once:

```bash
./agents/run_planner.sh
```

Publish planner proposals as GitHub Issues:

```bash
./agents/publish_planner_proposals.sh
```

Create a human-authored follow-up from a terminal:

```bash
cp agents/manual_followup_template.md /tmp/followup.md
$EDITOR /tmp/followup.md
./agents/create_followup_issue.sh --title "Eliminate residual AWGN retx" --body-file /tmp/followup.md --hardware
```

From GitHub, use the "Agent follow-up proposal" or "Hardware follow-up
proposal" issue template. These issues use the same `/approve codex` and
`/approve claude` approval flow as planner-generated proposals.

Process allowlisted GitHub approvals:

```bash
AGENT_APPROVERS=secup ./agents/process_approved_proposals.sh
```

Run a hardware sentinel once:

```bash
SSH_KEY="$HOME/.ssh/id_pi5" ./agents/run_hardware_sentinel.sh
```

After `tmux` is stable, see `agents/launchd/` for a macOS LaunchAgent example.

If your CLI accepts a prompt file instead of stdin:

```bash
AGENT_PROMPT_MODE=file AGENT_CMD='your-agent --prompt-file' ./agents/run_next_task.sh
```

## Useful Environment

- `AGENT_CMD`: required command used to run the agent.
- `AGENT_NAME`: label for reports and branch names, default `agent`.
- `AGENT_PROMPT_MODE`: `stdin` or `file`, default `stdin`.
- `AGENT_LOCAL_GATE`: local gate command, default `./agents/run_local_gate.sh`.
- `AGENT_RUN_LOCAL_GATE`: set `0` to skip the local gate, default `1`.
- `AGENT_RUN_HARDWARE`: set `1` to run hardware smoke after local gates.
- `AGENT_HARDWARE_CMD`: hardware gate command, default `./agents/run_hardware_smoke.sh`.
- `AGENT_AUTO_HARDWARE_GATE`: set `1` to let tasks mentioning the maintained
  hardware sentinel command trigger a runner-owned hardware gate, default `1`.
- `AGENT_AUTO_HARDWARE_CMD`: command used for auto hardware gates, default
  `SSH_KEY="$HOME/.ssh/id_pi5" ./agents/run_hardware_sentinel.sh`.
- `AGENT_AUTO_COMMIT`: set `1` to commit successful changes.
- `AGENT_PUSH`: set `1` to push the branch after an auto-commit.
- `AGENT_CREATE_PR`: set `1` to create a GitHub PR with `gh`; requires `AGENT_PUSH=1`.
- `AGENT_PR_DRAFT`: set `0` for ready-for-review PRs, default `1`.
- `AGENT_COMMENT_ISSUE_RESULTS`: comment completion output back to the source
  GitHub issue for approved planner tasks, default `1`.
- `AGENT_ALLOW_DIRTY`: set `1` to allow starting from a dirty worktree.
- `AGENT_SLEEP_SECONDS`: watchdog sleep between attempts, default `300`.
- `AGENT_HW_SENTINEL_MODE`: hardware sentinel mode, `quick`, `nightly`, or `full`.
- `AGENT_HW_SENTINEL_AWGN_QUICK_REPEATS`: AWGN quick repetitions, default `3`.
- `AGENT_HW_SENTINEL_MAX_RETX_AWGN_QUICK` /
  `AGENT_HW_SENTINEL_MAX_TIMEOUTS_AWGN_QUICK`: strict AWGN quick limits,
  both default `0`.
- `AGENT_PLANNER_SLEEP_SECONDS`: planner watchdog sleep interval.
- `AGENT_PLANNER_PUBLISH_ISSUES`: set `1` to publish planner proposals as GitHub Issues after each planner run.
- `AGENT_PLANNER_LOCK_DIR`: planner singleton lock directory, default
  `/tmp/projectultra_planner.lock`.
- `AGENT_APPROVERS`: comma-separated GitHub usernames allowed to approve planner issues.
- `AGENT_APPROVAL_SLEEP_SECONDS`: approval watchdog sleep interval.
- `AGENT_APPROVAL_LOCK_DIR`: approval singleton lock directory, default
  `/tmp/projectultra_approval.lock`.

The runner allows pending files under `agents/queue/`, `agents/reports/`, and
`agents/tmp/`. Other dirty files are rejected unless `AGENT_ALLOW_DIRTY=1`.
If a task produces no tracked commits, the runner skips push/PR creation and
posts the report summary back to the source issue when available.

## Permission Policy

Relax permissions by command prefix, not by granting all shell access. The
recommended pattern is:

- allow read-only discovery commands,
- allow `cmake`, `ctest`, maintained regression scripts, and maintained hardware scripts,
- allow branch creation and commits,
- allow PR creation only through `gh pr create` and only for feature branches,
- keep `sudo`, destructive git reset/checkout, and arbitrary network downloads blocked.

See `agents/permissions/claude-settings.example.json` for a repo-scoped Claude
Code example.
