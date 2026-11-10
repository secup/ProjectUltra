# Documentation Map

Last updated: 2026-05-07

This folder separates active source-of-truth docs from historical reference docs.
Agents must treat `docs/archive/` as background only unless a task explicitly
cites an archived file.

## Start Here

- `docs/PROJECT_GOALS.md`: Mission, priorities, throughput/reliability targets, and task filter.
- `docs/INVARIANTS.md`: Critical modem invariants that must not be weakened.
- `docs/KNOWN_BUGS.md`: Current open reliability/throughput issues only.

## Operator Path

- `docs/RUNNING.md`: One-page runbook for the normal `ultra_tnc` operator flow.
- `docs/TNC_INTERFACE.md`: TCP command/data interface exposed by `ultra_tnc`.
- `README.md`: Top-level operator overview, build instructions, and status.

## Diagnostic / Lab Tools

- `tools/gui_qso_scenario.sh`: **faithful full-protocol + fade/throughput gate** (two real `ultra_gui -sim` stations over `ota_simulator serve`).
- `tools/measure_ack_fer.cpp`: ACK/FER measurement over the real StreamingEncoder/Decoder.
- `ultra ptx/prx`: send one frame the way a session sends it / decode every frame of a recording
  (any WAV rate), both through ModemEngine, the GUI's modem.

## Quality Workflow

- `docs/QUALITY_STRATEGY.md`: Critical-software test, coverage, CI, and refactor policy.
- `docs/QUALITY_AUDIT.md`: Current quality baseline, coverage gaps, and hardening backlog.
- `docs/COVERAGE_MAP.md`: Module-specific critical coverage expectations and test priorities.
- `docs/GIT_WORKFLOW.md`: Commit/push/release workflow used in this repo.
- `docs/ALPHA_RELEASE_GATE.md`: Alpha readiness criteria and gate commands.

## Current Implementation References

- `docs/PROTOCOL_V2.md`: Current protocol behavior/spec aligned to implementation.
- `docs/BUILD_SYSTEM.md`: Build instructions, targets, dependencies, and coverage entry points.
- `docs/AUDIO_SYSTEM.md`: Audio I/O architecture.
- `docs/CONFIGURATION_SYSTEM.md`: Configuration model and runtime flow.
- `docs/GUI_ARCHITECTURE.md`: GUI structure and components.
- `docs/CFO_CORRECTION_FLOW.md`: Current CFO handling pipeline.
- `docs/ADDING_NEW_WAVEFORM.md`: Guide for adding waveform implementations.
- `docs/CHANGELOG.md`: Historical fixes and behavior changes.

## Archived Docs

Archived docs are retained for traceability and postmortem context but are not
source-of-truth for current behavior.

- `docs/archive/`
- Dated session reports, one-off audit reports, validation postmortems, and
  superseded research plans live here unless a task explicitly promotes them
  back to active source-of-truth status.

Do not base production changes on archived plans without first verifying current
code and adding a fresh task or design note.
