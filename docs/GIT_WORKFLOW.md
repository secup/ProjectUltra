# Git Workflow

**Purpose:** Document the git commit and branching strategy for ProjectUltra.

---

## Overview

ProjectUltra uses a **simple trunk-based workflow** suitable for a research project:

- All development on `main` branch
- Commit early, commit often
- Push when stable
- No complex branching

---

## Branching Strategy

### Main Branch (`main`)

- **Primary development branch**
- All commits go here directly
- Should always build (no broken commits)
- May have WIP commits (marked with `WIP:` prefix)

### When to Use Feature Branches

Feature branches are optional but recommended for:
- Large refactors (multi-day work)
- Experimental changes you might abandon
- Changes that break tests temporarily

```bash
# Create feature branch
git checkout -b feature/new-waveform

# Work on feature...
git commit -m "WIP: Initial AFDM implementation"
git commit -m "Add AFDM modulator"

# Merge back to main when done
git checkout main
git merge feature/new-waveform
git branch -d feature/new-waveform
```

---

## Commit Guidelines

### When to Commit

| Situation | Action |
|-----------|--------|
| Fixed a bug | Commit immediately |
| Completed a feature | Commit |
| Made progress on large task | Commit with `WIP:` prefix |
| About to try something risky | Commit first (safety checkpoint) |
| End of work session | Commit current state |

### Commit Message Format

```
<type>: <short description>

<optional body>
- What was changed
- Why it was changed

Fixes: BUG-XXX (if applicable)

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>
```

### Commit Types

| Type | Use For |
|------|---------|
| `Fix` | Bug fixes |
| `Add` | New features |
| `Update` | Enhancements to existing features |
| `Refactor` | Code restructuring (no behavior change) |
| `WIP` | Work in progress (incomplete) |
| `Docs` | Documentation only |
| `Test` | Test additions/changes |
| `Chore` | Build system, dependencies, cleanup |

### Examples

```bash
# Bug fix
git commit -m "Fix MC-DPSK CFO correction for training samples

- CFO was not applied to training/reference samples
- Added applyCFO() wrapper that preserves cfo_hz_ after correction
- Chirp CFO is now trusted over training CFO

Fixes: BUG-001

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"

# New feature
git commit -m "Add OFDM_CHIRP support to test_iwaveform

- Implemented decodeOFDMChirpFrame() using IWaveform directly
- Fixed process() to loop and retrieve ALL soft bits
- Added --waveform ofdm_chirp option

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"

# Work in progress
git commit -m "WIP: Debug RxPipeline chirp detection issues

- Added detailed logging
- Issue: chirp detected but position incorrect
- TODO: Compare with test_iwaveform path"

# Documentation
git commit -m "Docs: Add comprehensive system documentation

- GUI_ARCHITECTURE.md
- AUDIO_SYSTEM.md
- BUILD_SYSTEM.md
- CONFIGURATION_SYSTEM.md
- ADDING_NEW_WAVEFORM.md"
```

---

## Pre-Commit Checklist

Before committing (especially non-WIP commits):

- [ ] Code compiles without errors
- [ ] `./tests/regression_matrix.sh` passes (for significant changes)
- [ ] If fixing a bug: entry added to `docs/CHANGELOG.md`
- [ ] If finding a bug: entry added to `docs/KNOWN_BUGS.md`
- [ ] If completing refactor task: `docs/REFACTOR_PROGRESS.md` updated

---

## Push Strategy

### When to Push

| Situation | Push? |
|-----------|-------|
| Completed feature/bugfix | Yes |
| End of work day | Yes (backup) |
| WIP commits only | Optional |
| Tests failing | No (fix first) |

### Before Pushing

```bash
# Check status
git status
git log --oneline origin/main..HEAD  # See unpushed commits

# Run regression tests
./tests/regression_matrix.sh

# Push if tests pass
git push
```

---

## Handling WIP Commits

### Option 1: Keep WIP Commits (Recommended for Research)

WIP commits provide history of thought process. Keep them as-is.

### Option 2: Squash Before Push

If you want clean history:

```bash
# Squash last 3 commits into one
git rebase -i HEAD~3
# Change 'pick' to 'squash' for commits to combine
```

### Option 3: Amend Last Commit

For small additions to most recent commit:

```bash
git add .
git commit --amend --no-edit
```

---

## Tagging Releases

When reaching a milestone:

```bash
# Create annotated tag
git tag -a v1.0.0 -m "First stable release

- MC-DPSK verified on fading channels
- OFDM_CHIRP CFO correction working
- Full protocol v2 implementation"

# Push tags
git push --tags
```

### Version Scheme

`vMAJOR.MINOR.PATCH`

- **MAJOR**: Breaking changes, major milestones
- **MINOR**: New features, significant improvements
- **PATCH**: Bug fixes, minor improvements

---

## Undoing Mistakes

### Undo Last Commit (keep changes)

```bash
git reset --soft HEAD~1
```

### Undo Last Commit (discard changes)

```bash
git reset --hard HEAD~1
```

### Revert a Pushed Commit

```bash
git revert <commit-hash>
git push
```

### Discard Uncommitted Changes

```bash
git checkout -- <file>        # Single file
git checkout -- .             # All files
```

---

## Common Workflows

### Starting New Work Session

```bash
git status                    # Check state
git log --oneline -5          # Review recent commits
cat docs/REFACTOR_PROGRESS.md # Check what's next
```

### Ending Work Session

```bash
git status                    # Check uncommitted changes
git add -A                    # Stage all changes
git commit -m "WIP: <description of current state>"
git push                      # Backup to remote
```

### After Fixing a Bug

```bash
# 1. Verify fix works
./tests/regression_matrix.sh

# 2. Update documentation
# Edit docs/CHANGELOG.md - add entry
# Edit docs/KNOWN_BUGS.md - move to Fixed section

# 3. Commit
git add -A
git commit -m "Fix <description>

<details>

Fixes: BUG-XXX

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>"

# 4. Push
git push
```

---

## Summary

| Aspect | Strategy |
|--------|----------|
| Branching | Trunk-based (main), feature branches optional |
| Commits | Early and often, WIP allowed |
| Messages | Type prefix, body explains why |
| Testing | Run regression before non-WIP commits |
| Pushing | When stable, at least daily for backup |
| Tagging | At milestones (vX.Y.Z) |
