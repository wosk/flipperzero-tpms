# Repository Guidelines

## Project Structure & Module Organization
The Flipper Zero TPMS app is written in C and keeps its entry point in `tpms_app.c`, with the public interface in `tpms_app_i.h`. Interaction flows are assembled in `scenes/`, rendering logic in `views/`, and protocol decoders in `protocols/`. Shared helpers for radio presets and event data sit in `helpers/`, while deployable assets live under `images/`. `application.fam` defines the build metadata (appid `tpms`, category Sub-GHz); keep it in sync whenever you expose new scenes or resources.

## Build, Test, and Development Commands
- `./fbt fap_tpms` (run from a cloned `flipperzero-firmware` tree with this repo under `applications_user/tpms`) builds the FAP and drops it into `build/f7-firmware-D/apps`.
- `./fbt launch_app APPSRC=flipperzero-tpms` launches the debug build on a connected Flipper or emulator for quick iteration.
- `./fbt fap_dist APPSRC=flipperzero-tpms` creates a distributable `tpms.fap` archive in `dist/fap` for release builds.

## Coding Style & Naming Conventions
- Follow the established 4-space indentation, K&R braces, and `snake_case` function/file names prefixed with `tpms_`; reserve `PascalCase` for structs and enums such as `TPMSApp`.
- Group system/Furi includes before project headers, keep them alphabetized, and prefer `static` helpers when the symbol is local to a file.
- Document non-obvious radio timing or preset choices with a short comment; favor existing logging patterns over new macros.

## Testing Guidelines
There is no automated suite yet, so rely on hardware verification. After flashing via `./fbt fap_tpms` and loading the package with qFlipper, confirm menu navigation, sensor discovery, and relearn timing against supported sensors. Record manual smoke results in the pull request, including firmware hash and sensor IDs. When adding protocol decoders, attach concise RF logs or test notes that let reviewers understand the scenario you validated.

For receiver validation, do not stop at the first successful decode. Confirm repeated reception of an already-known sensor ID and verify that `Auto` behaves correctly on hardware without an external CC1101 attached.

## Commit & Pull Request Guidelines
Recent history favors concise, imperative subjects (e.g. `Update tpms for latest firmware version`); keep them ≤72 characters and scope each commit to one logical change. For pull requests, provide a short summary, testing notes, related issues/discussions, and screenshots or GIFs when UI states change. Request maintainer review prior to merge and ensure you build against the same `flipperzero-firmware` revision referenced in the PR description.

## Configuration Tips
Add new sensors by extending `protocols/` and documenting any radio preset changes directly in the decoder code so testers can replicate results. Re-run `./fbt fap_tpms` after asset changes to verify the `tpms_10px.png` icon and other resources remain within Flipper size/format limits.

## Path Hygiene
Prefer repo-relative paths in docs, scripts, comments, and answers unless an absolute path is required for a device-specific or host-specific operation. Avoid centering instructions around local workspace paths when a repository-relative path communicates the same thing.

Do not leave `picocom`, `minicom`, or similar serial sessions open after a check. Close them before launches, uploads, or additional device verification so the USB CDC path does not get wedged by your own session.
