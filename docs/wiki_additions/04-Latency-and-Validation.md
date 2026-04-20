# Latency and Validation

## Runtime latency metrics
The live pipeline exposes:
- `vision_us`
- `unsafe_detect_ms`
- `freeze_pipeline_ms`
- `freeze_cmd_ms`
- `total_stop_ms`
- `ack_to_resume_ms`

## Guardian thresholds (current live baseline)
- freeze after `15` bad frames
- recover after `3` good frames

## What these metrics mean
- `vision_us`: per-frame vision processing cost
- `unsafe_detect_ms`: first unsafe frame capture -> unsafe decision
- `freeze_pipeline_ms`: unsafe decision -> freeze path callback
- `freeze_cmd_ms`: callback entry -> stop command issue
- `total_stop_ms`: first unsafe frame capture -> stop command issue
- `ack_to_resume_ms`: operator acknowledge -> resume command issue

## Validation commands
```bash
./scripts/camera_backend_check.sh
./scripts/live_test.sh
ctest --test-dir build --output-on-failure
```

## Validation checklist
1. backend check passes with no frame capture failures
2. live mode arms only when safe
3. unsafe event triggers retract + freeze
4. resume requires operator action
5. manual mode still obeys guardian/interlock safety
