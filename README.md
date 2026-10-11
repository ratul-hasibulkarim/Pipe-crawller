# PIPEBOT-X

Self-locking, diameter-adaptive, 3-arm / 6-wheel in-pipe inspection robot.

Single source of truth: `params/bot_params.yaml` (header lists the design changes forced by the calcs).

    pip install -r calc/requirements.txt
    python3 calc/design_calcs.py     # prints every derived value, exits 1 if any check fails
    pytest tests/unit

Build status: step 1 of 9 (params + design calcs). CAD, firmware, Pi and app follow.
