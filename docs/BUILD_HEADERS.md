# Build Chat Headers (copy/paste)

## STANDARD BUILD CHAT HEADER
PROJECT: KILO .7 (ROS 2, Offline Autonomous Rover)
ROLE OF THIS CHAT: BUILD & IMPLEMENTATION (code/config allowed)
AUTHORITY DOCS: docs/PROJECT_CHARTER.md, docs/DECISIONS_LEDGER.md, docs/PROJECT_STATE.md

CURRENT PROJECT STATE (paste from PROJECT_STATE.md):
CURRENT PHASE:
CURRENT GOAL:
LAST CONFIRMED WORKING STATE:
WHAT IS BROKEN (KNOWN):
WHAT IS UNTESTED:
NEXT CONCRETE STEP:

SCOPE FOR THIS CHAT (non-negotiable)
IN SCOPE: tasks required to complete CURRENT GOAL; unblock current phase; observability needed to validate goal
OUT OF SCOPE: new features, new sensors, refactors not required, docking/mapping/nav outside current phase

DONE WHEN
- CURRENT GOAL met
- acceptance checks pass
- baseline is recoverable

SAFETY INVARIANTS
- loss of command → throttle neutral
- Safety Gate overrides all motion
- core control stable even if perception/nav fails
- relay kill independent of high-level logic

## EXPERIMENT MODE HEADER
ROLE OF THIS CHAT: EXPERIMENT / SPIKE
BASELINE MUST REMAIN RECOVERABLE

WHAT WE ARE TESTING:
WHY IT MATTERS:
WHAT SUCCESS LOOKS LIKE:
EXIT: graduate to change ticket OR document and discard

## HOTFIX HEADER
ROLE OF THIS CHAT: HOTFIX / RECOVERY
PRIORITY: restore LAST CONFIRMED WORKING STATE

WHAT BROKE:
WHEN:
LAST KNOWN GOOD:
CONSTRAINTS: minimal change only; no refactors; no new features; test immediately
