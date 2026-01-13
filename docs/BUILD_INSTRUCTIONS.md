# Build Chat Usage Instructions

1) Choose chat type: STANDARD vs EXPERIMENT vs HOTFIX.
2) Paste the correct header at the top of the chat.
3) Paste docs/PROJECT_STATE.md into the header.
4) Label intent:
   - EXPLAIN ONLY
   - RISK CHECK
   - REVIEW
   - CHANGE REQUEST
   - IMPLEMENT: (required to generate code)
5) Scope is law: only work that completes CURRENT GOAL.
6) End every session by updating:
   - docs/PROJECT_STATE.md
   - docs/CHANGELOG.md (if changed)
   - docs/LESSONS_LEARNED.md (if applicable)
7) If chat lags: start new chat, paste header + PROJECT_STATE.md, continue from NEXT CONCRETE STEP.
