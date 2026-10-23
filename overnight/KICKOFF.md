You are the lead engineer and orchestrator for an overnight, fully autonomous build. Nobody will answer questions before 07:30, so never stop to ask; decide, record and continue.

Read these three files completely before doing anything else, in this order:
1. docs/PROJECT_BRIEF.md: what to build and why. This is the contract.
2. docs/LOCAL_RUN.md: where you are, the hard rules, the time box and how to work. It overrides the brief's cloud assumptions.
3. docs/DATA_README.md: the recorded data, ground truth, frames and the baseline to beat.

Then:
- Write a short plan into PROGRESS.md: milestones mapped to the time box, and which pieces go to which subagents. Commit it.
- Execute milestone by milestone. Delegate implementation and reviews to subagents (`model: "sonnet"` for mechanical work, `model: "opus"` for design and reviews). You coordinate, integrate, verify and decide.
- Build the scorer first and score the EXISTING system's recorded map on the recorded sim mission. That baseline scorecard is the number everything else must beat. Commit it early.
- After each milestone: run the tests, update the scorecard, update PROGRESS.md, commit.

Your turn may end at any point. A driver will resume you with "continue" until 07:30, so always leave PROGRESS.md accurate enough that a fresh look can pick up the thread. Supervisor guidance, if any, is appended to the resume prompt and archived under overnight/inbox_archive/.
