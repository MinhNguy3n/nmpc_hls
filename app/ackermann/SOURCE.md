# Source of `app/ackermann/` and `docs/ackermann/`

Copied **unchanged** on 2026-11-21, for the Reely NEW1 RC car study (prompt
`.copilot/chats/2026-11-21-claude-to-claude-husky-agent-rc-car-prompt.md`, phase 0).

- **Source worktree:** `.claude/worktrees/agent-a3984fccbf29de483`
- **Source branch:** `ackermann-mpc-design` (head `8caa65c90bc37d9627d1b966b25cf273badede84`, 2026-10-25). The files were **untracked** there, so the commit doesn't contain them: the hashes below are the reference.
- **Copied with** `cp -p`. The hashes of the copies match the source:

```text
05711baa103b98baa2b2044d8660a563e28661f0cd6a6a776d6cd225178a412e  docs/ackermann/ackermann_mpc_pso_model.md
2deb03c2c96140ec34ae1847c9c7fe01efeeddddfbdf849b3f19003e2fd6507b  app/ackermann/ackermann_model.py
866c0bb60ebe89d534b199f25f368f4c8385f8d3f5b95c2d40dc4a9a44c1c7d3  app/ackermann/hw_estimates.py
bf1ef463758ee73ddd37ca628264a4d69487d8b7195041c7cfcb1da968f0206f  app/ackermann/test_ackermann_model.py
```

- **Test on import:** `/usr/bin/python3 -m pytest app/ackermann -q -p no:cacheprovider` gave **80 passed**.
- **The source worktree is read-only for this study.** Any change to the model is made here, and it is listed below with a reason.

## Changes since the copy

None.
