# The Mystery of `postStartCommand` Not Keeping `rmw_zenohd` Alive

## Symptom

After the devcontainer starts, `rmw_zenohd` (the Zenoh router, started via `postStartCommand` in `.devcontainer/devcontainer.json`) is never actually running. Any `ros2` command — even a plain `ros2 node list` — fails with:

```
WARN  zenoh::net::runtime::orchestrator: Unable to connect to tcp/127.0.0.1:7447! ...
       Connection refused (os error 111) ...
ERROR zenohc::session: Error opening session: Unable to connect to any of [tcp/127.0.0.1:7447]! ...
failed to initialize rcl: caught C++ exception std::exception constructing rmw_context_impl_t: Error setting up zenoh session.
```

`postStartCommand` at the time was:

```jsonc
"postStartCommand": "bash -lc 'source /opt/ros/jazzy/setup.bash && nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/rmw_zenohd.log 2>&1 & disown'"
```

This looks like a standard "background a daemon and detach it" pattern. It doesn't work here, and diagnosing why turned out to be more interesting than expected.

## Hypothesis 1: "The router is just slow to start"

Reasonable first guess: `postStartCommand` returns as soon as the process is backgrounded, without waiting for `rmw_zenohd` to actually finish binding its listen socket. Maybe `ros2 node list` is just being run too early.

**Disproven:**
- Waiting 10+ seconds after container start didn't help.
- `ss -ltn | grep 7447` showed nothing listening.
- `ps aux | grep zenohd` showed no `rmw_zenohd` process at all — not slow, just absent.
- `cat /tmp/rmw_zenohd.log` was empty (0 bytes), even though the file existed with a timestamp matching container start.
- Running the exact same command manually in the foreground, `ros2 run rmw_zenoh_cpp rmw_zenohd`, works instantly and logs normal startup output (`Started Zenoh router with id ...`) in well under a second.

Conclusion: `rmw_zenohd` isn't slow — it's being started and then killed almost immediately, before it can even flush its first log line.

## Hypothesis 2: "`nohup ... & disown` doesn't fully detach the process; use `setsid`"

`postStartCommand` runs via a `docker exec`-style invocation from the Dev Containers CLI. `nohup` only protects a process from `SIGHUP`; it does not remove the process from the exec session's process group. The theory: when that exec session ends (right after the command backgrounds itself and the shell returns), something tears down the process group of the exec session, taking `rmw_zenohd` down with it. `disown` only affects the *current interactive shell's* job table bookkeeping and doesn't change this.

Fix attempted:

```jsonc
"postStartCommand": "bash -lc 'source /opt/ros/jazzy/setup.bash && setsid nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/rmw_zenohd.log 2>&1 < /dev/null &'"
```

`setsid` creates a brand new session for the child process, fully detaching it from whatever session/process-group the exec invocation belongs to.

**Disproven, decisively:**
- After a full "Rebuild Container" (confirmed via the Dev Containers startup log that the *new* command text was actually what ran), the result was the same: no `rmw_zenohd` process, and this time not even an empty log file — meaning the backgrounded job's I/O redirection was never even set up.
- The critical test: running the **exact same `bash -lc '...'` string, verbatim**, by hand in a normal interactive terminal works perfectly. `rmw_zenohd` and its child process show up in `ps aux`, and the log has normal startup output. Closing that terminal and opening a new one — the process is still alive.

This ruled out the shell syntax entirely. The command, the `setsid`/`nohup`/redirection semantics, and the container's environment are all fine — proven by the fact that the identical string works when typed at a prompt. The only variable that changed was *how the command got executed*: via `postStartCommand`'s lifecycle-hook machinery vs. an ordinary interactive terminal session.

Conclusion: whatever `postStartCommand` (and by extension, `postCreateCommand` / `postAttachCommand` — they use the same underlying mechanism) does to run its command, it also cleans up something after the command line returns that an interactive terminal's `docker exec` session does not clean up. No shell-level detachment trick inside `postStartCommand` can be relied on to survive this, because the problem isn't about sessions/process groups at all — it's specific to the lifecycle-hook execution path itself.

## Current status

Unresolved. `postStartCommand` has been removed from `devcontainer.json` entirely (rather than left in its broken state), and no automatic startup mechanism for `rmw_zenohd` is currently configured. 
