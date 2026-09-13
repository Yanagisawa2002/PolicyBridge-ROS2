# Finalization and observation repair — 2026-09-13

Diagnostic publishing is now best effort at callback, timer and terminal boundaries. A failed diagnostics transport cannot prevent the winning Action terminal transition, replace its returned result, or leave admission occupied. The wrapper also contains a logging failure during shutdown.

ObservationStore now requires both a newer sequence and an unexpired synchronized-snapshot receipt. Fresh independent image/joint arrivals cannot renew an old synchronized value. The synchronized observation timeout is the TTL; expiry is checked at consumption and synchronization-fault classification.

Run `python -m pytest policy_bridge/test`. This revision passed 206 CPU tests on Windows; three ROS launch modules were skipped because rclpy/ROS 2 is unavailable. Four injected diagnostics cases cover startup, terminal publication, final cleanup and persistent publisher failure using production method bodies with transport doubles. These are not ROS executor or physical robot tests.

The README's 204-test Humble record and four demo episodes belong to the earlier recorded Ubuntu/scripted/mock environment. The pytest launch collector count is not a count of 204 independent robot acceptance assertions. Full Humble and hardware validation of this repair remains pending.
