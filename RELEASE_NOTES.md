# Release Notes

## [2.24.0] - 2026-08-10

-   Add: TM3S is now selectable in the robot Model dropdown
-   Add: `_move_to_target()` can now rotate a prim as well as move it, writing to whichever rotation op the asset actually uses
-   Fix: Virtual arm jitter at certain poses while mirroring Ethernet Slave motion. The smoothing holds its timing whatever frame rate the scene runs at
-   Fix: Cameras were never released on Stop, so render products piled up across repeated Start/Stop cycles
-   Fix: `_move_to_target()` never reported completion when the step size did not divide the remaining distance evenly, leaving the prim oscillating around its target
-   Fix: `_set_prim_size()` raised an error on a prim that does not exist or has zero-sized bounds
-   Fix: The surface gripper stayed closed when the service was stopped mid-grip, leaving a dangling joint that could stop the next run from picking anything up
-   Fix: "Articulation needs to be initialized" warnings on every physics step just after Start
-   Fix: `omni.hydra` primvar warnings flooding the console

## [2.23.2] - 2025-06-18

-   Fix: EIH/EXT01/EXT02 cameras may display the original camera object after the camera is activated [0024618]
-   Fix: Added a new Robot. When setting EIH in Activated Camera, there is a chance of error [0024619]
-   Add: Case 6: Example of Prismatic Joint Gripper

## [2.23.1] - 2025-04-25

-   Upgrade to support Isaac Sim 4.5.
