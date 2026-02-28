# Track: Z-Axis State Extension

**ID:** z-axis-states_20260228
**Type:** Feature
**Status:** Pending

## Documents

- [Specification](./spec.md)
- [Implementation Plan](./plan.md)

## Progress

- Phases: 0/4 complete
- Tasks: 0/22 complete

## Summary

Extend `StateVector` to 9D to include `z_ball`, `vz_ball`, and `z_table`. Add ImGui manual-state sliders (paused-only), update the 3D renderer to reflect all states, add "X"/"Y"/"Z" axis labels, and include new states in the real-time plotter. Physics, control, and estimation unchanged.

## Quick Links

- [Back to Tracks](../../tracks.md)
- [Product Context](../../product.md)
- [Tech Stack](../../tech-stack.md)
