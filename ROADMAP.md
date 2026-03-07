# openblack Roadmap

This document outlines planned performance enhancements and quality-of-life (QoL)
upgrades needed to reach full parity with the original Black & White (2001) game.
Items are grouped by category and roughly ordered by priority within each section.

---

## Performance Enhancements

### Rendering

- [ ] **Instanced rendering for foliage and static meshes** – Batch draw calls for
  trees, rocks, and other repeated static objects using hardware instancing to
  reduce per-draw-call overhead.
- [ ] **GPU-driven occlusion culling** – Replace or supplement CPU-side frustum
  culling with GPU occlusion queries so that off-screen geometry does not
  contribute to vertex shader work.
- [ ] **Level-of-detail (LOD) system for L3D meshes** – Introduce multiple mesh
  resolutions for creatures, villagers, and landscape features and switch between
  them based on camera distance.
- [ ] **Texture atlas and sprite batching** – Pack small 2-D sprites (status
  icons, particles) into a texture atlas so the sprite pass issues fewer draw
  calls.
- [ ] **Asynchronous shader compilation** – Move bgfx shader compilation and
  program linking off the main thread to reduce startup stalls.
- [ ] **Shadow map caching** – Cache static shadow maps for the island terrain and
  stationary objects between frames; only update dynamic shadow casters every
  frame.
- [ ] **Water reflection optimisation** – Render the reflection pass at a reduced
  resolution and reconstruct at full resolution with an upscaling filter.
- [ ] **Multi-threaded rendering command encoding** – Leverage bgfx's multi-thread
  submission API to encode draw commands in parallel across CPU threads.

### Physics & Dynamics

- [ ] **Broad-phase spatial hashing** – Replace or augment the current dynamics
  broad-phase with a spatial hash grid for faster collision-pair generation on
  the island.
- [ ] **Fixed-step physics with interpolation** – Decouple the physics tick from
  the render tick; run physics at a fixed frequency and interpolate entity
  transforms for smooth visual output at any frame rate.
- [ ] **Async physics stepping** – Move `DynamicsSystem` update onto a worker
  thread and synchronise only the result data needed for rendering.

### Pathfinding

- [ ] **Hierarchical pathfinding (HPA*)** – Layer a coarse abstract graph over
  the island navigation mesh so that long-range paths are found cheaply and
  refined on demand near waypoints.
- [ ] **Path request queuing with time-slicing** – Spread expensive A* queries
  across multiple frames to avoid per-frame spikes in `PathfindingSystem`.
- [ ] **Navigation mesh caching** – Pre-bake the island navigation mesh at load
  time and store it alongside the land data so it does not need to be
  recomputed on startup.

### Entity / ECS

- [ ] **Component storage contiguity** – Ensure hot ECS components (`Transform`,
  `Mesh`, `Velocity`, `RigidBody`) are stored in contiguous, cache-friendly
  arrays to improve iteration throughput.
- [ ] **Parallel system updates** – Identify independent ECS systems
  (`LivingActionSystem`, `TownSystem`, `CameraBookmarkSystem`) and run them
  concurrently using a task graph.
- [ ] **Archetype-based registry** – Investigate migrating from the current
  `entt`-backed registry to an archetype layout to improve cache locality for
  large entity populations.

### Audio

- [ ] **Streaming audio decoding** – Decode large background music and ambient
  tracks incrementally on a dedicated audio thread rather than loading the
  entire file before playback.
- [ ] **Audio occlusion / propagation** – Skip 3-D distance attenuation
  calculations for emitters outside the audible radius early to reduce the cost
  of `AudioManager` per-frame updates.

### General

- [ ] **Profiler-guided hotspot analysis** – Expand the existing `Profiler` to
  export per-frame timings in a machine-readable format (e.g. Chrome trace JSON)
  so bottlenecks can be identified systematically.
- [ ] **Link-time optimisation (LTO)** – Enable LTO for release builds to allow
  cross-translation-unit inlining of frequently called engine utilities.
- [ ] **Platform-specific SIMD paths** – Use SIMD intrinsics (SSE4 / NEON) for
  bulk vector-math operations in pathfinding and physics integration.

---

## Quality-of-Life Upgrades

### Gameplay Parity

- [ ] **Creature AI and learning** – Fully implement the creature belief and
  learning system so that creature behaviour responds correctly to player
  discipline and miracles over time.
- [ ] **Villager needs and daily schedules** – Complete villager routine
  simulation (eating, sleeping, praying, working) to match original game pacing.
- [ ] **Town resource production** – Finish `TownSystem` resource accumulation,
  food storage and tribute mechanics so towns behave as in the original.
- [ ] **Miracle system** – Implement all remaining miracles (water, storm, shield,
  etc.) including cast animations, particle effects, and game-world impact.
- [ ] **Scaffold and building construction** – Animate the multi-stage building
  construction sequence with villager labour and resource consumption.
- [ ] **Influence sphere and spiritual power** – Wire up the god-power / influence
  accumulation system so that worship generates correct amounts of power.
- [ ] **Challenge (scroll) system** – Load and execute in-game challenge scripts
  so that land objectives are presented and tracked as in the original.
- [ ] **Creature pen and leash mechanics** – Implement the creature pen boundary
  enforcement and leash-rope physics.

### Controls & Input

- [ ] **Hand grab and throw physics** – Implement realistic pick-up, carry, and
  throw mechanics for villagers, animals, trees, and boulders.
- [ ] **Gesture recognition** – Add support for mouse-gesture-driven miracle
  casting (circular gesture for fireball, etc.).
- [ ] **Camera keyboard shortcuts** – Honour all original hotkeys (bookmark
  creation/recall, quick-focus on creature, zoom-to-event).
- [ ] **Controller support** – Add gamepad bindings for movement, hand actions,
  and common shortcuts.
- [ ] **Variable scroll speed** – Scale camera pan and zoom speed with the current
  altitude above terrain.

### User Interface

- [ ] **In-game help / advisor system** – Integrate the conscience advisor speech
  and help text triggered by game events.
- [ ] **HUD transparency and scaling** – Allow the player to resize and reposition
  HUD elements and adjust their opacity.
- [ ] **Save/load UI** – Build a save-game browser with thumbnails and timestamps
  accessible from the in-game menu.
- [ ] **Options menu** – Provide in-game settings for graphics quality, audio
  volumes, key bindings, and mouse sensitivity without requiring restart.
- [ ] **Accessibility options** – Add subtitles for advisor speech, colourblind
  mode for influence indicators, and adjustable text sizes.

### Audio & Video

- [ ] **Cutscene playback** – Implement in-engine playback of pre-rendered
  cutscenes and in-engine scripted sequences at land transitions.
- [ ] **Ambient soundscape** – Trigger region-specific ambient audio (wind,
  ocean, birds) based on camera position on the island.
- [ ] **Music system** – Play the original music tracks with correct looping and
  cross-fade transitions based on game state (combat, peaceful, night).
- [ ] **Creature vocalisations** – Hook creature emotion states to the
  appropriate sound-group playback so creatures grunt, laugh, and cry at the
  right moments.

### Developer & Modding Quality of Life

- [ ] **In-game debug overlay** – Surface the existing `Profiler` data in an
  always-available debug window toggled by a hotkey, showing frame times per
  system, draw call count, and entity population.
- [ ] **Live asset hot-reload** – Watch shader and texture directories for changes
  and reload without restarting the engine.
- [ ] **Script console** – Expose a runtime LHVM/CHL console for issuing script
  commands, inspecting variables, and running challenge scripts interactively.
- [ ] **Map editor tooling** – Extend existing land/model tools (`l3dtool`,
  `lndtool`) with a graphical preview mode to facilitate custom map creation.
- [ ] **Comprehensive test coverage** – Expand unit and integration tests for
  ECS systems, file parsers, and scripting VM to guard against regressions.

---

## Milestones

| Milestone | Focus | Target State |
|-----------|-------|-------------|
| **M1 – Stable Foundation** | Engine stability, save/load, basic gameplay loop | Island explorable, creature follows player |
| **M2 – Gameplay Parity α** | Core mechanics matching original (villagers, miracles, creature AI) | Land 1 completable |
| **M3 – Performance Baseline** | Instanced rendering, fixed-step physics, LOD | 60 fps on mid-range hardware |
| **M4 – Full QoL** | Complete UI, audio, cutscenes, controller support | Feature-complete single-player experience |
| **M5 – Polish & Modding** | Hot-reload, script console, map editor, comprehensive tests | Moddable and contributor-friendly |

---

Contributions toward any item above are welcome. See [CONTRIBUTING](.github/contributing.md)
for guidelines on how to get involved.
