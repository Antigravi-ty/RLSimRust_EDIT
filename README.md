# RLSimRust_EDIT

Pure-Rust WebAssembly physics core and C-ABI bridge for **RocketCompanion**, built on [`ZealanL/RocketSim` (`v3-rust` `v0.3.5`)](https://github.com/ZealanL/RocketSim/tree/v3-rust).

## Workspace Crates

- **`rocketsim/`**: Upstream `ZealanL/RocketSim` (`v0.3.5`) pure-Rust Bullet + Rocket League physics engine with WebAssembly compatibility patches:
  - Saturating float-to-int broadphase index conversions (`grid_broadphase.rs`, `axis_sweep.rs`) to avoid `wasm32` `i32.trunc_f32_s` traps on `StaticPlaneShape` AABBs.
  - Runtime `.cmf` collision mesh initialization and hot-reloading (`init_or_reload_soccar_from_slices`).
  - Per-wheel suspension/steering/contact telemetry (`Car::wheel_telemetry`).
  - Per-car linear factor & gravity scale (`Arena::set_car_linear_factor_and_gravity` + `CollisionFlags::NoWorldGravity`).
- **`rocketsim_wasm/`**: Zero-Emscripten `wasm32-unknown-unknown` C-ABI bridge (`cdylib`) exporting the complete 114 `_physics_*` + `_malloc` / `_free` interface and 448-float `GameStateBufferPod` shared linear memory layout consumed by `RocketCompanion/src/physics/`.

## Building `core.wasm`

```bash
rustup target add wasm32-unknown-unknown
CARGO_TARGET_DIR=/tmp/rlsim-target cargo build --target wasm32-unknown-unknown --release -p rocketsim_wasm
cp /tmp/rlsim-target/wasm32-unknown-unknown/release/rocketsim_wasm.wasm /path/to/RocketCompanion/src/physics/core.wasm
```
