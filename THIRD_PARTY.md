# Third-party software, models, and acknowledgements

The root `LICENSE` applies only to code for which the GRIT authors hold the
necessary rights. The components and model files below remain subject to their
respective upstream terms. Public availability of a repository is not, by
itself, permission to redistribute its contents.

## Models

| File | Upstream source | SHA-256 in this package | Redistribution note |
| --- | --- | --- | --- |
| `sim2real/checkpoints/policy.onnx` | [mrzuang/GRIT_teleop_deploy policy.onnx](https://github.com/mrzuang/GRIT_teleop_deploy/blob/main/sim2real/checkpoints/policy.onnx), local reference commit `1477b56e2e792f63a8636a7c7858cb086ebc8b6d` | `39d573e970b7088275ed68ec8a9597fa9c402d98d5ad211991db7d8d0f5a82b9` | Permission to redistribute this model with this project has been confirmed. The upstream terms continue to apply. |
| `sim2real/checkpoints/Unitree-G1-AMP-Flat_model_30000.onnx` | [ccrpRepo/wbc_fsm](https://github.com/ccrpRepo/wbc_fsm/tree/main), local reference commit `b352409d73bed469169334040ee9ea70bc28a5f1` | `36cfc5d2a4ad8e07621a02f925d47f6a7cca4aa4e8da36d8116d07217b55f0d6` | The inspected upstream tree does not contain a conventional repository-level license file. Explicit permission to redistribute this model with this project has been confirmed. |

Redistribution permission for both model files included in this project has
been confirmed. The project maintainer should retain the corresponding
authorization records.

## Design acknowledgement

The workstation-side PICO retargeting implementation and configuration are
adapted from
[mrzuang/GRIT_teleop_deploy](https://github.com/mrzuang/GRIT_teleop_deploy)
(local reference commit `1477b56e2e792f63a8636a7c7858cb086ebc8b6d`).

The native bridge and sim-to-real structure were developed with reference to
the `sim2real` branch of
[Axellwppr/motion_tracking](https://github.com/Axellwppr/motion_tracking/tree/sim2real)
(local reference commit `0d5ba31e33397f3543d350d98b637e26d92f470a`).
That upstream project is licensed under the MIT License in the inspected
snapshot. Thank you to its authors and contributors.

## Bundled dependencies

Pinned source revisions are recorded in
`g1_sim2real/third_party/VERSIONS.md`.

| Component | Upstream | Bundled license/notices |
| --- | --- | --- |
| Unitree SDK2 | https://github.com/unitreerobotics/unitree_sdk2 | `g1_sim2real/third_party/unitree_sdk2/LICENSE` and its `licenses/` directory |
| yaml-cpp | https://github.com/jbeder/yaml-cpp | `g1_sim2real/third_party/yaml-cpp/LICENSE` |
| zlib | https://github.com/madler/zlib | `g1_sim2real/third_party/zlib/LICENSE` |
| ONNX Runtime 1.23.2 (ARM64) | https://github.com/microsoft/onnxruntime | `onnxruntime/LICENSE` and `onnxruntime/ThirdPartyNotices.txt` |
| ZeroMQ/libzmq | https://github.com/zeromq/libzmq | System dependency; MPL-2.0 |

The Unitree SDK2 snapshot includes Cyclone DDS and other third-party materials;
retain its nested license files when redistributing this package.

## External VR components

VR operation also depends on software that is not bundled here:

- [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)
- [XRoboToolkit Python binding](https://github.com/Axellwppr/XRoboToolkit-PC-Service-Pybind)

Review and comply with their licenses separately.
