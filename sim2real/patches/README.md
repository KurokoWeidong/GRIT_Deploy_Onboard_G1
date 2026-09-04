# Unitree SDK2 `ReadLatest` patch

`unitree_sdk2-read-latest.patch` records the five header changes used by the
GRIT onboard bridge relative to the official Unitree SDK2 commit:

```text
1a16684f32fb75dfd9c0aace012fc8126fb9794e
```

Upstream repository:

```text
https://github.com/unitreerobotics/unitree_sdk2.git
```

The patch adds a callback-free DDS reader and a `ReadLatest()` path used by the
bridge's timer-driven low-state publishing mode. It does not contain Unitree
SDK2 binaries or a full SDK copy. Unitree SDK2 is distributed under the
BSD-3-Clause license; retain the upstream `LICENSE` when redistributing a
patched SDK checkout.

## Apply

From the root of a clean Unitree SDK2 checkout at the commit above:

```bash
git apply --check /path/to/unitree_sdk2-read-latest.patch
git apply /path/to/unitree_sdk2-read-latest.patch
```

## Verify

```bash
git diff --check
git diff -- include/unitree/common/dds/dds_entity.hpp \
  include/unitree/common/dds/dds_factory_model.hpp \
  include/unitree/common/dds/dds_topic_channel.hpp \
  include/unitree/robot/channel/channel_factory.hpp \
  include/unitree/robot/channel/channel_subscriber.hpp
```

The patch must be applied before building `g1_udp_bridge` when the bridge uses
`freq.state_publish_mode: timer`.
