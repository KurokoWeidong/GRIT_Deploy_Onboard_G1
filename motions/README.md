# Trajectory directory / 轨迹目录

This package contains five example trajectories:

- `boxing1.npz`
- `boxing2.npz`
- `boxing3.npz`
- `kick1.npz`
- `kpopdance.npz`

本项目包含以上 5 条示例轨迹。也可以把其他经过审核的 `.npz` 文件放入
此目录，或通过 `--motion-file` 指定其他文件或目录。

For a directory playlist, files are discovered non-recursively and sorted by
filename. The required NPZ fields and shapes are documented in the root
README files.

使用文件夹播放列表时，程序只扫描当前层，并按照文件名排序。NPZ 必需字段和
数组形状见项目根目录的中英文 README。
