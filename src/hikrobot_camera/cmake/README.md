# SDK 构建集成

主 `CMakeLists.txt` 使用 `MVS_ROOT` 变量查找 MVS SDK，默认值为 `/opt/MVS`。

在安装到其他路径的环境中，可以覆盖：

```bash
colcon build --cmake-args -DMVS_ROOT=/path/to/MVS
```
