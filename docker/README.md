# Seeker1 Docker 运行说明

> 全景原理、性能与开关命令见 [docs/全景拼接与运行说明.md](../docs/全景拼接与运行说明.md)。

## 构建和启动

```powershell
docker compose up -d --build
```

容器启动后会自动运行 `roscore`。进入容器：

```powershell
docker exec -it seeker1 bash
```

## 当前 RK3588S-4P 环境推荐：主机 ADB

当前 Docker Desktop 的 `docker-desktop` WSL 发行版没有 bash，`usbipd 5.3` 附加 USB 会失败；同时 `attach-camera.ps1` 会把 `usbipd bind` 的 "already shared" stderr 当成致命错误。推荐不要再把 USB 附加到 WSL，而是让 Windows 主机直接持有 ADB：

```powershell
cd F:\Code\Seeker1
.\docker\start-adb-host.cmd
docker exec seeker1 adb devices
```

容器里的 `adb` 已安装为代理，会自动连接 `host.docker.internal:5037` 的主机 adb server。若容器被重建后代理失效，重新安装：

```powershell
docker cp F:\Code\Seeker1\calib\scripts\install_adb_proxy.py seeker1:/tmp/
docker exec seeker1 python3 /tmp/install_adb_proxy.py
```

板端 ROS 启动：

```powershell
docker exec seeker1 adb shell 'bash -lc "screen -wipe >/dev/null 2>&1; screen -dmS cam /data/local/tmp/start_cam.sh; screen -dmS imu /data/local/tmp/start_imu.sh"'
```

如果之前运行过 `attach-camera.cmd`，先执行 `usbipd detach --busid 4-2`，再运行 `start-adb-host.cmd`。下面的 usbipd 流程仅适用于能正常附加到 `docker-desktop` 的环境。
## 相机 USB 转发

本项目通过 libusb 直接访问相机，USB 设备 ID 为 `2207:0000`。Windows 上的 Docker Desktop 需要先通过 `usbipd-win` 把设备转发到 Docker Desktop 的 WSL2 发行版。

相机插上后，以管理员身份打开 PowerShell，执行仓库中的一键脚本：

```powershell
.\docker\attach-camera.cmd
```

脚本会自动查找 `2207:0000`、绑定设备、转发到 `docker-desktop` 并重启容器。`.cmd` 入口会绕过 Windows PowerShell 的脚本执行策略限制。

也可以手工执行：

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl docker-desktop --busid <BUSID>
docker restart seeker1
```

确认容器能看到设备：

```powershell
docker exec seeker1 lsusb
```

## 启动相机和看图

```powershell
docker exec -it seeker1 seeker-camera
```

另开一个 PowerShell 启动网页看图服务：

```powershell
docker exec -d seeker1 seeker-web-view
Start-Process http://localhost:8080
```

常用话题包括 `/fisheye/left/image_raw`、`/fisheye/right/image_raw`、`/fisheye/bright/image_raw`、`/fisheye/bleft/image_raw`。

## 读取标定参数

相机正常连接后执行：

```powershell
docker exec -it seeker1 seeker-calibrate
```

该命令会把 `/tmp/kalibr_cam_chain.yaml` 复制到
`config/seeker_omni_depth/kalibr_cam_chain.yaml`，供拼接、去畸变、深度图和点云 launch 使用。

## 实时全景 + 四宫格 + 延迟（method 4）

前置：相机已通过 ADB 连接，容器运行中。

```powershell
cd F:\Code\Seeker1
.\docker\start-panorama.cmd
```

打开 <http://localhost:8092/> 。

- 上方：360°×90° 等距柱面全景（默认 960×240）
- 下方：四路原始鱼眼 2×2
- 卡片：全景延迟、处理滞后、全景刷新率、节点 RSS、板端可用内存
- 表格：四路原始流各自的延迟、最小/平均/最大和帧率

自定义参数：

```powershell
.\docker\start-panorama.ps1 -Width 1280 -Height 320 -Fps 15 -Vfov 90 -Blend 0.10
```

停止：

```powershell
docker exec seeker1 adb shell 'screen -S pano_eq -X quit; screen -S dash -X quit'
```

查看日志和内存：

```powershell
docker exec seeker1 adb shell 'tail -n 40 /tmp/pano_eq.log; ps -eo pid,rss,pcpu,args | grep pano_manager | grep -v grep'
```

投影模式说明：

- `method=4`：新增的等距柱面全景（360°×可配垂直视场），适合四目水平环，上下只有极窄的天顶/天底盲区。
- `method=3`：厂商球极投影，会包含天顶/天底方向，四目水平环会出现两个明显的黑色盲区。
- `method=0`：四路竖直条带简单拼接。

内存修复说明：节点改为“每路只保存最新一帧 + 定时器单线程处理”，融合使用固定大小的 8U/16U 缓冲，输入回调不再并发写同一组 Mat，也不再因处理慢而积压。实测连续 4 分钟 RSS 在 144–184 MB 间波动（均值约 170 MB），没有持续增长；全景刷新约 9.5–10 FPS，端到端延迟约 170–230 ms（平均约 200 ms），处理滞后约 135 ms。
