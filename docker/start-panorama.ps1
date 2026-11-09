param(
    [int]$Width = 960,
    [int]$Height = 240,
    [int]$Fps = 15,
    [double]$Blend = 0.10,
    [int]$Vfov = 90,
    [string]$Config = "/data/local/tmp/kalibr_cam_chain_a4.yaml",
    [int]$Port = 8092
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$scripts = Join-Path $repo "calib\scripts"
$calib = Join-Path $repo "calib"

$running = docker inspect -f "{{.State.Running}}" seeker1 2>$null
if ($running -ne "true") {
    throw "容器 seeker1 未运行，请先启动容器。"
}

$devices = docker exec seeker1 adb devices
if (-not (($devices -join "`n") -match "\sdevice\s*$")) {
    throw "ADB 未连接。请先运行 .\docker\start-adb-host.cmd 并确认 docker exec seeker1 adb devices 有设备。"
}

$pushList = @(
    @{ Local = (Join-Path $scripts "start_panorama_final.sh"); Remote = "/data/local/tmp/start_panorama_final.sh" },
    @{ Local = (Join-Path $scripts "start_pano_q1.sh"); Remote = "/data/local/tmp/start_pano_q1.sh" },
    @{ Local = (Join-Path $scripts "panorama_q1.launch"); Remote = "/data/local/tmp/panorama_q1.launch" },
    @{ Local = (Join-Path $scripts "preview_dashboard.py"); Remote = "/data/local/tmp/preview_dashboard.py" },
    @{ Local = (Join-Path $scripts "run_preview_dashboard.sh"); Remote = "/data/local/tmp/run_preview_dashboard.sh" },
    @{ Local = (Join-Path $calib "kalibr_cam_chain_a4.yaml"); Remote = "/data/local/tmp/kalibr_cam_chain_a4.yaml" }
)
foreach ($item in $pushList) {
    if (-not (Test-Path -LiteralPath $item.Local)) {
        throw "缺少文件: $($item.Local)"
    }
    $name = Split-Path -Leaf $item.Local
    docker cp $item.Local "seeker1:/tmp/$name" | Out-Null
    docker exec seeker1 adb push "/tmp/$name" $item.Remote | Out-Null
}
docker exec seeker1 adb shell "sed -i 's/\r$//' /data/local/tmp/start_panorama_final.sh /data/local/tmp/start_pano_q1.sh /data/local/tmp/panorama_q1.launch /data/local/tmp/preview_dashboard.py /data/local/tmp/run_preview_dashboard.sh; chmod +x /data/local/tmp/start_panorama_final.sh /data/local/tmp/start_pano_q1.sh /data/local/tmp/run_preview_dashboard.sh /data/local/tmp/preview_dashboard.py"

$camRunning = docker exec seeker1 adb shell "pgrep -f test_4cam.launch || true"
if (-not ($camRunning -join "").Trim()) {
    Write-Host "启动相机桥接..."
    docker exec seeker1 adb shell "screen -wipe >/dev/null 2>&1; screen -dmS cam /data/local/tmp/start_cam.sh"
    Start-Sleep -Seconds 6
}

Write-Host "启动 method 4 全景: ${Width}x${Height}, ${Fps}fps, vfov=${Vfov}..."
docker exec seeker1 adb shell "bash /data/local/tmp/start_panorama_final.sh $Config $Width $Height $Fps $Blend $Vfov $Port" | Out-Null

docker exec seeker1 adb forward "tcp:$Port" "tcp:$Port" | Out-Null
$hostAdb = Get-Command adb -ErrorAction SilentlyContinue
if ($hostAdb) {
    & $hostAdb.Source forward "tcp:$Port" "tcp:$Port" | Out-Null
}

Write-Host "等待全景首帧..."
$ready = $false
for ($i = 0; $i -lt 120; $i++) {
    Start-Sleep -Seconds 1
    $raw = & curl.exe -s --max-time 2 "http://127.0.0.1:$Port/status.json" 2>$null
    if ($LASTEXITCODE -eq 0 -and $raw) {
        try {
            $status = ($raw -join "") | ConvertFrom-Json
            if ($status.pano.has_frame) {
                $ready = $true
                Write-Host ("全景已出图: {0}x{1}, {2} fps, 延迟 {3} ms, 节点 RSS {4} MB" -f `
                    $status.pano.width, $status.pano.height, $status.pano.fps, `
                    $status.pano.age_ms.last, $status.nodelet_rss_mb)
                break
            }
        } catch {
        }
    }
    if (($i % 15) -eq 14) { Write-Host ("  等待中... {0}s" -f ($i + 1)) }
}
if (-not $ready) {
    Write-Warning "180 秒内未收到全景帧。首次启动建投影表约需 1 分钟；若仍无图请查看: docker exec seeker1 adb shell 'cat /tmp/pano_eq.log'"
}
Write-Host "打开: http://localhost:$Port/"
Write-Host "停止: docker exec seeker1 adb shell 'screen -S pano_eq -X quit; screen -S dash -X quit'"