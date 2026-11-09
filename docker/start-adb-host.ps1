param([string]$AdbPath)
$ErrorActionPreference = "Stop"

if (-not $AdbPath) {
    $candidates = @()
    $cmd = Get-Command adb -ErrorAction SilentlyContinue
    if ($cmd) { $candidates += $cmd.Source }
    $candidates += "F:\Softwares\scrcpy-win64-v2.7\adb.exe"
    $candidates += (Join-Path $env:LOCALAPPDATA "Android\Sdk\platform-tools\adb.exe")
    $candidates += "C:\Android\platform-tools\adb.exe"
    $AdbPath = $candidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
}

if (-not $AdbPath) {
    throw "adb.exe not found. Install Android platform-tools or pass -AdbPath."
}

Write-Host "Using adb: $AdbPath"
& $AdbPath kill-server 2>$null | Out-Null
Start-Sleep -Seconds 1
Start-Process -FilePath $AdbPath -ArgumentList "-a", "nodaemon", "server" -WindowStyle Hidden
Start-Sleep -Seconds 2

$listen = netstat -ano | Select-String "0.0.0.0:5037"
if (-not $listen) {
    throw "adb server is not listening on 0.0.0.0:5037"
}

Write-Host "adb server listening on 0.0.0.0:5037"
& $AdbPath devices
$ports = @(8090, 8091, 8092)
foreach ($port in $ports) {
    $prevEap = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    & $AdbPath forward --remove "tcp:$port" 2>&1 | Out-Null
    $ErrorActionPreference = $prevEap
    & $AdbPath forward "tcp:$port" "tcp:$port" | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "adb forward tcp:$port failed"
    }
}
Write-Host "preview forwarding: 8090 (4cam) / 8091 (panorama) / 8092 (dashboard)"


