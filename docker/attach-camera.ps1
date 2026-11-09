param(
    [string]$Distribution = "docker-desktop",
    [switch]$SkipRestart
)

$ErrorActionPreference = "Stop"

$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = [Security.Principal.WindowsPrincipal]::new($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "请以管理员身份运行此脚本。"
}

$usbipdCommand = Get-Command usbipd -ErrorAction SilentlyContinue
if ($usbipdCommand) {
    $usbipdExe = $usbipdCommand.Source
} else {
    $usbipdCandidates = @(
        (Join-Path $env:ProgramFiles "usbipd-win\usbipd.exe"),
        "C:\Program Files\usbipd-win\usbipd.exe"
    )
    $usbipdExe = $usbipdCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
}

if (-not $usbipdExe) {
    throw "未找到 usbipd，请先执行: winget install --id dorssel.usbipd-win -e"
}

$rows = & $usbipdExe list
$row = $rows | Where-Object { $_ -match '\s2207:0000(\s|$)' } | Select-Object -First 1
if (-not $row) {
    throw "未找到 Seeker 相机 (2207:0000)，请确认 USB 已插好。"
}

$busid = ($row -split '\s+')[0]
Write-Host "Seeker busid: $busid"

$prevEap = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$bindOutput = & $usbipdExe bind --busid $busid 2>&1
$bindExit = $LASTEXITCODE
$ErrorActionPreference = $prevEap
if ($bindExit -ne 0 -and (($bindOutput -join ' ') -notmatch 'already')) {
    throw "usbipd bind 失败: $bindOutput"
}

$prevEap = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$attachOutput = & $usbipdExe attach --wsl $Distribution --busid $busid 2>&1
$attachExit = $LASTEXITCODE
$ErrorActionPreference = $prevEap
if ($attachExit -ne 0) {
    throw "usbipd attach 失败。可先执行 usbipd attach --help 检查参数。"
}

if (-not $SkipRestart) {
    docker restart seeker1 | Out-Null
    Start-Sleep -Seconds 3
}

$lsusb = docker exec seeker1 lsusb 2>&1
$lsusb
if (($lsusb -join "`n") -notmatch '2207:0000') {
    Write-Warning "容器没有看到 2207:0000。当前环境请改用 .\docker\start-adb-host.cmd（主机 ADB 方案）。"
}
