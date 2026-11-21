# PowerShell Quick Connect Launcher & Sync
$uploadScript = Join-Path $PSScriptRoot "upload.ps1"
$ipFile = Join-Path $PSScriptRoot "robot_ip.txt"
if (-not (Test-Path $ipFile)) {
    $altIpFile = Join-Path (Split-Path -Parent $PSScriptRoot) "docs\robot_ip.txt"
    if (Test-Path $altIpFile) { $ipFile = $altIpFile }
}

if (Test-Path $ipFile) {
    $IP = (Get-Content -Path $ipFile).Trim()
} else {
    $IP = "10.24.170.0"
}

Clear-Host
Write-Host "==============================================================" -ForegroundColor Cyan
Write-Host "           SPARK ROBOT: QUICK CONNECT & CONTROL               " -ForegroundColor Yellow
Write-Host "==============================================================" -ForegroundColor Cyan
Write-Host "Current Robot IP: $IP" -ForegroundColor Green
Write-Host "Robot password (if prompted): simple2026`n" -ForegroundColor Gray

$userInput = Read-Host "Press [Enter] to start (with auto-sync) or enter new IP"

if ($userInput -and $userInput.Trim() -ne "") {
    $IP = $userInput.Trim()
    Set-Content -Path $ipFile -Value $IP
    Write-Host ">> New IP saved to robot_ip.txt: $IP" -ForegroundColor Yellow
}

# 1. Automatic synchronization of all updated files to robot
Write-Host "`n>> [1/2] Synchronizing updated scripts to robot ($IP)..." -ForegroundColor Yellow
& $uploadScript -NewIP $IP

if ($LASTEXITCODE -ne 0) {
    Write-Host "`n[WARNING] Synchronization failed or host unreachable. Check WiFi / IP." -ForegroundColor Red
} else {
    Write-Host ">> [OK] All scripts on robot are up to date!" -ForegroundColor Green
}

# 2. Connect to interactive control dashboard
Write-Host "`n>> [2/2] Entering interactive robot dashboard..." -ForegroundColor Cyan
Write-Host "--------------------------------------------------------------" -ForegroundColor DarkGray

ssh -t "spark@${IP}" "bash -ic '~/codes/dashboard.sh'"

Write-Host "`n--------------------------------------------------------------" -ForegroundColor DarkGray
Write-Host "Control session ended." -ForegroundColor Yellow
Start-Sleep -Seconds 2
