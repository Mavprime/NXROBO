param(
    [string]$NewIP = "",
    [switch]$PushCalib
)

# Resolve directories relative to auto/
$codesDir = Join-Path (Split-Path -Parent $PSScriptRoot) "codes"
$ipFile = Join-Path $PSScriptRoot "robot_ip.txt"
if (-not (Test-Path $ipFile)) {
    $altIpFile = Join-Path (Split-Path -Parent $PSScriptRoot) "docs\robot_ip.txt"
    if (Test-Path $altIpFile) { $ipFile = $altIpFile }
}
$nodesDir = "/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes"

if ($NewIP -ne "") {
    Set-Content -Path $ipFile -Value $NewIP.Trim()
    Write-Host ">> Updated saved robot IP: $NewIP" -ForegroundColor Yellow
}

if (Test-Path $ipFile) {
    $IP = (Get-Content -Path $ipFile).Trim()
} else {
    $IP = "10.24.170.0"
}

Write-Host "==============================================" -ForegroundColor Cyan
Write-Host "  Uploading updates to Spark robot ($IP)      " -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan

# 1. Files that must live next to runtask.py in the ROS package (including pad_detect.py)
scp "$codesDir\runtask.py" "$codesDir\detect_ros.py" "$codesDir\match_logic.py" "$codesDir\pad_detect.py" "spark@${IP}:${nodesDir}/"
if ($LASTEXITCODE -ne 0) {
    Write-Host ">> [ERROR] SCP to ROS nodes failed - check the IP / Wi-Fi. Nothing was restarted." -ForegroundColor Red
    exit 1
}

# 2. Operator tooling in ~/codes (teleop, dashboard, launchers, IK shim, field grasp tools).
# NOTE: transform_3d.dat is intentionally NOT uploaded by default so a fresh
# calib3d.py fit on the robot is never silently overwritten (SI-38).
scp "$codesDir\runtask.py" "$codesDir\detect_ros.py" "$codesDir\keypatched2v.py" `
    "$codesDir\arm_ik.py" "$codesDir\match_logic.py" "$codesDir\pad_detect.py" `
    "$codesDir\grasp_generic.py" "$codesDir\robot_view.py" "$codesDir\run_grasp_test.sh" `
    "$codesDir\patch_swiftpro.py" "$codesDir\snap.py" `
    "$codesDir\pick_direct.py" "$codesDir\calib3d.py" "$codesDir\servo_grasp.py" `
    "$codesDir\cube_check.py" "$codesDir\approach.py" `
    "$codesDir\start_system.sh" "$codesDir\stop_system.sh" "$codesDir\dashboard.sh" `
    "$codesDir\camera_stream.py" "$codesDir\test_all_systems.py" `
    "$codesDir\test_all_systems.sh" "$codesDir\view_log.py" `
    "spark@${IP}:/home/spark/codes/"

if ($LASTEXITCODE -ne 0) {
    Write-Host ">> [ERROR] SCP to ~/codes failed - check the IP / Wi-Fi. Nothing was restarted." -ForegroundColor Red
    exit 1
}

# 3. Optional explicit calibration push (only when -PushCalib is passed)
if ($PushCalib) {
    Write-Host ">> [-PushCalib] Uploading transform_3d.dat (+ meta if present) to ~/..." -ForegroundColor Yellow
    scp "$codesDir\transform_3d.dat" "spark@${IP}:/home/spark/transform_3d.dat"
    if (Test-Path "$codesDir\transform_3d.meta.json") {
        scp "$codesDir\transform_3d.meta.json" "spark@${IP}:/home/spark/transform_3d.meta.json"
    }
}

ssh "spark@${IP}" "chmod +x /home/spark/codes/*.sh /home/spark/codes/*.py ${nodesDir}/runtask.py ${nodesDir}/detect_ros.py ${nodesDir}/match_logic.py ${nodesDir}/pad_detect.py; FORCE_DETECT=1 bash /home/spark/codes/start_system.sh"

Write-Host ">> All files successfully copied to robot!" -ForegroundColor Green
