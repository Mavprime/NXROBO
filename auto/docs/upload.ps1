param(
    [string]$NewIP = ""
)

$ipFile = "D:\ROS competition\robot_ip.txt"

if ($NewIP -ne "") {
    Set-Content -Path $ipFile -Value $NewIP.Trim()
    Write-Host ">> Updated saved robot IP: $NewIP" -ForegroundColor Yellow
}

if (Test-Path $ipFile) {
    $IP = (Get-Content -Path $ipFile).Trim()
} else {
    $IP = "10.24.170.0"
}

$codesDir = "D:\ROS competition\spark_humble_ws\codes"
$nodesDir = "/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes"

Write-Host "==============================================" -ForegroundColor Cyan
Write-Host "  Uploading updates to Spark robot ($IP)      " -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan

# 1. Files that must live next to runtask.py in the ROS package (they import
#    each other, so runtask.py + match_logic.py are always uploaded together).
scp "$codesDir\runtask.py" "$codesDir\detect_ros.py" "$codesDir\match_logic.py" "spark@${IP}:${nodesDir}/"

# 2. Operator tooling in ~/codes (teleop, dashboard, launchers, IK shim, grasp tools).
scp "$codesDir\runtask.py" "$codesDir\detect_ros.py" "$codesDir\keypatched2v.py" `
    "$codesDir\arm_ik.py" "$codesDir\match_logic.py" `
    "$codesDir\pick_direct.py" "$codesDir\calib3d.py" "$codesDir\servo_grasp.py" `
    "$codesDir\start_system.sh" "$codesDir\stop_system.sh" "$codesDir\dashboard.sh" `
    "$codesDir\camera_stream.py" "$codesDir\test_all_systems.py" `
    "$codesDir\test_all_systems.sh" "$codesDir\view_log.py" `
    "spark@${IP}:/home/spark/codes/"

if ($LASTEXITCODE -ne 0) {
    Write-Host ">> [ERROR] SCP failed - check the IP / Wi-Fi. Nothing was restarted." -ForegroundColor Red
    exit 1
}

ssh "spark@${IP}" "chmod +x /home/spark/codes/*.sh /home/spark/codes/*.py ${nodesDir}/runtask.py ${nodesDir}/detect_ros.py ${nodesDir}/match_logic.py; FORCE_DETECT=1 bash /home/spark/codes/start_system.sh"

Write-Host ">> All files successfully copied to robot!" -ForegroundColor Green
