# PowerShell 1-Click Master Launcher & Sync
$codesDir = "D:\ROS competition\spark_humble_ws\codes"
$ipFile = "D:\ROS competition\robot_ip.txt"

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
scp "$codesDir\runtask.py" "spark@${IP}:/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py"
scp "$codesDir\detect_ros.py" "spark@${IP}:/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py"
scp "$codesDir\runtask.py" "$codesDir\keypatched2v.py" "$codesDir\arm_ik.py" "$codesDir\start_system.sh" "$codesDir\stop_system.sh" "$codesDir\dashboard.sh" "$codesDir\camera_stream.py" "$codesDir\test_all_systems.py" "$codesDir\test_all_systems.sh" "$codesDir\view_log.py" "spark@${IP}:/home/spark/semi/"
ssh "spark@${IP}" "chmod +x /home/spark/semi/*.sh /home/spark/semi/*.py /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py; FORCE_DETECT=1 bash /home/spark/semi/start_system.sh"

Write-Host ">> [OK] All scripts on robot are up to date!" -ForegroundColor Green

# 2. Connect to interactive control dashboard
Write-Host "`n>> [2/2] Entering interactive robot dashboard..." -ForegroundColor Cyan
Write-Host "--------------------------------------------------------------" -ForegroundColor DarkGray

ssh -t "spark@${IP}" "bash -ic '~/semi/dashboard.sh'"

Write-Host "`n--------------------------------------------------------------" -ForegroundColor DarkGray
Write-Host "Control session ended." -ForegroundColor Yellow
Start-Sleep -Seconds 2
