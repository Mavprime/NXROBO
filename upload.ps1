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

Write-Host "==============================================" -ForegroundColor Cyan
Write-Host "  Uploading updates to Spark robot ($IP)      " -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan

scp "D:\ROS competition\spark_humble_ws\codes\runtask.py" "spark@${IP}:/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py"
scp "D:\ROS competition\spark_humble_ws\codes\detect_ros.py" "spark@${IP}:/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py"
scp "D:\ROS competition\spark_humble_ws\codes\runtask.py" "D:\ROS competition\spark_humble_ws\codes\detect_ros.py" "D:\ROS competition\spark_humble_ws\codes\keypatched2v.py" "D:\ROS competition\spark_humble_ws\codes\arm_ik.py" "D:\ROS competition\spark_humble_ws\codes\start_system.sh" "D:\ROS competition\spark_humble_ws\codes\stop_system.sh" "D:\ROS competition\spark_humble_ws\codes\dashboard.sh" "D:\ROS competition\spark_humble_ws\codes\camera_stream.py" "D:\ROS competition\spark_humble_ws\codes\test_all_systems.py" "D:\ROS competition\spark_humble_ws\codes\test_all_systems.sh" "D:\ROS competition\spark_humble_ws\codes\view_log.py" "spark@${IP}:/home/spark/codes/"
ssh "spark@${IP}" "chmod +x /home/spark/codes/*.sh /home/spark/codes/*.py /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py; FORCE_DETECT=1 bash /home/spark/codes/start_system.sh"

Write-Host ">> All files successfully copied to robot!" -ForegroundColor Green
