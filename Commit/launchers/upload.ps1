param(
    [string]$NewIP = ""
)

$ipFile = "D:\ROS competition\robot_ip.txt"

if ($NewIP -ne "") {
    Set-Content -Path $ipFile -Value $NewIP.Trim()
    Write-Host ">> Обновлен сохраненный IP робота: $NewIP" -ForegroundColor Yellow
}

if (Test-Path $ipFile) {
    $IP = (Get-Content -Path $ipFile).Trim()
} else {
    $IP = "10.24.170.0"
}

Write-Host "==============================================" -ForegroundColor Cyan
Write-Host "  Загрузка обновлений на робота Spark ($IP)    " -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan

scp "D:\ROS competition\spark_humble_ws\codes\runtask.py" "spark@${IP}:/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py"
scp "D:\ROS competition\spark_humble_ws\codes\runtask.py" "D:\ROS competition\spark_humble_ws\codes\keypatched2v.py" "D:\ROS competition\spark_humble_ws\codes\arm_ik.py" "D:\ROS competition\spark_humble_ws\codes\start_system.sh" "D:\ROS competition\spark_humble_ws\codes\stop_system.sh" "D:\ROS competition\spark_humble_ws\codes\dashboard.sh" "spark@${IP}:/home/spark/codes/"

Write-Host ">> Все файлы успешно скопированы на робота!" -ForegroundColor Green
