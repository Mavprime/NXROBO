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
Write-Host "  Подключение к пульту управления Spark ($IP) " -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan

ssh -t "spark@${IP}" "bash -ic '~/codes/dashboard.sh'"
