$ErrorActionPreference = 'SilentlyContinue'
0..25 | ForEach-Object {
    $ip = "10.24.170.$_"
    $ping = Test-Connection -ComputerName $ip -Count 1 -Quiet
    if ($ping) {
        Write-Host "Active IP: $ip"
    }
}
