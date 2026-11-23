# One-shot remote helper for the Spark robot (sandbox-safe: no piped stdio).
function Invoke-RobotScript {
    param([Parameter(Mandatory = $true)][string]$Script)
    $sh  = "D:\NXROBO\.tmp\remote.sh"
    $out = "D:\NXROBO\.tmp\rr.out"
    Set-Content -Path $sh -Value $Script -Encoding ASCII -NoNewline
    if (Test-Path $out) { Remove-Item $out -Force -ErrorAction SilentlyContinue }
    $line = 'ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=no spark@10.24.170.0 bash -s < ' + $sh + ' > ' + $out + ' 2>&1'
    Start-Process cmd.exe -ArgumentList '/c', $line -Wait -NoNewWindow | Out-Null
    if (Test-Path $out) { Get-Content $out -Encoding UTF8 }
}
