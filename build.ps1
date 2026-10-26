param(
    [string]$KeilPath = 'D:\STM32_Dev\Software\Keil_v5\UV4\UV4.exe',
    [string]$LogPath = ''
)
$ErrorActionPreference = 'Stop'
$taskProjectFile = Join-Path $PSScriptRoot 'MDK-ARM\STM32_D_Route.uvprojx'
if (-not (Test-Path -LiteralPath $KeilPath)) { throw "Keil executable not found: $KeilPath" }
if ($LogPath -eq '') { $LogPath = Join-Path $PSScriptRoot 'MDK-ARM\build.log' }
$taskArguments = @('-r', ('"' + $taskProjectFile + '"'), '-j0', '-o', ('"' + $LogPath + '"'))
$taskBuildProcess = Start-Process -FilePath $KeilPath -ArgumentList $taskArguments `
    -WorkingDirectory (Join-Path $PSScriptRoot 'MDK-ARM') -WindowStyle Hidden -Wait -PassThru
if (Test-Path -LiteralPath $LogPath) { Get-Content -LiteralPath $LogPath }
Write-Output ('Keil exit code: ' + $taskBuildProcess.ExitCode)
exit $taskBuildProcess.ExitCode
