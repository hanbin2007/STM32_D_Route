param([string]$GccPath='gcc')
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
$taskExe=Join-Path $PSScriptRoot 'test_parking.exe'
& $GccPath '-std=c11' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    ('-I'+(Join-Path $taskRoot 'App/Inc')) `
    (Join-Path $PSScriptRoot 'test_parking.c') `
    (Join-Path $taskRoot 'App/Src/parking.c') `
    '-lm' '-o' $taskExe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $taskExe
exit $LASTEXITCODE
