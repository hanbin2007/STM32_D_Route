param([string]$GccPath='D:\mingw64\bin\gcc.exe')
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
$taskHostExe=Join-Path $PSScriptRoot 'test_route.exe'
$taskSources=@('Tests/test_route.c','App/Src/route.c','App/Src/odometry.c',
    'App/Src/crossline_stop.c','App/Src/app_run.c','App/Src/chassis.c',
    'App/Src/speed_control.c','App/Src/pid.c','App/Src/hmi.c','App/Src/imu.c') | ForEach-Object { Join-Path $taskRoot $_ }
& $GccPath '-std=c11' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    ('-I'+(Join-Path $PSScriptRoot 'host')) ('-I'+(Join-Path $taskRoot 'App/Inc')) `
    @taskSources '-lm' '-o' $taskHostExe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $taskHostExe
exit $LASTEXITCODE
