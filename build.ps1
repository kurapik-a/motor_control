param(
 [string]$SdkRoot = 'C:\Users\Sinx\Documents\Codex\hpm_sdk_env_v1.9.0',
 [switch]$ConfigureOnly
)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$sdk = Join-Path $SdkRoot 'hpm_sdk'
$apps = Join-Path $SdkRoot 'hpm_apps'
$boardSource = Join-Path $apps 'boards/hpm6e00sip_pmsm'
$cmake = Join-Path $SdkRoot 'tools/cmake/bin/cmake.exe'
$ninja = Join-Path $SdkRoot 'tools/ninja/ninja.exe'
$python = Join-Path $SdkRoot 'tools/python3/python.exe'
$toolchain = Join-Path $SdkRoot 'toolchains/rv32imac_zicsr_zifencei_multilib_b_ext-win'
$required = @($cmake,$ninja,$python,(Join-Path $sdk 'VERSION'),(Join-Path $sdk 'cmake/hpm-sdk-config.cmake'),(Join-Path $apps 'CMakeLists.txt'),(Join-Path $boardSource 'board.h'),(Join-Path $boardSource 'board.c'),(Join-Path $toolchain 'bin/riscv32-unknown-elf-gcc.exe'),(Join-Path $apps 'middleware/hpm_motor/lib/libhpm_motor.a'),(Join-Path $apps 'middleware/vofa_debug/inc/vofa_debug.h'),(Join-Path $apps 'middleware/vofa_debug/port/hpm_uart/vofa_hpm_uart.c'))
$missing = @($required | Where-Object { -not (Test-Path -LiteralPath $_) })
if ($missing.Count) { throw "SDK extraction is incomplete. Missing files:`n$($missing -join "`n")" }
$version = Get-Content -LiteralPath (Join-Path $sdk 'VERSION') -Raw
if ($version -notmatch 'VERSION_MAJOR\s*=\s*1\b' -or $version -notmatch 'VERSION_MINOR\s*=\s*9\b') { throw 'Expected HPM SDK 1.9.x; check SdkRoot.' }
$localBoard = Join-Path $projectRoot 'boards/hpm6e00sip_pmsm'
if (-not (Test-Path -LiteralPath (Join-Path $localBoard 'board.h'))) {
 throw 'Local VOFA board files are missing; restore the project boards directory.'
}
 $boardHeader = Join-Path $localBoard 'board.h'
 $text = [IO.File]::ReadAllText($boardHeader)
 $originalText = $text
 foreach ($entry in @(@('MOTOR_USE_TRAJECTORY','1'),@('MOTOR_AUTO_RUN','0'))) {
  $pattern = '(?m)^([ \t]*#define[ \t]+' + $entry[0] + '[ \t]+)[^\r\n]+'
  if ($text -notmatch $pattern) { throw "Board macro missing: $($entry[0]); manual review required." }
  $text = [regex]::Replace($text,$pattern,('${1}'+$entry[1]))
 }
 if ($text -ne $originalText) { [IO.File]::WriteAllText($boardHeader,$text,(New-Object Text.UTF8Encoding($false))) }
$env:HPM_SDK_BASE = $sdk.Replace('\','/')
$env:GNURISCV_TOOLCHAIN_PATH = $toolchain.Replace('\','/')
$env:HPM_SDK_TOOLCHAIN_VARIANT = 'gcc'
$env:PATH = "$(Split-Path $python);$(Split-Path $ninja);$(Split-Path $cmake);$env:PATH"
$buildDir = Join-Path $projectRoot 'build'
& $cmake -S (Join-Path $projectRoot 'app') -B $buildDir -G Ninja "-DBOARD=hpm6e00sip_pmsm" "-DBOARD_SEARCH_PATH=$($projectRoot.Replace('\','/'))/boards" "-DHPM_APPS_BASE=$($apps.Replace('\','/'))" '-DHPM_BUILD_TYPE=flash_xip' "-DCMAKE_MAKE_PROGRAM=$($ninja.Replace('\','/'))" "-DPYTHON_EXECUTABLE=$($python.Replace('\','/'))"
if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
# CMake also organizes SES groups when invoked outside this wrapper.
if (-not $ConfigureOnly) {
 & $cmake --build $buildDir
 if ($LASTEXITCODE -ne 0) { throw 'Build failed.' }
}
