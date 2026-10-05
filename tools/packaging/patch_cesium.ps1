# Patches the project's copy of Cesium for Unreal (see copy_cesium.bat). Each change is applied only once.
param([Parameter(Mandatory)][string]$Plugin)
$ErrorActionPreference = 'Stop'

# Rewrites a file through $Edit, keeping its UTF-8 encoding (with or without BOM) and line endings
function Update-File([string]$Path, [scriptblock]$Edit) {
    $bytes = [IO.File]::ReadAllBytes($Path)
    $bom = $bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF
    $encoding = New-Object Text.UTF8Encoding($bom)
    $text = $encoding.GetString($bytes)
    if ($bom) { $text = $text.Substring(1) }
    $edited = & $Edit $text
    if ($edited -cne $text) { [IO.File]::WriteAllText($Path, $edited, $encoding) }
}

$runtime = Join-Path $Plugin 'Source\CesiumRuntime'

# Linux builds treat unreachable code as an error: report it as a warning until CesiumGS/cesium-unreal#1906 lands
Update-File (Join-Path $runtime 'CesiumRuntime.Build.cs') {
    param($s)
    if ($s -match 'UnreachableCodeWarningLevel') { return $s }
    $nl = if ($s -match "`r`n") { "`r`n" } else { "`n" }
    $s -replace '(public CesiumRuntime\(ReadOnlyTargetRules Target\) : base\(Target\)\s*\{)', ('$1' + $nl + '        CppCompileWarningSettings.UnreachableCodeWarningLevel = WarningLevel.Warning;')
}

# A headless game (-nullrhi) can't create the tiles' render resources; collision still works without them (until CesiumGS/cesium-unreal#1907 lands)
Update-File (Join-Path $runtime 'Private\CesiumGltfComponent.cpp') {
    param($s)
    if ($s -match 'FApp::CanEverRender\(\)\) \{\s*TRACE_CPUPROFILER_EVENT_SCOPE\(Cesium::InitResources\)') { return $s }
    $nl = if ($s -match "`r`n") { "`r`n" } else { "`n" }
    $s = [regex]::Replace($s, '\{(\s*TRACE_CPUPROFILER_EVENT_SCOPE\(Cesium::InitResources\)\s*pStaticMesh->InitResources\(\);)', 'if (FApp::CanEverRender()) {$1')
    if ($s -notmatch '#include "Misc/App.h"') { $s = ([regex]'#include "(Misc|Engine)/').Replace($s, ('#include "Misc/App.h"' + $nl + '$&'), 1) }
    $s
}

# Compiled from source, so not an installed (precompiled) plugin
Update-File (Join-Path $Plugin 'CesiumForUnreal.uplugin') {
    param($s)
    $s -replace '"Installed": true', '"Installed": false'
}
