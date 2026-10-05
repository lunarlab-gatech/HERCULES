# usage: set_default_map.ps1 <project folder> <map>
# Points a project's GameDefaultMap and its MapsToCook entry under /Game at one map (used by package.bat).
param([string]$Project, [string]$Map)
$engine = Join-Path $Project 'Config\DefaultEngine.ini'
(Get-Content -Raw $engine) -replace 'GameDefaultMap=[^\r\n]*', "GameDefaultMap=$Map" | Set-Content -NoNewline $engine
$game = Join-Path $Project 'Config\DefaultGame.ini'
(Get-Content -Raw $game) -replace '\+MapsToCook=\(FilePath="/Game/[^"]*"\)', "+MapsToCook=(FilePath=`"$Map`")" | Set-Content -NoNewline $game
