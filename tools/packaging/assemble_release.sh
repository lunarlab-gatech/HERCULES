#!/usr/bin/env bash
# Zips a packaged HERCULES world for release, with its settings files, launchers, README and VERSION.txt.
#   assemble_release.sh <World> <Win64|Linux> <package folder> <settings json, relative to the repository>...
# e.g. assemble_release.sh Blocks Win64 "$HERCULES_OUT/packages/Blocks_Win64" ros2/settings/hero_blocks_team.json
# Writes $HERCULES_OUT/release/HERCULES-<World>-UE5.8-<Platform>.zip. HERCULES_OUT defaults to the folder
# containing the repository, as for the Windows scripts; UE_DIR locates the engine for VERSION.txt.
set -euo pipefail
world=$1 platform=$2 archive=$3; shift 3
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
out=${HERCULES_OUT:-$(dirname "$repo")}/release
ue_dir=${UE_DIR:-"/mnt/c/Program Files/Epic Games/UE_5.8"}
name="HERCULES-${world}-UE5.8-${platform}"
stage="$out/$name"
rm -rf "$stage" "$out/$name.zip"
mkdir -p "$stage/settings"

case $platform in
  Win64) src="$archive/Windows" ;;
  Linux) src="$archive/Linux" ;;
  *) echo "unknown platform $platform"; exit 1 ;;
esac
# Saved/ holds logs and config from test runs (which can include command lines and tokens), and
# Cesium's request cache holds the streamed tiles (not ours to redistribute) and their request URLs
rsync -a --exclude 'Saved/' --exclude 'cesium-request-cache.sqlite*' "$src/" "$stage/"

# Each settings file, and a _network copy that accepts connections from other machines (and WSL)
for f in "$@"; do
  base=$(basename "$f" .json)
  cp "$repo/$f" "$stage/settings/$base.json"
  python3 -c '
import json, sys
d = json.load(open(sys.argv[1]))
d["LocalHostIp"] = "0.0.0.0"
json.dump(d, open(sys.argv[2], "w"), indent=2)
' "$repo/$f" "$stage/settings/${base}_network.json"
done

# Launchers: start windowed with the first settings file; extra arguments are passed through
first_settings=$(basename "$1" .json)
if [[ $platform == Win64 ]]; then
  exe=$(cd "$stage" && ls *.exe | head -1)
  printf '@echo off\r\nREM Starts %s with the HERCULES team settings. Extra arguments are passed to the game,\r\nREM e.g. run_hero_team.bat -CesiumOrigin=33.7756,-84.3963\r\nstart "" "%%~dp0%s" -windowed -ResX=1280 -ResY=720 -settings="%%~dp0settings\\%s.json" %%*\r\n' \
    "$world" "$exe" "$first_settings" > "$stage/run_hero_team.bat"
else
  sh_name=$(cd "$stage" && ls *.sh | head -1)
  printf '#!/usr/bin/env bash\n# Starts %s with the HERCULES team settings. Extra arguments are passed to the game,\n# e.g. ./run_hero_team.sh -CesiumOrigin=33.7756,-84.3963\ncd "$(dirname "$0")"\nexec ./%s -windowed -ResX=1280 -ResY=720 -settings="$PWD/settings/%s.json" "$@"\n' \
    "$world" "$sh_name" "$first_settings" > "$stage/run_hero_team.sh"
  chmod +x "$stage/run_hero_team.sh"
fi

cp "$repo/docs/packaged_environments.md" "$stage/README.md"
cp "$repo/LICENSE" "$stage/LICENSE-HERCULES.txt"
ue_version=$(python3 -c '
import json, sys
try:
    v = json.load(open(sys.argv[1]))
    print("{}.{}.{} (CL {})".format(v["MajorVersion"], v["MinorVersion"], v["PatchVersion"], v["Changelist"]))
except OSError:
    print("unknown")
' "$ue_dir/Engine/Build/Build.version")
dirty=$(git -C "$repo" diff --quiet HEAD -- . || echo ' + uncommitted changes')
{
  echo "HERCULES $world for $platform"
  echo "Unreal Engine: $ue_version"
  echo "Source: $(git -C "$repo" remote get-url origin) branch $(git -C "$repo" rev-parse --abbrev-ref HEAD) commit $(git -C "$repo" rev-parse --short HEAD)$dirty"
  echo "Built: $(date -u +%Y-%m-%dT%H:%MZ)"
} > "$stage/VERSION.txt"

# zip keeps the Linux executables' permissions
if command -v 7zz >/dev/null || command -v 7z >/dev/null; then
  (cd "$out" && "$(command -v 7zz || command -v 7z)" a -tzip -mx=5 -bso0 -bsp0 "$name.zip" "$name")
else
  (cd "$out" && zip -q -r -y "$name.zip" "$name")
fi
rm -rf "$stage"
ls -la "$out/$name.zip"
