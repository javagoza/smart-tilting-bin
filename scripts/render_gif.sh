#!/usr/bin/env bash
# Renders the sorting animation of the Smart Tilting Bin to docs/animation.gif
# Requires: openscad (2021.01 or later) and ffmpeg. On a headless Linux machine, also xvfb.
set -euo pipefail

SCAD="${SCAD:-cad/smart_tilting_bin.scad}"
OUT="${OUT:-docs}"
FRAMES="${FRAMES:-160}"        # number of animation frames
SIZE="${SIZE:-800,600}"        # frame size in pixels
FPS="${FPS:-20}"
WIDTH="${WIDTH:-640}"          # final GIF width (keep the file under about 5 MB)
# Camera: translate x,y,z, rotate x,y,z, distance (the distance is recomputed by --viewall)
CAMERA="${CAMERA:-0,0,100,65,0,35,1500}"

mkdir -p "$OUT/frames"
rm -f "$OUT"/frames/f*.png

RUN=""
if command -v xvfb-run >/dev/null 2>&1; then RUN="xvfb-run -a"; fi

# OpenSCAD writes f00000.png, f00001.png, ...
$RUN openscad -o "$OUT/frames/f.png" \
  -D 'animate=true' \
  --animate "$FRAMES" \
  --imgsize="$SIZE" \
  --camera="$CAMERA" --viewall --autocenter \
  --projection=p \
  "$SCAD"

ffmpeg -y -framerate "$FPS" -i "$OUT/frames/f%05d.png" \
  -vf "scale=${WIDTH}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer" \
  "$OUT/animation.gif"

echo "Written $OUT/animation.gif"
