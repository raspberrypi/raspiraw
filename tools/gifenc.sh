#!/bin/sh

# doc:   http://blog.pkh.me/p/21-high-quality-gif-with-ffmpeg.html
# needs: ffmpeg

palette="/tmp/palette.png"

#filters="fps=15,scale=320:-1:flags=lanczos"
filters="fps=25,scale=640:-1:flags=lanczos"

ffmpeg -v warning -i $1 -vf "$filters,palettegen" -y $palette
ffmpeg -v warning -i $1 -i $palette -lavfi "$filters [x]; [x][1:v] paletteuse" -y $2
