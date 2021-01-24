#!/bin/sh

model_folder="turtlebot3_ver_0_2"
file_id="1k7LxvUcP8yEql3D8JL5l_INmRi6pr3q8"

# download model from google drvie
curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${file_id}" > /dev/null
CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
curl -Lb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${file_id}" -o ${model_folder}.zip

# unzip & remove
unzip ${model_folder}.zip
rm -r ${model_folder}.zip
