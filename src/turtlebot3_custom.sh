#!/bin/sh

model_folder="turtlebot3_ver_0_3"
file_id="1kzoKzVoB5ctvGR7Q1BT5qEWwwiBI_76J"

# download model from google drvie
curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${file_id}" > /dev/null
CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
curl -Lb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${file_id}" -o ${model_folder}.zip

# unzip & remove
unzip ${model_folder}.zip
rm -r ${model_folder}.zip
