#!/bin/sh

model_folder="turtlebot3_ver_0_1"
file_id="1GS_ZrNeqW90VQQ-gHEkg0-c-EqkENMza"

# download model from google drvie
curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${file_id}" > /dev/null
CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
curl -Lb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${file_id}" -o ${model_folder}.zip

# unzip & remove
unzip ${model_folder}.zip
rm -r ${model_folder}.zip