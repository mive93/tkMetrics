#!/bin/bash


function elaborate_testset {
    wget $1 -O $2.zip
    unzip -d $2 $2.zip
    rm $2.zip
    cd $2/
    realpath images/* > all_images.txt
    cd ..
} 

cd data
for valset in $@
do
    
    if [ $valset = "COCO" ]; then
        echo "Downloading $valset validation set in data"
        elaborate_testset "https://cloud.hipert.unimore.it/s/LNxBDk4wzqXPL8c/download" "COCO_val2017"
    elif [ $valset = "BDD" ]; then
        echo "Downloading $valset validation set in data"
        elaborate_testset "https://cloud.hipert.unimore.it/s/bikqk3FzCq2tg4D/download" "BDD100K_val"
    fi

done
