#!/bin/bash
path="images"
filename="sequence.txt" 
rm $filename
touch $filename
find $path/*.png >> $filename
