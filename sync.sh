#!bin/bash

# Run this script on Linux or Mac to sync the libraries in this repo to your Arduino sketchbook.

# Parse argument (sketchbook location) 
if [ $# == 1 ]; then  # arg 1 is the sketchbook
	sketchbook=$1
	echo "Using user-provided sketchbook location ($sketchbook)"
else  # detect sketchbook based on OS
	if [[ $OSTYPE == 'darwin'* ]]; then  # Mac OS 
		echo "Detected Mac OS X"
		sketchbook=~/Documents/Arduino
	elif [[ $(uname -s) == Linux ]]; then  # Linux
		echo "Detected Linux"
		sketchbook=~/Arduino
	else 
		echo "Could not detect sketchbook location for your OS."
		echo "Please pass your sketchbook directory as an argument to this script"
		read -p "Press [Enter] to finish"
		exit
	fi
	echo "Using default sketchbook location ($sketchbook)"
	echo "To override this, pass your sketchbook directory as an argument to this script"
fi

# Verify sketchbook directory exists
echo ""
if [ ! -d $sketchbook ]; then
	echo "Error: Cannot find sketchbook at $sketchbook"
	read -p "Press [Enter] to finish"
	exit
fi

# DEPRECATED, LEFT FOR REFERENCE: Sync all libraries to the sketchbook
# echo "Syncing science libraries to Arduino..."
# for file in $( find libraries -type f); do
# 	dir=$( dirname $file )
# 	echo "Syncing directory: $dir"
# 	mkdir -p $sketchbook/$dir
# 	if [ ! -f $sketchbook/$file ]; then 
# 		ln $file $sketchbook/$file
# 	fi
# done

# Make a link in the Arduino libraries folder that points here
ln . $sketchbook/libraries/BURT_TMC

echo "Done!"
read -p "Press [Enter] to finish"
