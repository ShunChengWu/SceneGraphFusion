# This script is from https://github.com/WaldJohannaU/3RScan with MIT License (Copyright (c) 2020 Johanna Wald)
# 26 July 2022, modified by Shun-Cheng Wu

if [[ ! -d "data" ]]; then
	mkdir data
fi

# download example data
if [[ ! -d "data/3RScan" ]]; then
	if [[ ! -f "data/3RScan.v2.zip" ]]; then
		wget "http://www.campar.in.tum.de/public_datasets/3RScan/3RScan.v2.zip" -P data
	fi
	unzip "data/3RScan.v2.zip" -d ./data/3RScan
fi

if [[ ! -f "data/3RScan/3RScan.json" ]]; then
	wget "http://www.campar.in.tum.de/public_datasets/3RScan/3RScan.json" -P data/3RScan
fi
if [[ ! -f "data/3RScan/objects.json" ]]; then
	wget "http://www.campar.in.tum.de/public_datasets/3DSSG/3DSSG/objects.json" -P data/3RScan
fi
