# create scan directory if it doesn't exist
if [ ! -d "/home/canary/scans" ]; then
	mkdir /home/canary/scans
	mkdir /home/canary/scans/temp
fi

if [ ! -d "/home/canary/scans/temp" ]; then
	mkdir /home/canary/scans/temp
fi

build/receiver &
build/motorNode &
build/lidarNode &
wait

