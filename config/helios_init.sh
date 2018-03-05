
#!/bin/bash

echo GPS to tcp
sudo socat -u /dev/gps,b9600,raw,echo=0 tcp-listen:4001,reuseaddr,fork &
pid_socat1=$!

sleep 1

echo tcp to SBG
sudo socat -u tcp:localhost:4001 /dev/sbg_aux,b9600,raw,echo=0 &
pid_socat2=$!

sleep 1

echo RTK to GPS
sudo socat -u tcp:192.168.0.187:5017 /dev/gps,b9600,raw,echo=0 &

pid_socat3=$!

sleep 1

function cleanup {
	sudo kill $pid_socat1
	sudo kill $pid_socat2
	sudo kill $pid_socat3
}
trap cleanup EXIT

echo tcp to LOG
file_name=log_rtk_raw_$(date +%y-%m%d_%H.%M.%S)
echo $file_name

sudo socat -u tcp:localhost:4001 ~/$file_name,seek-end=0




