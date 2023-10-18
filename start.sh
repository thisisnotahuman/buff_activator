#/bin/bash
#/bin/bash
export i=0
#$1为项目的源代码所在地，可修改
path=/home/tdt/tdtvision2021
mkdir log
trap "echo 'cdscdcs';pid=$( ps aux|grep TDTVision_RM2021|grep -v grep|awk '{ print $2}');echo $pid;kill -s SIGINT $pid;exit;" SIGINT
trap "exit;" SIGHUP
while [ $i -ne 1 ]
do
	if [ $(ps aux|grep TDTVision_RM2021|grep -v grep|wc -l) -ne 1 ]
	then
		export time=$(date +%Y-%m-%d-%H-%M-%S)
			if [ -n "$path" ]
			then
				cd "$path/build/bin"
				
			fi
		touch $path/log/$time.log
		echo tdt|sudo -S ./TDTVision_RM2021 >$path/log/$time.log
	fi
	sleep 0.5
done

# cd build/bin
# export i=0
# trap "echo 'cdscdcs';pid=$( ps aux|grep TDTVision_RM2021|grep -v grep|awk '{ print $2}');echo $pid;kill -4 $pid;exit;" SIGINT
# while [ $i -ne 1 ]
# do
# 	if [ $(ps aux|grep TDTVision_RM2021|grep -v grep|wc -l) -ne 1 ]
# 	then
# 			if [ -n "$1" ]
# 			then
# 				cd $1
# 				cmake ..
# 				make -j8
# 				cd bin
# 			fi
# 		sudo ./TDTVision_RM2021
# 	fi
# 	sleep 1
# done

