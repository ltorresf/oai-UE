#/bin/bash
echo "Start: `date`"
#fecha=$(date +"%Y%m%d%H%M%S")
#file=${fecha}_UE.log
cd /home/esk/src/openairinterface5g/
source oaienv
cd cmake_targets/lte_build_oai/build/
#a=2660000000
#a=2611250000
#a=2646900000
#a=2110000000
a=1845000000
#a=1815000000
gain=125
rb=50

b=$a
c=2170000000
i=1
while [[ $i -le 1 ]]; do
#while [[ $b -le $c ]]; do
	fecha=$(date +"%Y%m%d"."%H%M%S")
	file=${fecha}_${b}_${rb}_${gain}_UE.log
	#b=$(($a+$i*2500000))
	#sleep 10s
	sudo -E ./lte-softmodem -U -C${b} -r${rb} --ue-scan-carrier --ue-txgain 70 --ue-rxgain $gain &> $file
	#sudo gdb --args -E ./lte-softmodem -U --ue-scan-carrier --ue-txgain 70 --ue-rxgain 80 &> $file
	#sudo -E ./lte-softmodem -U --ue-scan-carrier --ue-txgain 70 --ue-rxgain 110 &> $file
	offset=$(cat $file | grep "carrier off" | tail -1 | awk -F" " '{print $6}')
	dump=$(cat $file | grep "dump" | wc -l)
	echo "i = $i, freq = $b, offset = $offset, dump = $dump"
	cat $file | grep "dump" | head
	b=$(($a+$i*7500))
	((i++))
done
#offset=$(cat $file | grep "carrier off" | tail -1 | awk -F" " '{print $6}')
#echo "offset = $offset"
echo "End: `date`"
echo "======================="
