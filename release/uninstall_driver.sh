#!/bin/bash
echo "--------------------------------------"
echo "usage: sudo ./uninstall_driver.sh veyecam2m/veye327/csimx307/cssc132"
driver_name=null;
if [[ $1 == "veyecam2m" ]]
then
    driver_name=veyecam2m;
    sudo sed 's/^dtoverlay=veyecam2m/#dtoverlay=veyecam2m/g' -i /boot/config.txt

elif [[ $1 == "veye327" ]]
then
    driver_name=veye327;
    sudo sed 's/^dtoverlay=veye327/#dtoverlay=veye327/g' -i /boot/config.txt

elif [[ $1 == "csimx307" ]]
then
    driver_name=csimx307;
    sudo sed 's/^dtoverlay=csimx307/#dtoverlay=csimx307/g' -i /boot/config.txt
elif [[ $1 == "cssc132" ]]
then
    driver_name=cssc132;
    sudo sed 's/^dtoverlay=cssc132/#dtoverlay=cssc132/g' -i /boot/config.txt
else
    echo "please tell me the correct camera module name!"
    exit 0;
fi

echo "uninstall $driver_name driver"
#sudo sed 's/^dtoverlay=$driver_name/#dtoverlay=$driver_name/g' -i /boot/config.txt
echo "reboot now?(y/n):"
read USER_INPUT
case $USER_INPUT in
'y'|'Y')
    echo "reboot"
    sudo reboot
;;
*)
    echo "cancel"
;;
esac

