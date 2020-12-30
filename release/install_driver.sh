#!/bin/bash
echo "--------------------------------------"
echo "usage: sudo ./install_driver.sh veye327/csimx307/cssc132"
driver_name=null;
if [[ $1 == "veye327" ]]
then
    driver_name=veye327;
elif [[ $1 == "csimx307" ]]
then
    driver_name=csimx307;
elif [[ $1 == "cssc132" ]]
then
    driver_name=cssc132;
else
    echo "please tell me the correct camera module name!"
    exit 0;
fi

write_camera_to_config()
{
    if [[ $driver_name == "veye327" ]]
    then
        awk 'BEGIN{ count=0 }       \
        {                           \
            if($1 == "dtoverlay=veye327"){       \
                count++;            \
            }                       \
        }END{                       \
            if(count <= 0){         \
                system("sudo sh -c '\''echo dtoverlay=veye327 >> /boot/config.txt'\''"); \
            }                       \
        }' /boot/config.txt
    elif [[ $driver_name == "csimx307" ]]
    then
        awk 'BEGIN{ count=0 }       \
        {                           \
            if($1 == "dtoverlay=csimx307"){       \
                count++;            \
            }                       \
        }END{                       \
            if(count <= 0){         \
                system("sudo sh -c '\''echo dtoverlay=csimx307 >> /boot/config.txt'\''"); \
            }                       \
        }' /boot/config.txt
    elif [[ $driver_name == "cssc132" ]]
    then
        awk 'BEGIN{ count=0 }       \
        {                           \
            if($1 == "dtoverlay=cssc132"){       \
                count++;            \
            }                       \
        }END{                       \
            if(count <= 0){         \
                system("sudo sh -c '\''echo dtoverlay=cssc132 >> /boot/config.txt'\''"); \
            }                       \
        }' /boot/config.txt
    else
        echo "please tell me the correct camera module name!"
        exit 0;
    fi
}
echo "--------------------------------------"
echo "Enable i2c0 adapter..."
echo "--------------------------------------"
sudo modprobe i2c-dev
# add dtparam=i2c_vc=on to /boot/config.txt
awk 'BEGIN{ count=0 }       \
{                           \
    if($1 == "dtparam=i2c_vc=on"){       \
        count++;            \
    }                       \
}END{                       \
    if(count <= 0){         \
        system("sudo sh -c '\''echo dtparam=i2c_vc=on >> /boot/config.txt'\''"); \
    }                       \
}' /boot/config.txt
echo "Add dtoverlay=$driver_name to /boot/config.txt "
echo "--------------------------------------"
write_camera_to_config;
echo "Add gpu=400M to /boot/config.txt "
awk 'BEGIN{ count=0 }       \
{                           \
    if($1 == "gpu_mem=400"){       \
        count++;            \
    }                       \
}END{                       \
    if(count <= 0){         \
        system("sudo sh -c '\''echo gpu_mem=400 >> /boot/config.txt'\''"); \
    }                       \
}' /boot/config.txt
echo "Add cma=128M to /boot/cmdline.txt "
echo "--------------------------------------"
sudo sed 's/cma=128M//g' -i /boot/cmdline.txt
sudo sed 's/[[:blank:]]*$//' -i /boot/cmdline.txt
sudo sed 's/$/& cma=128M/g' -i /boot/cmdline.txt
echo "Installing the $driver_name.ko driver"
echo "--------------------------------------"
sudo install -p -m 644 ./driver_bin/$(uname -r)/$driver_name.ko  /lib/modules/$(uname -r)/kernel/drivers/media/i2c/
sudo install -p -m 644 ./driver_bin/$(uname -r)/$driver_name.dtbo /boot/overlays/
sudo /sbin/depmod -a $(uname -r)
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

        
