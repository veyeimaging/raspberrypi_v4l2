#!/bin/bash
#/-------------------\        /----------------------\        /--------------------\        /------------------\
#| MIPI CSI RECEIVER |        | V-by-ONE DESERIALIZER|        | V-by-ONE SERIALIZER|        | MIPI CSI CAMERA  |
#+-------------------+  CSI   +----------------------+        +--------------------+  CSI   +------------------+
#|                   | <----- |                      |  Coax  |                    | <----- |                  |
#| Raspberry Pi (4)  |  GPIO  | V-by-ONE RX          | <----> | V-by-ONE TX        |  GPIO  | MV series CAM    |
#| Jetson Nano       | <----> | THCV242-P            |        | THCV241A-P         | <----> | RAW series CAM   |
#| ...               |   I2C  |                      |        |                    |   I2C  |                  |
#|                   | <----> |                      |        |                    | <----> |                  |
#\-------------------/        \----------------------/        \--------------------/        \------------------/

I2C_DEV=10;
SER_ADDR=0x34;
DES_ADDR=0x65;
CAM_ADDR=0x3B;

CAM_MIPI_DATARATE=1500;
CAM_LAN=2;
COAX_NUM=1;
IO_MODE=0;

print_usage()
{   
    echo "Use the I2C command to initialize the SERDES channel of V-by-One."
	echo "Usage:  ./vbyone_i2c_init.sh "
	echo "options:"
    echo "    -b [i2c bus num] 		   i2c bus number,default 10"
    echo "    -n [coax number] 	       coax number [1,2],default 1"
	echo "    -s [camera mipi datarate]    1500 means:1500Mbps/lan; 1188 means: 1188Mbps/lan"
	echo "    -l [camera lan number] 		2 or 4"
    echo "    -r [io remote or local] 		0 means use io locally(on TX side); 1 means use io remotely(on RX side)"
    echo "    -h                            print this help"
}

while getopts "b:n:s:l:r:h" opt; do
  case $opt in
    b) I2C_DEV=$OPTARG ;;
    n) COAX_NUM=$OPTARG ;;
    s) CAM_MIPI_DATARATE=$OPTARG ;;
    l) CAM_LAN=$OPTARG ;;
    r) IO_MODE=$OPTARG ;;
    h)
      print_usage
      exit 0
      ;;
    \?)
      echo "Invalid option: -$OPTARG"
      print_usage
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument."
      print_usage
      exit 1
      ;;
  esac
done

echo "I2C_DEV: $I2C_DEV"
echo "COAX_NUM: $COAX_NUM"
echo "CAM_MIPI_DATARATE: $CAM_MIPI_DATARATE"
echo "CAM_LAN: $CAM_LAN"
echo "IO_MODE: $IO_MODE"

if [ $COAX_NUM -ne 1 ]; then
  echo "Error: Invalid COAX_NUM value. Must be 1."
  exit 1
fi

if [ $CAM_LAN -ne 2 ]; then
  echo "Error: Invalid CAM_LAN value. Must be 2."
  exit 1
fi

if [[ $CAM_MIPI_DATARATE -ne 1500 && $CAM_MIPI_DATARATE -ne 1188 ]]; then
  echo "Error: Invalid CAM_MIPI_DATARATE value. Must be 1500 or 1188."
  exit 1
fi

if [[ $IO_MODE -ne 0 && $IO_MODE -ne 1 ]]; then
  echo "Error: Invalid IO_MODE value. Must be 0 or 1."
  exit 1
fi

# [Rx Register Settings]
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x50 $SER_ADDR i
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x04 0x03 i
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x10 0x10 i
i2cset -y $I2C_DEV $DES_ADDR 0x17 0x04 0x01 i
i2cset -y $I2C_DEV $DES_ADDR 0x01 0x02 0x02 i
i2cset -y $I2C_DEV $DES_ADDR 0x01 0x03 0x02 i
i2cset -y $I2C_DEV $DES_ADDR 0x01 0x04 0x02 i
i2cset -y $I2C_DEV $DES_ADDR 0x01 0x05 0x02 i
i2cset -y $I2C_DEV $DES_ADDR 0x01 0x00 0x03 i
i2cset -y $I2C_DEV $DES_ADDR 0x01 0x0F 0x25 i
i2cset -y $I2C_DEV $DES_ADDR 0x01 0x0A 0x15 i
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x31 0x02 i
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x32 0x10 i
# [Tx Register Setting]
i2cset -y $I2C_DEV $SER_ADDR 0x00 0xFE 0x11 i
# [Rx Register Setting only for Pass Through]
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x32 0x00 i
# [Tx Register Settings]
i2cset -y $I2C_DEV $SER_ADDR 0xF3 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0xF2 0x22 b
i2cset -y $I2C_DEV $SER_ADDR 0xF0 0x03 b
i2cset -y $I2C_DEV $SER_ADDR 0xFF 0x19 b
i2cset -y $I2C_DEV $SER_ADDR 0xF6 0x15 b
i2cset -y $I2C_DEV $SER_ADDR 0xC9 0x05 b
i2cset -y $I2C_DEV $SER_ADDR 0xCA 0x05 b
# [Tx Register Settings]
i2cset -y $I2C_DEV $SER_ADDR 0xFE 0x21 b
i2cset -y $I2C_DEV $SER_ADDR 0x76 0x10 b
# [PLL Settings for Pass Through mode]
i2cset -y $I2C_DEV $SER_ADDR 0x0F 0x01 b

if [ $CAM_MIPI_DATARATE -eq 1500 ]; then
    i2cset -y $I2C_DEV $SER_ADDR 0x11 0x29 b
    i2cset -y $I2C_DEV $SER_ADDR 0x12 0xAA b
    i2cset -y $I2C_DEV $SER_ADDR 0x13 0xAA b
    i2cset -y $I2C_DEV $SER_ADDR 0x14 0xAA b
    i2cset -y $I2C_DEV $SER_ADDR 0x15 0x43 b
    #echo "mipi 1500!!"
elif [ $CAM_MIPI_DATARATE -eq 1188 ]; then
    i2cset -y $I2C_DEV $SER_ADDR 0x11 0x2C b
    i2cset -y $I2C_DEV $SER_ADDR 0x12 0x00 b
    i2cset -y $I2C_DEV $SER_ADDR 0x13 0x00 b
    i2cset -y $I2C_DEV $SER_ADDR 0x14 0x00 b
    i2cset -y $I2C_DEV $SER_ADDR 0x15 0x44 b
    #echo "mipi 1188!!"
else
    echo "Error: Invalid CAM_MIPI_DATARATE value. EXIT!!"
    exit 1
fi

i2cset -y $I2C_DEV $SER_ADDR 0x16 0x01 b
i2cset -y $I2C_DEV $SER_ADDR 0x00 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0x01 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0x02 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0x55 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0x04 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0x2B 0x04 b
i2cset -y $I2C_DEV $SER_ADDR 0x2F 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0x2D 0x11 b
i2cset -y $I2C_DEV $SER_ADDR 0x2C 0x01 b
i2cset -y $I2C_DEV $SER_ADDR 0x05 0x01 b
i2cset -y $I2C_DEV $SER_ADDR 0x06 0x01 b
i2cset -y $I2C_DEV $SER_ADDR 0x27 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0x1D 0x00 b
i2cset -y $I2C_DEV $SER_ADDR 0x1E 0x00 b

if [ $IO_MODE -eq 1 ]; then
    i2cset -y $I2C_DEV $SER_ADDR 0x3D 0x0D b
    i2cset -y $I2C_DEV $SER_ADDR 0x3E 0x2C b
    i2cset -y $I2C_DEV $SER_ADDR 0x3F 0x0F b
    #echo "IO_MODE is 1!!"
else
    i2cset -y $I2C_DEV $SER_ADDR 0x3D 0x00 b
    i2cset -y $I2C_DEV $SER_ADDR 0x3E 0x24 b
    i2cset -y $I2C_DEV $SER_ADDR 0x3F 0x07 b
    #echo "IO_MODE is 0!!"
fi

# [ Rx Register Settings]
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x10 0x11 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x10 0xA1 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x11 0x06 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x12 0x00 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x21 0x20 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x22 0x02 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x23 0x11 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x24 0x00 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x25 0x00 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x26 0x00 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x27 0x07 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x28 0x02 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x30 0x00 i
i2cset -y $I2C_DEV $DES_ADDR 0x11 0x00 0x01 i
i2cset -y $I2C_DEV $DES_ADDR 0x11 0x01 0x01 i
i2cset -y $I2C_DEV $DES_ADDR 0x11 0x02 0x01 i
i2cset -y $I2C_DEV $DES_ADDR 0x16 0x00 0x1A i
i2cset -y $I2C_DEV $DES_ADDR 0x16 0x05 0x29 i
i2cset -y $I2C_DEV $DES_ADDR 0x16 0x06 0x44 i
i2cset -y $I2C_DEV $DES_ADDR 0x16 0x1F 0x00 i

if [ $CAM_MIPI_DATARATE -eq 1500 ]; then
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x09 0x0E i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0A 0x18 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0B 0x0C i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0D 0x11 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0E 0x06 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0F 0x09 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x10 0x05 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x11 0x1A i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x12 0x0D i
    #echo "mipi 1500!!"
elif [ $CAM_MIPI_DATARATE -eq 1188 ]; then
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x09 0x0B i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0A 0x12 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0B 0x0A i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0D 0x0E i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0E 0x03 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x0F 0x07 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x10 0x04 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x11 0x14 i
    i2cset -y $I2C_DEV $DES_ADDR 0x16 0x12 0x0B i
    #echo "mipi 1188!!"
else
    echo "Error: Invalid CAM_MIPI_DATARATE value. EXIT!!"
    exit 1
fi

i2cset -y $I2C_DEV $DES_ADDR 0x17 0x03 0x01 i
i2cset -y $I2C_DEV $DES_ADDR 0x17 0x04 0x11 i

if [ $IO_MODE -eq 1 ]; then
    i2cset -y $I2C_DEV $DES_ADDR 0x10 0x03 0x44 i
    i2cset -y $I2C_DEV $DES_ADDR 0x10 0x04 0x03 i
    #echo "IO_MODE is 1!!"
else
    i2cset -y $I2C_DEV $DES_ADDR 0x10 0x03 0x00 i
    i2cset -y $I2C_DEV $DES_ADDR 0x10 0x04 0x00 i
     #echo "IO_MODE is 0!!"
fi
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x1B 0x18 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x03 0x44 i
i2cset -y $I2C_DEV $DES_ADDR 0x10 0x04 0x33 i
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x1B 0x18 i
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x32 0x10 i
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x40 $CAM_ADDR i
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x41 $CAM_ADDR i

#I2C passthrough setting 2-bytes addr and 4-bytes value
i2cset -y $I2C_DEV $DES_ADDR 0x00 0x32 0x13 i

# End of file
