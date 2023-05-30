KERNEL_ROOT=/lib/modules/$(shell uname -r)/build 
PWD=$(shell pwd)

obj-m := veye_mvcam.o veyecam2m.o csimx307.o cssc132.o veye_vbyone.o

all:
	make -C $(KERNEL_ROOT) M=$(PWD) modules

clean:
	make -C $(KERNEL_ROOT) M=$(PWD) clean
