obj-m := fpga_step_motor_driver.o fpga_push_switch_driver.o sonic_driver.o fpga_buzzer_driver.o ofpga_text_lcd_driver.o

KDIR :=/root/work/achroimx6q/kernel
PWD :=$(shell pwd)

all: driver app
driver:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules ARCH=arm
app:
	arm-none-linux-gnueabi-g++ -I/root/work/OPENCV/opencv-2.4.13/build/install/include -L/root/work/OPENCV/opencv-2.4.13/build/install/lib final.c -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lrt -lpthread -lm -ldl -o final
clean:
	rm -rf *.ko
	rm -rf *.mod.*
	rm -rf *.o
	rm -rf final
	rm -rf Module.symvers
	rm -rf modules.order
	rm -rf .fnd*
	rm -rf .tmp*
