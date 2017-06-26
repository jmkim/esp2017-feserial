ifneq (${KERNELRELEASE},)
	obj-m := feserial.o
else
	KERNEL_SOURCE := /usr/local/src/linux-stable
	PWD := $(shell pwd)
default:
	${MAKE} -C ${KERNEL_SOURCE} M=${PWD} modules

clean:
	${MAKE} -C ${KERNEL_SOURCE} M=${PWD} clean
endif
