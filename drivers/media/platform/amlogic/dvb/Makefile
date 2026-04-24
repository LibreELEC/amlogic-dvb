obj-$(CONFIG_DVB_AMLOGIC) += amlogic-dvb.o
obj-$(CONFIG_DVB_AMLOGIC) += m88rs6060-fe.o

amlogic-dvb-y := \
	amlogic-dvb-asyncfifo.o \
	amlogic-dvb-core.o      \
	amlogic-dvb-debugfs.o   \
	amlogic-dvb-demux.o     \
	amlogic-dvb-dsc.o       \
	amlogic-dvb-frontend.o  \
	amlogic-dvb-gpio.o      \
	amlogic-dvb-pts.o       \
	amlogic-dvb-regmap.o    \
	amlogic-dvb-stats.o     \
	amlogic-dvb-ts.o

ccflags-y += -I$(src)
ccflags-y += -I$(srctree)/drivers/media/dvb-frontends
ccflags-y += -I$(srctree)/drivers/media/dvb-core
