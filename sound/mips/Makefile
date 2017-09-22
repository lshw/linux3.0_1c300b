#
# Makefile for ALSA
#

snd-au1x00-objs := au1x00.o
snd-sgi-o2-objs := sgio2audio.o ad1843.o
snd-sgi-hal2-objs := hal2.o

# Toplevel Module Dependency
obj-$(CONFIG_SND_AU1X00) += snd-au1x00.o
obj-$(CONFIG_SND_SGI_O2) += snd-sgi-o2.o
obj-$(CONFIG_SND_SGI_HAL2) += snd-sgi-hal2.o


obj-$(CONFIG_SND_LS1A_PCM)      += snd-ls1a-pcm.o
snd-ls1a-pcm-objs               := ls1a-pcm.o

obj-$(CONFIG_SND_LS1A_LIB)      += snd-ls1a-lib.o
snd-ls1a-lib-y                  := ls1a-pcm-lib.o
snd-ls1a-lib-$(CONFIG_SND_LS1A_LIB_AC97)        += ls1a-ac97-lib.o

obj-$(CONFIG_SND_LS1A_AC97)     += snd-ls1a-ac97.o
snd-ls1a-ac97-objs              := ls1a-ac97.o

