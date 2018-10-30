DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
DEFINES+=APROS_SERIAL_LINE_H=\"apros-serial-line.h\"
DEFINES+=APROS_GPIO_CONTROL_H=\"apros-gpio-control.h\"
DEFINES+=APROS_PACKET_CONTROL_H=\"apros-packet-control.h\"
CONTIKI_PROJECT = cc13xx-node

all: $(CONTIKI_PROJECT)

PROJECT_SOURCEFILES += apros-serial-line.c apros-gpio-control.c apros-packet-control.c

CONTIKI_WITH_RIME = 1

CONTIKI = ../../../..
include $(CONTIKI)/Makefile.include
