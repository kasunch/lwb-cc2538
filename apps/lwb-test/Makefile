CONTIKI_PROJECT = lwb-test
CONTIKI_WITH_RIME = 1

DEFINES += PROJECT_CONF_H=\"project-conf.h\" LWB_CUSTOM_CONF_H=\"lwb-custom-conf.h\"

PROJECTDIRS += ../deployment
PROJECT_SOURCEFILES += deployment.c lwb-debug-print.c

include ../../Makefile.lwb

all: $(CONTIKI_PROJECT)
CONTIKI = ../../contiki
include $(CONTIKI)/Makefile.include
