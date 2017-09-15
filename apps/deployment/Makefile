CONTIKI_PROJECT = deployment-test
CONTIKI_WITH_RIME = 1

DEFINES += PROJECT_CONF_H=\"project-conf.h\"

PROJECTDIRS += ../deployment-test
PROJECT_SOURCEFILES += deployment.c

include ../../Makefile.lwb

all: $(CONTIKI_PROJECT)
CONTIKI = ../../contiki
include $(CONTIKI)/Makefile.include
