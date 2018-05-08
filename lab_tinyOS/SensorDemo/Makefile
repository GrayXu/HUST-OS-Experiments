CFLAGS += -I$(TOSDIR)/lib/printf
PFLAGS += -DNEW_PRINTF_SEMANTICS
CFLAGS += -DPRINTF_BUFFER_SIZE=128
COMPONENT=SensorAppC

BUILD_EXTRA_DEPS += SensorMsg.class
CLEAN_EXTRA = *.class SensorMsg.java

SensorMsg.class: SensorMsg.java
	javac SensorMsg.java

SensorMsg.java:
	mig java -target=$(PLATFORM) -java-classname=SensorMsg Sense.h SensorMsg -o $@

include $(MAKERULES)

