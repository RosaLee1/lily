COMPONENT=RssiToSerialC

CFLAGS += -DCC2420_DEF_CHANNEL=16
CFLAGS += -DPRINTF_BUFFER_SIZE=250

PFLAGS += -I$(TOSROOT)/tos/lib/printf
PFLAGS += -DNEW_PRINTF_SEMANTICS

BUILD_EXTRA_DEPS = RssiSerialMsg.java SpecAnalyzer.class
CLEAN_EXTRA = *.class RssiSerialMsg.java

RssiSerialMsg.java: RssiToSerial.h
	mig java -target=$(PLATFORM) $(CFLAGS) -java-classname=RssiSerialMsg RssiToSerial.h rssi_serial_msg -o $@

SpecAnalyzer.class: $(wildcard *.java) SpecAnalyzer.java
	javac *.java

include $(MAKERULES)

