COMPONENT=RssiBaseAppC

PFLAGS += -I$(TOSROOT)/tos/lib/printf
PFLAGS += -DNEW_PRINTF_SEMANTICS

INCLUDES= -I..               \
          -I../InterceptBase

CFLAGS += $(INCLUDES)
CFLAGS += -DCC2420_DEF_CHANNEL=16

ifneq ($(filter iris,$(MAKECMDGOALS)),) 
	CFLAGS += -DRF230_RSSI_ENERGY
endif

include $(MAKERULES)
