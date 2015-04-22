# name of your application
APPLICATION = dummyradio

# If no BOARD is found in the environment, use this default:
BOARD ?= native

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEMODULE += vtimer
USEMODULE += dummyradio
USEMODULE += netdev_802154
USEMODULE += ieee802154

USEPKG += openwsn

include $(RIOTBASE)/Makefile.include

INCLUDES +=	-I$(RIOTBASE)/../openwsn/bsp/boards \
			-I$(RIOTBASE)/../openwsn/kernel \
			-I$(RIOTBASE)/../openwsn/openstack \
			-I$(RIOTBASE)/../openwsn/inc \
			-I$(RIOTBASE)/../openwsn/bsp/boards/dummyriot
