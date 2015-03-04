# name of your application
APPLICATION = pingpong_at86rf231

# If no BOARD is found in the environment, use this default:
BOARD ?= vesna

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEMODULE += vtimer
USEMODULE += ps
USEMODULE += transceiver
USEMODULE += at86rf231
USEMODULE += netdev_802154
USEMODULE += ieee802154
#USEMODULE += config


include $(RIOTBASE)/Makefile.include
