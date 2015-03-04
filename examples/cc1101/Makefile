# name of your application
APPLICATION = cc1101

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

#USEMODULE += vtimer
#USEMODULE += cc110x_legacy
#USEMODULE += transceiver
USEMODULE += at86rf231
#USEMODULE += ieee802154

include $(RIOTBASE)/Makefile.include
