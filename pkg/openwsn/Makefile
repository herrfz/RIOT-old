PKG_NAME=openwsn
PKG_URL=https://github.com/herrfz/OpenWSN-RIOT.git
PKG_DIR=../../../$(PKG_NAME)

ifneq ($(RIOTBOARD),)
include $(RIOTBOARD)/$(BOARD)/Makefile.include
endif

.PHONY: all clean patch reset

all: patch
	@echo "Compiling OpenWSN..."
	"$(MAKE)" -C $(PKG_DIR)

patch:
	test -d "$(PKG_DIR)" || git clone "$(PKG_URL)" "$(PKG_DIR)"

clean:
	@echo "Cleaning up OpenWSN package..."
	@cd "$(PKG_DIR)" 2> /dev/null > /dev/null && \
		git clean -x -f && \
		git am --abort ; \
		git reset --hard "$(PKG_VERSION)" && \
		$(MAKE) patch || true

distclean:
	rm -rf "$(PKG_DIR)"

Makefile.include:
	@true
