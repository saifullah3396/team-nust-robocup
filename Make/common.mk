ifdef DEBUG
CONFFLAGS=-DCMAKE_BUILD_TYPE=Debug
else
CONFFLAGS=-DCMAKE_BUILD_TYPE=Release
endif

ifdef CROSS
CONFFLAGS+=-DMODULE_IS_REMOTE=OFF
TOOLCHAIN_FLAG=-c ${CROSS}
CONFFLAGS+=-DTOOLCHAIN_NAME=${CROSS}
else
ifdef REMOTE
CONFFLAGS+=-DMODULE_IS_REMOTE=ON
TOOLCHAIN_FLAG=-c ${REMOTE}
CONFFLAGS+=-DTOOLCHAIN_NAME=${REMOTE}
else
$(error Please define either CROSS=<cross toolchain name> or REMOTE=<remote toolchain name> to continue)
endif
endif

ifdef DEBUG
BUILD_PREFIX=Debug
else
BUILD_PREFIX=Release
endif
