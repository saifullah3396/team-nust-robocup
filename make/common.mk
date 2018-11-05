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
		ifdef SIMULATION
			CONFFLAGS+=-DMODULE_IS_REMOTE=OFF
			CONFFLAGS+=-DMODULE_IS_LOCAL_SIMULATED=ON
			TOOLCHAIN_FLAG=-c ${SIMULATION}
			CONFFLAGS+=-DTOOLCHAIN_NAME=${SIMULATION}
		else
			$(error Please define either CROSS=<cross toolchain name>, REMOTE=<remote toolchain name> or SIMULATION=<sim toolchain name> to continue)
		endif
	endif
endif

ifdef DEBUG
	BUILD_PREFIX=Debug
else
	BUILD_PREFIX=Release
endif