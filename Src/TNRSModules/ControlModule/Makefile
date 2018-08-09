include ${PATH_TO_TEAM_NUST_DIR}/Make/common.mk

.PHONY: tests clean qi

configure-tests:
	qibuild configure -DBUILD_TNRS_TESTS=ON ${CONFFLAGS} ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}

tests:
	qibuild make ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}

clean:
ifdef CROSS
	rm -rf ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}/build-${CROSS}/tnrs-control-module
endif
ifdef REMOTE
	rm -rf ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}/build-${REMOTE}/tnrs-control-module
endif
	
configure:
	qibuild configure -DBUILD_TNRS_TESTS=OFF ${CONFFLAGS} ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}

install:
	qibuild make ${TOOLCHAIN_FLAG} --build-prefix ${PATH_TO_TEAM_NUST_DIR}/build/${BUILD_PREFIX}
	
