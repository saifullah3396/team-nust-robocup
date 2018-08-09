ifdef DEBUG
FLAGS=DEBUG=1
endif

ifdef CROSS
FLAGS+=CROSS=${CROSS}
else
ifdef REMOTE
FLAGS+=REMOTE=${REMOTE} 
else
$(error Please define either =<cross toolchain name> or =<remote toolchain name> to continue)
endif
endif

MAKE_FLAGS=--no-print-directory

.PHONY: tests clean all

configure-tests:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL

tests:
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL

clean:
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule
	$(MAKE) clean -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL

configure:
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL ${MAKE_FLAGS}

all:
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL ${MAKE_FLAGS}
	
Resources:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Resources/RandomLib ${MAKE_FLAGS}
	
Utils-tests:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils ${MAKE_FLAGS}
	
Utils:
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/Utils ${MAKE_FLAGS}
	
ConfigManager:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ConfigManager ${MAKE_FLAGS}
	
MemoryModule-tests:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule ${MAKE_FLAGS}
	
MemoryModule:
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/MemoryModule ${MAKE_FLAGS}
	
DebugModule:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/DebugModule ${MAKE_FLAGS}
	
CommModule:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/CommModule ${MAKE_FLAGS}

ControlModule:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ControlModule ${MAKE_FLAGS}
	
BehaviorManager:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/BehaviorManager ${MAKE_FLAGS}
	
PlanningModule:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/PlanningModule ${MAKE_FLAGS}
	
MotionModule-tests:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule ${MAKE_FLAGS}
	
MotionModule:
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/MotionModule ${MAKE_FLAGS}

SBModule:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/SBModule ${MAKE_FLAGS}
	
LocalizationModule:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/LocalizationModule ${MAKE_FLAGS}
	
VisionModule:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingChilds/VisionModule ${MAKE_FLAGS}
	
ProcessingModule:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/ProcessingModule ${MAKE_FLAGS}
	
TeamNUSTSPL:
	$(MAKE) configure-tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL ${MAKE_FLAGS}
	$(MAKE) tests -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL ${MAKE_FLAGS}
	$(MAKE) configure -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL ${MAKE_FLAGS}
	$(MAKE) install -C ${PATH_TO_TEAM_NUST_DIR}/Src/TeamNUSTSPL ${MAKE_FLAGS}

