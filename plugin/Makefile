SOURCE_DIR = $(PWD)

ifndef OPENHRPHOME
OPENHRPHOME=../../../..
endif

ifndef ROBOT
ROBOT=HRP2JRL
endif


SYSDIR=$(OPENHRPHOME)/Controller/IOserver/sys/plugin

sample:
	cd $(SYSDIR);$(MAKE) -f $(SOURCE_DIR)/Makefile.user SOURCE_DIR=$(SOURCE_DIR) ROBOT=$(ROBOT) user_all
sample_clean:
	cd $(SYSDIR);$(MAKE) -f $(SOURCE_DIR)/Makefile.user SOURCE_DIR=$(SOURCE_DIR) ROBOT=$(ROBOT) user_clean
sample_install:
	cd $(SYSDIR);$(MAKE) -f $(SOURCE_DIR)/Makefile.user SOURCE_DIR=$(SOURCE_DIR) ROBOT=$(ROBOT) user_install

echo:
	cd $(SYSDIR);$(MAKE) -f $(SOURCE_DIR)/Makefile.echo SOURCE_DIR=$(SOURCE_DIR) ROBOT=$(ROBOT)
echo_clean:
	cd $(SYSDIR);$(MAKE) -f $(SOURCE_DIR)/Makefile.echo SOURCE_DIR=$(SOURCE_DIR) ROBOT=$(ROBOT) user_clean

install:sample_install
clean:sample_clean echo_clean
