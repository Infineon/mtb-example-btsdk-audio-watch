#
# Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#
ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif
#
# Basic Configuration
#
APPNAME=Watch
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYW920819EVB-02

SUPPORTED_TARGETS = \
  CYW920819EVB-02 \
  CYW920820EVB-02 \
  CYW920719B2Q40EVB-01 \
  CYW920706WCDEVAL \
  CYW9M2BASE-43012BT \
  CYW920721M2EVK-01 \
  CYW920721M2EVK-02 \
  CYW943012BTEVK-01 \
  CYW955572BTEVK-01 \
  CYW920721M2EVB-03 \
  CYW920820M2EVB-01

#
# Advanced Configuration
#
SOURCES=
INCLUDES=
DEFINES=
VFP_SELECT=
CFLAGS=
CXXFLAGS=
ASFLAGS=
LDFLAGS=
LDLIBS=
LINKER_SCRIPT=
PREBUILD=
POSTBUILD=
FEATURES=

#
# App features/defaults
#
OTA_FW_UPGRADE?=0
BT_DEVICE_ADDRESS?=default
UART?=AUTO
XIP?=xip
TRANSPORT?=UART
A2DP_SRC_INCLUDED := 1
A2DP_SNK_INCLUDED ?= 0
AVRCP_TG_INCLUDED := 1
AVRCP_CT_INCLUDED := 1
HFP_AG_INCLUDED ?= 0
HFP_HF_INCLUDED ?= 0
HCI_TEST_INCLUDED := 1
LE_INCLUDED := 1
ANCS_INCLUDED ?= 1
AMS_INCLUDED ?= 1
PANU_SUPPORT=0
PANNAP_SUPPORT=0
PAN_PTS_SUPPORT=0
AUDIO_SHIELD_20721M2EVB_03_INCLUDED?=0

# PAN only supported on CYW920721M2EVK-01 currently
ifeq (1,$(filter 1,$(PANU_SUPPORT) $(PANNAP_SUPPORT) $(PAN_PTS_SUPPORT)))
ifneq ($(TARGET),CYW920721M2EVK-01)
$(error PANU_INCLUDED PANNAP_INCLUDED and PAN_PTS_SUPPORT only supported on CYW920721M2EVK-01 TARGET)
endif
endif

ifeq ($(PAN_PTS_SUPPORT), 1)
PAN_PTS_INCLUDED := 1
endif
ifeq ($(PANU_SUPPORT), 1)
PANU_INCLUDED := 1
endif
ifeq ($(PANNAP_SUPPORT), 1)
PANNAP_INCLUDED := 1
endif
ifeq ($(PANU_INCLUDED), 1)
ifeq ($(PANNAP_INCLUDED), 1)
$(error PANU_INCLUDED PANNAP_INCLUDED can't enable at the same time)
endif
endif

AMA_INCLUDED := 0
MP3_DECODER_INCLUDED ?= 1
ENABLE_DEBUG?=0

SLEEP_SUPPORT ?= 0
COEX_SUPPORT ?= 0
AUTO_ELNA_SWITCH ?= 0
AUTO_EPA_SWITCH ?= 0

AUDIO_SHIELD_20721M2EVB_03_INCLUDED?=0

-include internal.mk

ifeq ($(TARGET),CYW920721M2EVB-03)
AUDIO_SHIELD_20721M2EVB_03_INCLUDED=1
endif

ifeq ($(TARGET),$(filter $(TARGET), CYW920719B2Q40EVB-01))
HFP_AG_INCLUDED = 0
endif

ifeq ($(TARGET),$(filter $(TARGET), CYW920721M2EVK-01 CYW920721M2EVK-02 CYW920721M2EVB-03 CYW9M2BASE-43012BT CYW943012BTEVK-01))
ifeq ($(HFP_AG_INCLUDED),1)
HFP_HF_INCLUDED = 0
else
HFP_AG_INCLUDED = 0
HFP_HF_INCLUDED = 1
endif
endif

ifeq ($(PAN_PTS_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_PAN_PTS_INCLUDED
endif
ifeq ($(PANU_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_PANU_INCLUDED
endif
ifeq ($(PANNAP_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_PANNAP_INCLUDED
endif

# wait for SWD attach
ifeq ($(ENABLE_DEBUG),1)
CY_APP_DEFINES+=-DENABLE_DEBUG=1
endif

CY_APP_DEFINES += -DWICED_BT_TRACE_ENABLE
CY_APP_DEFINES += -DWICED_HCI_TRANSPORT_SPI=2
CY_APP_DEFINES += -DWICED_HCI_TRANSPORT_UART=1
CY_APP_DEFINES += -DSLEEP_SUPPORTED=$(SLEEP_SUPPORT)
CY_APP_DEFINES += -DCOEX_SUPPORTED=$(COEX_SUPPORT)
#CY_APP_DEFINES += -DPTS_TEST_ONLY
ifeq ($(TRANSPORT),SPI)
#$(info Transport=SPI)
ifeq ($(TARGET),$(filter $(TARGET), CYW920706WCDEVAL CYW920719B2Q40EVB-01 CYW920721M2EVK-01 CYW920721M2EVK-02 CYW920721M2EVB-03))
CY_APP_DEFINES += -DWICED_HCI_TRANSPORT=2
else
$(error invalid TARGET for SPI transport, supported only for CYW920706WCDEVAL and CYW920719B2Q40EVB-01)
endif
else
#$(info Transport=UART)
CY_APP_DEFINES += -DWICED_HCI_TRANSPORT=1
endif # TRANSPORT

ifeq ($(A2DP_SRC_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_AUDIO_SRC_INCLUDED
endif

ifeq ($(A2DP_SNK_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_AUDIO_SNK_INCLUDED
CY_APP_DEFINES += -DA2DP_SINK_ENABLE_CONTENT_PROTECTION
CY_APP_DEFINES += -DWICED_A2DP_EXT_CODEC=0
ifeq ($(A2DP_SRC_INCLUDED), 1)
	# A2DP / AVRCP Role Switch Method: Default is A2DP_SINK + AVRCP_CT
	CY_APP_DEFINES += -DWICED_APP_AUDIO_ROLE_SERVICE_SWITCH_WITH_SNK
endif
endif

ifeq ($(AVRCP_TG_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_AUDIO_RC_TG_INCLUDED
CY_APP_DEFINES += -DCATEGORY_2_PASSTROUGH
endif

ifeq ($(AVRCP_CT_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_AUDIO_RC_CT_INCLUDED
CY_APP_DEFINES += -DMAX_CONNECTED_RCC_DEVICES=1
endif

ifeq ($(HFP_AG_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_HFP_AG_INCLUDED
endif

ifeq ($(HFP_HF_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_HFP_HF_INCLUDED
endif

ifeq ($(LE_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_LE_INCLUDED
CY_APP_DEFINES += -DWICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
endif

ifeq ($(ANCS_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_ANCS_INCLUDED
endif

ifeq ($(AMS_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_AMS_INCLUDED
endif

ifeq ($(AMA_INCLUDED), 1)
CY_APP_DEFINES += -DAMA_ENABLED
CY_APP_DEFINES += -DAMA_SPEECH_AUDIO_FORMAT_OPUS_BITRATE=0
CY_APP_DEFINES += -DAMA_VOICE_BUFFER_LENGTH_IN_MS=500
ifeq ($(A2DP_SNK_INCLUDED), 1)
CY_APP_DEFINES += -DAMA_AUDIO_SNK_INCLUDED
endif
ifeq ($(HFP_HF_INCLUDED), 1)
CY_APP_DEFINES += -DAMA_HANDSFREE_INCLUDED
endif
endif

ifeq ($(HCI_TEST_INCLUDED), 1)
CY_APP_DEFINES += -DWICED_APP_TEST_INCLUDED
endif

#
# Patch Library for different target
#
ifeq ($(TRANSPORT),SPI)
CY_20706A2_APP_PATCH_LIBS += wiced_hal_lib.a
endif
ifeq ($(A2DP_SRC_INCLUDED), 1)
CY_20706A2_APP_PATCH_LIBS += wiced_audio_source.a
endif

ifeq ($(TARGET),CYW943012EVB-04-BT)
CY_APP_DEFINES += -DWICED_HCI_BAUDRATE=115200
endif # 43012EVK-04-BT

CY_20819A1_APP_PATCH_LIBS += wiced_ble_pre_init_lib.a
CY_20820A1_APP_PATCH_LIBS += wiced_ble_pre_init_lib.a

ifeq ($(A2DP_SRC_INCLUDED), 1)
CY_20719B2_APP_PATCH_LIBS += wiced_audio_source_lib.a
CY_20721B2_APP_PATCH_LIBS += wiced_audio_source_lib.a
endif
ifeq ($(A2DP_SNK_INCLUDED), 1)
CY_20719B2_APP_PATCH_LIBS += wiced_audio_sink_lib.a
CY_20721B2_APP_PATCH_LIBS += wiced_audio_sink_lib.a
endif
ifeq ($(AMA_INCLUDED), 1)
CY_20719B2_APP_PATCH_LIBS += i2s_aud_record_lib.a
CY_20721B2_APP_PATCH_LIBS += i2s_aud_record_lib.a
endif

ifeq ($(A2DP_SRC_INCLUDED), 1)
CY_43012C0_APP_PATCH_LIBS += wiced_audio_sink_lib.a
endif
ifneq ($(filter 1, $(HFP_AG_INCLUDED) $(HFP_HF_INCLUDED)), )
CY_43012C0_APP_PATCH_LIBS += wiced_sco_lib.a
endif

#
# Components (middleware libraries)
#
COMPONENTS += bsp_design_modus

ifeq ($(AUDIO_SHIELD_20721M2EVB_03_INCLUDED),1)
DISABLE_COMPONENTS += bsp_design_modus
COMPONENTS += bsp_design_modus_shield
endif

ifeq ($(A2DP_SNK_INCLUDED), 1)
COMPONENTS += a2dp_sink_profile
endif

ifeq ($(AVRCP_TG_INCLUDED), 1)
COMPONENTS += avrc_target
endif

ifeq ($(AVRCP_CT_INCLUDED), 1)
COMPONENTS += avrc_controller
endif

ifeq ($(HFP_AG_INCLUDED), 1)
COMPONENTS += hfp_audio_gateway
endif

ifeq ($(HFP_HF_INCLUDED), 1)
COMPONENTS += handsfree_profile
endif

ifeq ($(AMS_INCLUDED), 1)
COMPONENTS += ams
endif

ifeq ($(ANCS_INCLUDED), 1)
COMPONENTS += ancs
endif

ifeq ($(AMA_INCLUDED), 1)
COMPONENTS += ama
COMPONENTS += audio_record_lib
COMPONENTS += nanopbuf
endif

ifneq ($(filter 1, $(PANU_INCLUDED) $(PANNAP_INCLUDED)), )
COMPONENTS += pan_lib
endif

#
# Target Configuration
#
ifeq ($(TARGET),CYW920721M2EVK-01)
COMPONENTS += audiomanager
COMPONENTS += cyw9bt_audio2
COMPONENTS += codec_cs47l35_lib
CY_APP_DEFINES += -DCS47L35_CODEC_ENABLE
CY_APP_DEFINES += -DCYW9BT_AUDIO
CY_APP_PATCH_LIBS += wiced_mem_lib.a
AUTO_ELNA_SWITCH = 1
endif # TARGET

ifneq ($(filter $(TARGET),CYW920721M2EVK-02 CYW920721M2EVB-03),)
COMPONENTS += audiomanager
COMPONENTS += cyw9bt_audio2
COMPONENTS += codec_cs47l35_lib
CY_APP_DEFINES += -DCS47L35_CODEC_ENABLE
CY_APP_DEFINES += -DCYW9BT_AUDIO
CY_APP_PATCH_LIBS += wiced_mem_lib.a
AUTO_ELNA_SWITCH = 1
endif # TARGET

ifeq ($(TARGET),CYW920721M2EVB-03)
AUDIO_SHIELD_20721M2EVB_03_INCLUDED=1
endif

ifeq ($(AUDIO_SHIELD_20721M2EVB_03_INCLUDED),1)
DISABLE_COMPONENTS += bsp_design_modus
COMPONENTS += bsp_design_modus_shield
endif

ifeq ($(TARGET),$(filter $(TARGET), CYW9M2BASE-43012BT))
COMPONENTS += audiomanager
COMPONENTS += cyw9bt_audio
COMPONENTS += codec_ak4679_lib
CY_APP_DEFINES += -DAK_4679_CODEC_ENABLE
CY_APP_DEFINES += -DCYW9BT_AUDIO
OTA_FW_UPGRADE = 0
SLEEP_SUPPORT = 0
endif # TARGET

ifeq ($(TARGET),CYW943012BTEVK-01)
COMPONENTS += audiomanager
COMPONENTS += cyw9bt_audio2
COMPONENTS += codec_cs47l35_lib
CY_APP_DEFINES += -DCS47L35_CODEC_ENABLE
CY_APP_DEFINES += -DCYW9BT_AUDIO
OTA_FW_UPGRADE = 0
SLEEP_SUPPORT = 0
endif # TARGET

ifeq ($(TARGET),CYW955572BTEVK-01)
CY_APP_DEFINES += -DAPP_CFG_ENABLE_BR_AUDIO=1
# Apply new Audio Profiles
ifeq ($(A2DP_SNK_INCLUDED), 1)
DISABLE_COMPONENTS += a2dp_sink_profile
COMPONENTS += a2dp_sink_profile_btstack
COMPONENTS += audio_sink_route_config_lib
endif
ifeq ($(AVRCP_CT_INCLUDED), 1)
DISABLE_COMPONENTS += avrc_controller
COMPONENTS += avrc_controller_btstack
endif
ifeq ($(AVRCP_TG_INCLUDED), 1)
DISABLE_COMPONENTS += avrc_target
COMPONENTS += avrc_target_btstack
endif
ifeq ($(HFP_HF_INCLUDED), 1)
DISABLE_COMPONENTS += handsfree_profile
COMPONENTS += handsfree_profile_btstack
CY_APP_DEFINES += -DWICED_BT_HFP_HF_WBS_INCLUDED=TRUE
endif
endif # TARGET

ifeq ($(TARGET),$(filter $(TARGET), CYW920721M2EVK-01 CYW920721M2EVK-02 CYW920721M2EVB-03))
ifeq ($(MP3_DECODER_INCLUDED), 1)
COMPONENTS += mp3_decoder_lib
CY_APP_DEFINES += -DMP3_DECODER_INCLUDED
endif # MP3_DECODER_INCLUDED
endif # 20721B2

ifeq ($(AUTO_ELNA_SWITCH),1)
CY_APP_DEFINES += -DAUTO_ELNA_SWITCH
endif

ifeq ($(AUTO_EPA_SWITCH),1)
CY_APP_DEFINES += -DAUTO_EPA_SWITCH
endif

ifeq ($(OTA_FW_UPGRADE),1)
CY_APP_DEFINES += -DOTA_FW_UPGRADE=1
COMPONENTS += fw_upgrade_lib
OTA_SEC_FW_UPGRADE ?= 0
ifeq ($(OTA_SEC_FW_UPGRADE), 1)
    CY_APP_DEFINES += -DOTA_SECURE_FIRMWARE_UPGRADE
endif
endif

################################################################################
# Paths
################################################################################

# Path (absolute or relative) to the project
CY_APP_PATH=.

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI><COMMIT><LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler (Default: GCC in the tools)
CY_COMPILER_PATH=

# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_* \
    $(CY_IDE_TOOLS_DIR))

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

# tools that can be launched with "make open CY_OPEN_TYPE=<tool>
CY_BT_APP_TOOLS=BTSpy ClientControl

ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
$(error TARGET $(TARGET) not supported for this application. Edit SUPPORTED_TARGETS in the code example makefile to add new BSPs)
endif
include $(CY_TOOLS_DIR)/make/start.mk
