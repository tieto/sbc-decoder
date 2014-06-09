LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := test

LOCAL_CFLAGS += -DBUILDCFG -DHAS_BDROID_BUILDCFG

LOCAL_SRC_FILES := sbc_debug.c \
../decoder/srce/sbc_decoder.c

LOCAL_C_INCLUDES  += \
$(LOCAL_PATH)/../decoder/include \
$(LOCAL_PATH)/../encoder/include \
external/bluetooth/bluedroid/include \
external/bluetooth/bluedroid/stack/include \
device/sony/tsubasa/bluetooth \
external/bluetooth/bluedroid/gki/ulinux \
external/bluedroid/include

LOCAL_LDLIBS += -lpthread -ldl -llog -lreadline
LIBS_c += -lreadline

LOCAL_SHARED_LIBRARIES += libcutils   \
libutils    \
libhardware \
libhardware_legacy

include $(BUILD_EXECUTABLE)

#bt_target.h external/bluetooth/bluedroid/include \
#bt_types.h                     external/bluetooth/bluedroid/stack/include
#bdroid_buildcfg.h  device/sony/tsubasa/bluetooth
#data_type.h external/bluetooth/bluedroid/stack/include \