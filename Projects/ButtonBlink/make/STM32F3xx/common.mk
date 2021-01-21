export

SERIES_CPU  = cortex-m4
SERIES_ARCH = armv7e-m+fp

ifeq (STM32F301x6, $(DEVICE))
    MAPPED_DEVICE = STM32F301x8
endif

ifeq (STM32F302x6, $(DEVICE))
    MAPPED_DEVICE = STM32F302x8
endif

ifeq (STM32F302xB, $(DEVICE))
    MAPPED_DEVICE = STM32F302xC
endif

ifeq (STM32F302xD, $(DEVICE))
    MAPPED_DEVICE = STM32F302xE
endif

ifeq (STM32F303x6, $(DEVICE))
    MAPPED_DEVICE = STM32F303x8
endif

ifeq (STM32F303xB, $(DEVICE))
    MAPPED_DEVICE = STM32F303xC
endif

ifeq (STM32F303xD, $(DEVICE))
    MAPPED_DEVICE = STM32F303xE
endif

ifeq (STM32F318x8, $(DEVICE))
    MAPPED_DEVICE = STM32F318xx
endif

ifeq (STM32F328x8, $(DEVICE))
    MAPPED_DEVICE = STM32F328xx
endif

ifeq (STM32F334x4, $(DEVICE))
    MAPPED_DEVICE = STM32F334x8
endif

ifeq (STM32F334x6, $(DEVICE))
    MAPPED_DEVICE = STM32F334x8
endif

ifeq (STM32F358xC, $(DEVICE))
    MAPPED_DEVICE = STM32F358xx
endif

ifeq (STM32F373x8, $(DEVICE))
    MAPPED_DEVICE = STM32F373xC
endif

ifeq (STM32F373xB, $(DEVICE))
    MAPPED_DEVICE = STM32F373xC
endif

ifeq (STM32F378xC, $(DEVICE))
    MAPPED_DEVICE = STM32F378xx
endif

ifeq (STM32F398xE, $(DEVICE))
    MAPPED_DEVICE = STM32F398xx
endif