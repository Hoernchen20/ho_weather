# Set the name of your application:
APPLICATION = ho_weather

# If no BOARD is found in the environment, use this default:
BOARD ?= nucleo-l073rz

DEFAULT_PERIOD_SENDING ?= 300
DEFAULT_RESOLUTION ?= 1

# BME280 config
CFLAGS += -DBMX280_PARAM_I2C_DEV=I2C_DEV\(0\)
CFLAGS += -DBMX280_PARAM_I2C_ADDR=0x76

# Lorawan config
-include lorawan.credentials
LORA_REGION ?= EU868

# defines
CFLAGS += -DDEFAULT_PERIOD_SENDING=$(DEFAULT_PERIOD_SENDING)
CFLAGS += -DDEFAULT_RESOLUTION=$(DEFAULT_RESOLUTION)
CFLAGS += -DDEVEUI=\"$(DEVEUI)\" -DAPPEUI=\"$(APPEUI)\" -DAPPKEY=\"$(APPKEY)\"
CFLAGS += -DDISABLE_LORAMAC_DUTYCYCLE

CFLAGS += "-DSX127X_PARAM_SPI=(SPI_DEV(0))"
CFLAGS += "-DSX127X_PARAM_SPI_NSS=GPIO_PIN(PORT_A, 4)"
CFLAGS += "-DSX127X_PARAM_RESET=GPIO_PIN(PORT_C, 9)"
CFLAGS += "-DSX127X_PARAM_DIO0=GPIO_PIN(PORT_C, 6)"
CFLAGS += "-DSX127X_PARAM_DIO1=GPIO_PIN(PORT_C, 7)"
CFLAGS += "-DSX127X_PARAM_DIO2=GPIO_PIN(PORT_C, 8)"
CFLAGS += "-DSX127X_PARAM_DIO3=GPIO_UNDEF"
CFLAGS += "-DSX127X_PARAM_PASELECT=(SX127X_PA_BOOST)"

# Uncomment this to enable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP = 1

# Change this to 0 to show compiler invocation lines by default:
QUIET ?= 1

# Modules to include:
USEPKG += semtech-loramac
USEMODULE += bme280_i2c
USEMODULE += fmt
USEMODULE += ho_scaling
USEMODULE += semtech_loramac_rx
USEMODULE += sx1276
USEMODULE += xtimer

FEATURES_REQUIRED += periph_adc
FEATURES_REQUIRED += periph_gpio
FEATURES_REQUIRED += periph_rtc

# Specify custom dependencies for your application here ...
INCLUDES += -I$(CURDIR)/ho_helper_functions/scaling
DIRS += $(CURDIR)/ho_helper_functions/scaling
#APPDEPS = scaling.h

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/RIOT

include $(RIOTBASE)/Makefile.include
