deps_config := \
	/home/zeno/esp/esp-idf/components/app_trace/Kconfig \
	/home/zeno/esp/esp-idf/components/aws_iot/Kconfig \
	/home/zeno/esp/esp-idf/components/bt/Kconfig \
	/home/zeno/esp/esp-idf/components/driver/Kconfig \
	/home/zeno/esp/esp-idf/components/esp32/Kconfig \
	/home/zeno/esp/esp-idf/components/esp_adc_cal/Kconfig \
	/home/zeno/esp/esp-idf/components/esp_event/Kconfig \
	/home/zeno/esp/esp-idf/components/esp_http_client/Kconfig \
	/home/zeno/esp/esp-idf/components/esp_http_server/Kconfig \
	/home/zeno/esp/esp-idf/components/ethernet/Kconfig \
	/home/zeno/esp/esp-idf/components/fatfs/Kconfig \
	/home/zeno/esp/esp-idf/components/freemodbus/Kconfig \
	/home/zeno/esp/esp-idf/components/freertos/Kconfig \
	/home/zeno/esp/esp-idf/components/heap/Kconfig \
	/home/zeno/esp/esp-idf/components/libsodium/Kconfig \
	/home/zeno/esp/esp-idf/components/log/Kconfig \
	/home/zeno/esp/esp-idf/components/lwip/Kconfig \
	/home/zeno/esp/esp-idf/components/mbedtls/Kconfig \
	/home/zeno/esp/esp-idf/components/mdns/Kconfig \
	/home/zeno/esp/esp-idf/components/mqtt/Kconfig \
	/home/zeno/esp/esp-idf/components/nvs_flash/Kconfig \
	/home/zeno/esp/esp-idf/components/openssl/Kconfig \
	/home/zeno/esp/esp-idf/components/pthread/Kconfig \
	/home/zeno/esp/esp-idf/components/spi_flash/Kconfig \
	/home/zeno/esp/esp-idf/components/spiffs/Kconfig \
	/home/zeno/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/zeno/esp/esp-idf/components/vfs/Kconfig \
	/home/zeno/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/zeno/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/zeno/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/zeno/Documents/MotionTracking_1.2/main/Kconfig.projbuild \
	/home/zeno/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/zeno/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
