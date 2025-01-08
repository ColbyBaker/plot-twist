idf_component_register(SRCS "esp_pn532.cpp" "PN532.cpp" "PN532_SPI.cpp"
					   INCLUDE_DIRS "./include"
					   REQUIRES driver
					   PRIV_INCLUDE_DIRS
					   PRIV_REQUIRES
						)