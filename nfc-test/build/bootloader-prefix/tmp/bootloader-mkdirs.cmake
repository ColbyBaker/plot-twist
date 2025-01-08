# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/colbybaker/esp-adf-env/esp-adf/esp-idf/components/bootloader/subproject"
  "/Users/colbybaker/esp-adf-env/nfc-test/build/bootloader"
  "/Users/colbybaker/esp-adf-env/nfc-test/build/bootloader-prefix"
  "/Users/colbybaker/esp-adf-env/nfc-test/build/bootloader-prefix/tmp"
  "/Users/colbybaker/esp-adf-env/nfc-test/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/colbybaker/esp-adf-env/nfc-test/build/bootloader-prefix/src"
  "/Users/colbybaker/esp-adf-env/nfc-test/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/colbybaker/esp-adf-env/nfc-test/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/colbybaker/esp-adf-env/nfc-test/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
