# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/dhruv/esp/esp-idf/components/bootloader/subproject"
  "/Users/dhruv/esp/functional_brain_c6/build/bootloader"
  "/Users/dhruv/esp/functional_brain_c6/build/bootloader-prefix"
  "/Users/dhruv/esp/functional_brain_c6/build/bootloader-prefix/tmp"
  "/Users/dhruv/esp/functional_brain_c6/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/dhruv/esp/functional_brain_c6/build/bootloader-prefix/src"
  "/Users/dhruv/esp/functional_brain_c6/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/dhruv/esp/functional_brain_c6/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/dhruv/esp/functional_brain_c6/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
