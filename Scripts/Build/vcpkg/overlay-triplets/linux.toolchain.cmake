# Waiting vars:
#     - UE_ENGINE

if(NOT _VCPKG_UE_LINUX_TOOLCHAIN)

  set(_VCPKG_UE_LINUX_TOOLCHAIN 1)
  
  # Engine/Source/ThirdParty location
  set (UE_THIRD_PARTY $ENV{UE_ENGINE}/Source/ThirdParty/)
  # Full path to toolchain root
  set (LINUX_MULTIARCH_ROOT $ENV{UE_ENGINE}/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v22_clang-16.0.6-centos7)
  set(ARCHITECTURE_TRIPLE "x86_64-unknown-linux-gnu" CACHE STRING "")
  set(UE_INCLUDE ${UE_THIRD_PARTY}/Unix/LibCxx/include)
  set(UE_LIBS ${UE_THIRD_PARTY}/Unix/LibCxx/lib/Unix/${ARCHITECTURE_TRIPLE})

  set(CMAKE_CROSSCOMPILING ON CACHE BOOL "")

  message (STATUS "CMAKE_HOST_SYSTEM_NAME is '${CMAKE_HOST_SYSTEM_NAME}'")
  message (STATUS "CMAKE_SYSTEM_NAME is '${CMAKE_SYSTEM_NAME}'")
  message (STATUS "LINUX_MULTIARCH_ROOT is '${LINUX_MULTIARCH_ROOT}'")
  message (STATUS "VCPKG_TARGET_ARCHITECTURE is '${VCPKG_TARGET_ARCHITECTURE}'")
  message (STATUS "ARCHITECTURE_TRIPLE is '${ARCHITECTURE_TRIPLE}'")

  # sysroot
  set(CMAKE_SYSROOT ${LINUX_MULTIARCH_ROOT}/${ARCHITECTURE_TRIPLE} CACHE PATH "")

  set(CMAKE_LIBRARY_ARCHITECTURE ${ARCHITECTURE_TRIPLE} CACHE STRING "")

  set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

  # specify the cross compiler
  set(CMAKE_INSTALL_OLDINCLUDEDIR ${CMAKE_SYSROOT}/usr/include CACHE STRING "")

  set(CMAKE_C_COMPILER_TARGET     ${ARCHITECTURE_TRIPLE} CACHE STRING "")
  set(CMAKE_CXX_COMPILER_TARGET   ${ARCHITECTURE_TRIPLE} CACHE STRING "")

  set(CMAKE_CXX_STANDARD 17 CACHE STRING "")

  # Compile options
  include_directories(BEFORE SYSTEM ${UE_INCLUDE} ${UE_INCLUDE}/c++/v1/ )
  add_compile_options(-fPIC -fms-extensions -fno-math-errno -fdiagnostics-absolute-paths)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-stdlib=libc++>)
  add_compile_options(-mssse3)

  # Link options
  add_link_options(-stdlib=libc++ -L${UE_LIBS} ${UE_LIBS}/libc++.a ${UE_LIBS}/libc++abi.a )
  
  set(CMAKE_AR                    ${CMAKE_SYSROOT}/bin/llvm-ar CACHE FILEPATH "")
  set(CMAKE_ASM_COMPILER          ${CMAKE_SYSROOT}/bin/clang CACHE FILEPATH "")
  set(CMAKE_C_COMPILER            ${CMAKE_SYSROOT}/bin/clang CACHE FILEPATH "")
  set(CMAKE_C_COMPILER_AR         ${CMAKE_SYSROOT}/bin/llvm-ar CACHE FILEPATH "")
  set(CMAKE_CXX_COMPILER          ${CMAKE_SYSROOT}/bin/clang++ CACHE FILEPATH "")
  set(CMAKE_CXX_COMPILER_AR       ${CMAKE_SYSROOT}/bin/llvm-ar CACHE FILEPATH "")
  set(CMAKE_OBJCOPY               ${CMAKE_SYSROOT}/bin/llvm-objcopy CACHE FILEPATH "")

  set(CMAKE_ADDR2LINE             ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-addr2line CACHE FILEPATH "")
  set(CMAKE_C_COMPILER_RANLIB     ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-ranlib CACHE FILEPATH "")
  set(CMAKE_CXX_COMPILER_RANLIB   ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-ranlib CACHE FILEPATH "")
  set(CMAKE_LINKER                ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-ld CACHE FILEPATH "")
  set(CMAKE_NM                    ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-nm CACHE FILEPATH "")
  set(CMAKE_OBJDUMP               ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-objdump CACHE FILEPATH "")
  set(CMAKE_RANLIB                ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-ranlib CACHE FILEPATH "")
  set(CMAKE_READELF               ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-readelf CACHE FILEPATH "")
  set(CMAKE_STRIP                 ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-strip CACHE FILEPATH "")
  set(COVERAGE_COMMAND            ${CMAKE_SYSROOT}/bin/${ARCHITECTURE_TRIPLE}-gcov CACHE FILEPATH "")
endif()
