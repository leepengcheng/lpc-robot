# file(GLOB ADSLIB_SRC *.cpp *.h)
aux_source_directory(. ADSLIB_SRC)
add_library(AdsLib STATIC ${ADSLIB_SRC})