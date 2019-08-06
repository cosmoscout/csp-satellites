# ------------------------------------------------------------------------------------------------ #
#                                This file is part of CosmoScout VR                                #
#       and may be used under the terms of the MIT license. See the LICENSE file for details.      #
#                         Copyright: (c) 2019 German Aerospace Center (DLR)                        #
# ------------------------------------------------------------------------------------------------ #

# build plugin -------------------------------------------------------------------------------------

file(GLOB SOURCE_FILES src/*.cpp)

add_library(csp-satellites SHARED
  ${SOURCE_FILES}
)

target_link_libraries(csp-satellites
  PUBLIC
    cs-core
    cs-graphics
    cs-scene
)

# install plugin -----------------------------------------------------------------------------------

install(TARGETS csp-satellites DESTINATION "share/plugins")
install(DIRECTORY "textures" DESTINATION "share/resources")