# Copyright 1996-2018 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

#INCLUDE += -I "../src"
C_SOURCES += heuristics_ga.c autonomous_vehicle_ga.c ../src/robc_lane_detection.c ../src/robc_control.c ../src/robc_heuristics.c ../src/robc_file.c

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

LIBRARIES = -ldriver -lcar -lgsl -lgslcblas
CFLAGS = -Wall

# Do not modify the following: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
