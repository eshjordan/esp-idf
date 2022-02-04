#!/bin/bash
# This shell snippet configure esp-idf tools
#
# Use this script like this:
#
# . init.sh
#
if [ -z ${IDF_PATH} ]; then
	local actual_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
	echo Actual Dir: $actual_dir
	. /Users/danielburnier/Documents/EPFL-Mobots/Projects/esp/Run\ ESP32\ tools.sh
	. /Users/danielburnier/Documents/EPFL-Mobots/Projects/esp/esp-idf/add_path.sh
	cd $actual_dir
else
	echo Init already done !
fi