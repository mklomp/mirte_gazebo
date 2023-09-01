#!/bin/bash

$(dirname $0)/buildMap.py "$1" "$2"
$(which xacro) "$2" -o "$3"