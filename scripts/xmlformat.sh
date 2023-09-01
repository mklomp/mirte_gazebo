#!/bin/bash
find . -wholename "**/*.launch" -type f -exec xmllint --output '{}' --format '{}' \;
find . -wholename "**/*.xacro" -type f -exec xmllint --output '{}' --format '{}' \;
find . -wholename "**/*.world" -type f -exec xmllint --output '{}' --format '{}' \;
find . -wholename "**/*.xml" -type f -exec xmllint --output '{}' --format '{}' \;