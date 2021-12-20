#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/../lcm_files/java
export CLASSPATH=${DIR}/../lcm_files/java/my_types.jar
pwd
lcm-spy --lcm-url="udpm://239.255.12.21:1221"
