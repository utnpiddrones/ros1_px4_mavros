#!/bin/bash

DIR="$(rospack find my_mavros)/recordings/default"
DB0="${DIR}/uav0.db"
DB1="${DIR}/uav1.db"
DB2="${DIR}/uav2.db"
DB3="${DIR}/uav3.db"

OUTPUT="${DIR}/merged.db"

ARG="${DB0};${DB1};${DB2};${DB3}"
 
rtabmap-reprocess $ARG $OUTPUT