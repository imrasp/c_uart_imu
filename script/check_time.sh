#!/bin/bash

uptime
df -h | grep mmcblk0p2
echo "Free memory: $(free -h | sed -n 2p | tr -s ' ' | cut -d' ' -f 4)"
echo
echo "IMU timestamp $(python -c "from datetime import datetime; dt = datetime.fromtimestamp($(tail -n1 ../record_data/imu0.csv | cut -d ',' -f1) // 1000000000); s = dt.strftime('%Y-%m-%d %H:%M:%S'); print(s);")"
echo "Current time: $(date)"
echo
ps -ef | grep c_uart[n]
echo "This process has been running for $(echo $(ps -o etimes= -p $(pgrep c_uart))/3600 | bc -l;) hours"
