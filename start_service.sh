#!/bin/bash
# 嵌入式机器人启动脚本
PATH=/usr/local/bin:$PATH
LOG_FILE=/var/log/robot_run.log

# 提升实时优先级
sudo chrt -f -p 99 robot_app

# 循环守护，异常自动重启
while true
do
    echo "======== 机器人程序启动 $(date) =======" >> $LOG_FILE
    robot_app >> $LOG_FILE 2>&1
    sleep 2
done