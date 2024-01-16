#!/usr/bin/env bash

if [ $# -gt 0 ]; then
  CONF_NAME=$1

  echo "Using config: conf/frpc_${CONF_NAME}.ini"
  sudo systemctl stop frpc >> /dev/null 2>&1
  sudo mkdir -p /opt/frp
  sudo cp -r . /opt/frp
  sudo cp service/frpc.service /etc/systemd/system/
  sudo sed -ir "s/server/${CONF_NAME}/" /etc/systemd/system/frpc.service
  sudo systemctl daemon-reload
  sudo systemctl enable frpc
  sudo systemctl restart frpc
else
  echo "USAGE: $0 [conf_name]"
fi

