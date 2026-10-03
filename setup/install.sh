#!/bin/bash
# Install and start the kvn boot-time hardware setup services.
set -euo pipefail
cd "$(dirname "$0")"

sudo install -m 0755 kvn-ina3221-setup.sh /usr/local/sbin/kvn-ina3221-setup.sh
sudo install -m 0644 kvn-ina3221.service /etc/systemd/system/kvn-ina3221.service
sudo systemctl daemon-reload
sudo systemctl enable --now kvn-ina3221.service
systemctl --no-pager status kvn-ina3221.service
