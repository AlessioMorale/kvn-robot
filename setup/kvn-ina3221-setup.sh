#!/bin/bash
# Bind the INA3221 power monitor on kvn_expio and set its shunt resistors.
# Run as root at boot by kvn-ina3221.service. Safe to run again.
set -euo pipefail

I2C_BUS=1
I2C_ADDR=0x40
SHUNT_MICRO_OHM=20000          # 20 mOhm on all 3 channels

bus_dir=/sys/bus/i2c/devices/i2c-${I2C_BUS}
dev_dir=/sys/bus/i2c/devices/${I2C_BUS}-$(printf '%04x' "${I2C_ADDR}")

wait_for() {  # wait_for <description> <test command...>
  local what=$1; shift
  for _ in $(seq 50); do
    "$@" && return 0
    sleep 0.2
  done
  echo "timeout waiting for ${what}" >&2
  return 1
}

modprobe ina3221
wait_for "${bus_dir}" test -d "${bus_dir}"

# Create the device unless it already exists with the driver bound
if [ ! -e "${dev_dir}/driver" ]; then
  [ -e "${dev_dir}" ] && echo "${I2C_ADDR}" > "${bus_dir}/delete_device"
  echo "ina3221 ${I2C_ADDR}" > "${bus_dir}/new_device"
fi

# hwmonN numbering depends on probe order, so look it up under the device
find_hwmon() { compgen -G "${dev_dir}/hwmon/hwmon*" | head -n1; }
wait_for "hwmon dir under ${dev_dir}" find_hwmon >/dev/null
hwmon=$(find_hwmon)

for ch in 1 2 3; do
  echo "${SHUNT_MICRO_OHM}" > "${hwmon}/shunt${ch}_resistor"
done
echo "INA3221 at ${dev_dir} -> ${hwmon}, shunts ${SHUNT_MICRO_OHM} uOhm"
