# setup

Boot-time hardware setup for the robot (Raspberry Pi 5 + kvn_expio).

```bash
./install.sh
```

This installs and enables:

| Unit | Script | What it does |
|---|---|---|
| `kvn-ina3221.service` | `/usr/local/sbin/kvn-ina3221-setup.sh` | Binds the INA3221 at I2C1 0x40 (the stock overlays don't support it) and sets the 20 mΩ shunts on all 3 channels. Finds the `hwmonN` dir under `/sys/bus/i2c/devices/1-0040` because `N` changes with probe order. |

Check it with `systemctl status kvn-ina3221` or `journalctl -u kvn-ina3221`. After editing a script, run `./install.sh` again.
