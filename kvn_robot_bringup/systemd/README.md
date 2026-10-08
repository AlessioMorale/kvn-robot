# Robot services, CPU isolation and firewall (plan T4.3, T4.4)

| File | Purpose |
|---|---|
| `kvn-ros.env` | shared environment, includes `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (DDS never crosses the VPN) |
| `kvn-control.service` | control stack (`mini_mvp_complete.launch.py`), `Nice=-5`, never depends on ZeroTier |
| `kvn-wifi.service` | `kvn_wifi.launch.py` (bridge + video): `Nice=10`, `CPUQuota=150%`, `After=`/`Wants=zerotier-one.service` |
| `nftables-kvn.conf` | table `inet kvn`: TCP 8765 accepted on `zt*` only, dropped elsewhere; nothing else touched |
| `zerotier-flow-rules.example` | optional ZeroTier Central flow rules |

Adjust `User=`, `KVN_WS` and the CPU quota (150% of 400% on 4 cores) for your robot.

## Install

```bash
sudo install -d /etc/kvn
sudo install -m 0644 kvn-ros.env /etc/kvn/kvn-ros.env
sudo install -m 0644 kvn-control.service kvn-wifi.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now kvn-control.service kvn-wifi.service
```

Other ROS 2 shells on the robot should `set -a; . /etc/kvn/kvn-ros.env; set +a` too.

## Firewall (nftables)

```bash
sudo nft -c -f nftables-kvn.conf                       # syntax check
sudo install -D -m 0644 nftables-kvn.conf /etc/nftables.d/kvn.conf
# make /etc/nftables.conf load it (once):
echo 'include "/etc/nftables.d/*.conf"' | sudo tee -a /etc/nftables.conf
sudo systemctl enable --now nftables.service
sudo nft list table inet kvn
```

The file deletes and recreates only `inet kvn`, so reloads are idempotent. Verify: `nc -zv <lan-ip> 8765` fails, `nc -zv <zerotier-ip> 8765` succeeds, rules survive reboot.
If ZeroTier interfaces are named differently (`ip -br link`; default `ztXXXXXXXX`), edit the `iifname` pattern.

## Measure control-path jitter (T4.3)

With the bridge streaming to a viewer and video at maximum settings:

```bash
ros2 run kvn_robot_bringup joy_cmdvel_jitter.py --duration 60 --budget-ms 5
ros2 node list    # from a laptop on the VPN: must show nothing
```
