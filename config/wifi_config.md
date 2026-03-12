# WiFi Configuration Guide for Raspberry Pi 4B
## ASCEND Drone Project

---

## Option A: Connect RPi to Existing WiFi Network (Recommended)

Your RPi joins the same WiFi network as your laptop.

### Step 1: Configure WiFi via raspi-config
```bash
sudo raspi-config
# Navigate to: System Options → Wireless LAN
# Enter your WiFi SSID and password
```

### Step 2: Or edit wpa_supplicant directly
```bash
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

Add:
```
country=IN
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="YOUR_WIFI_NETWORK_NAME"
    psk="YOUR_WIFI_PASSWORD"
    key_mgmt=WPA-PSK
}
```

### Step 3: Set Static IP (important for reliable SSH)
```bash
sudo nano /etc/dhcpcd.conf
```

Add at the bottom:
```
interface wlan0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1 8.8.8.8
```

### Step 4: Restart networking
```bash
sudo systemctl restart dhcpcd
```

### Step 5: Test from your laptop
```bash
ping 192.168.1.100
ssh pi@192.168.1.100
```

---

## Option B: RPi as WiFi Hotspot (Field Use / No Router)

RPi creates its own WiFi network that your laptop connects to.

### Using NetworkManager (Raspberry Pi OS Bookworm+)
```bash
sudo nmcli dev wifi hotspot ifname wlan0 ssid ASCEND_DRONE password ascend2024
```

### Make hotspot auto-start on boot
```bash
sudo nmcli connection modify Hotspot connection.autoconnect yes
```

### Your laptop connects to:
- **SSID**: ASCEND_DRONE
- **Password**: ascend2024
- **RPi IP**: 10.42.0.1

### SSH into RPi
```bash
ssh pi@10.42.0.1
```

---

## SSH Key Setup (Passwordless Login)

On your laptop, generate an SSH key and copy to RPi:

```bash
ssh-keygen -t ed25519 -C "ascend-laptop"
ssh-copy-id pi@192.168.1.100
```

Now you can SSH without typing the password every time.

---

## Useful SSH Commands for Mission Control

| Command | Description |
|---------|-------------|
| `ssh pi@192.168.1.100` | Connect to RPi |
| `ssh pi@192.168.1.100 'tmux attach -t ascend_mission'` | View live mission output |
| `ssh pi@192.168.1.100 'tail -f ~/ascend_logs/latest_run.log'` | Stream logs |
| `scp pi@192.168.1.100:~/ascend_logs/*.csv .` | Download telemetry data |
| `ssh pi@192.168.1.100 'htop'` | Monitor RPi CPU/memory |

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Can't find RPi on network | Try `ping raspberrypi.local` or scan with `nmap -sn 192.168.1.0/24` |
| SSH connection refused | Ensure SSH is enabled: `sudo systemctl enable ssh` |
| WiFi drops during flight | Use Option B (hotspot) for reliable direct connection |
| High latency | Ensure RPi and laptop are on the same subnet (5GHz preferred) |
