# Manual udev rule steps for Rosmaster board

These are the detailed manual steps that `add_rule.sh` automates. Use them if you prefer not to run the script.

## 1. Identify the device
Plug the board in and list serial devices:
```
ls /dev/ttyUSB*
```
Optionally inspect attributes (adjust ttyUSB0 if different):
```
udebadm info -a -n /dev/ttyUSB0 | less
```

Typical CH340 / QinHeng adapters use vendor:product 1a86:7523.

## 2. Create the udev rule file
```
sudo nano /etc/udev/rules.d/99-rosmaster.rules
```
Paste (modify `myserial` to your desired symlink, keep quotes):
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="myserial"
```
(You can also tighten permissions, e.g. MODE:="0666" or use GROUP="dialout").

## 3. Reload rules
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Or unplug and replug the device.

## 4. Verify
```
ls -l /dev/myserial
```
It should show a symlink pointing to something like `ttyUSB0`.

## 5. Use in Python
```python
from Rosmaster_Lib import Rosmaster
rm = Rosmaster(com="/dev/myserial")
```
Because `Rosmaster` defaults to `/dev/myserial` (if you keep that name) you can also omit the argument.

## 6. Custom vendor/product
If your adapter uses a different VID:PID, adapt the rule:
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="abcd", ATTRS{idProduct}=="1234", MODE:="0666", SYMLINK+="myros"
```
Find the correct IDs via:
```
lsusb
```

## 7. Multiple identical adapters
Match by physical path or serial attribute. After running:
```
udebadm info -a -n /dev/ttyUSB0 | grep -E 'serial|idVendor|idProduct'
```
Add an extra selector, e.g. `ATTRS{serial}=="A50285BI"` to distinguish boards.

## 8. Reverting
Delete the line (or file) and reload rules:
```
sudo rm /etc/udev/rules.d/99-rosmaster.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Script alternative
All of the above condensed:
```
sudo ./add_rule.sh myserial
```
