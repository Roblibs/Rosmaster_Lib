## Rosmaster_Lib

Python library for Yahboom Rosmaster based robot driver board https://www.yahboom.net/study/ROS-Driver-Board

# Getting started

```
pip install git+https://github.com/RobLibs/Rosmaster_Lib@V3.3.9
```

Then in code:

```python
from Rosmaster_Lib import Rosmaster
rm = Rosmaster(com="COM5")  # or Linux device path
```

# Install Rule (recommended)

Linux: USB‑serial names (`/dev/ttyUSB0`, `/dev/ttyUSB1`, …) can shift. Use the helper script to create a stable symlink so your code can simply call `Rosmaster()`.

Quick setup (creates `/dev/myserial`):
```
sudo ./add_rule.sh myserial
```

Custom name & explicit VID:PID (hex):
```
sudo ./add_rule.sh my_ros_board 1a86:7523
```

After running, replug the board (or `sudo udevadm trigger`) and verify:
```
ls -l /dev/myserial
```

Then in Python you may omit the parameter (default assumed `/dev/myserial`):
```python
from Rosmaster_Lib import Rosmaster
rm = Rosmaster()  # uses /dev/myserial
```

Manual step‑by‑step instructions (and advanced matching) are in `rule_steps.md`.

# For developpers
## Minimal Example

```python
from Rosmaster_Lib import Rosmaster
rm = Rosmaster(com="/dev/ttyUSB0", debug=True)
rm.create_receive_threading()
rm.set_beep(100)  # 100 ms beep
```

## Dependencies
* pyserial

# For maintainers

### Versioning

Current version: `3.3.9` (git tag: `V3.3.9`).

To release a new version (example 3.4.0):
1. Update version string in `setup.py` and `Rosmaster_Lib/__init__.py`.
2. Commit the changes.
3. Create an annotated tag: `git tag -a V3.4.0 -m "Release 3.4.0"`
4. Push: `git push origin main --tags`
5. Build & upload to PyPI:
	```
	pip install --upgrade build twine
	python -m build
	twine upload dist/*
	```

# License
Proprietary – usage restricted to authorized Yahboom / Rosmaster hardware contexts (see LICENSE).

This organization also reserves the right to Yahboom to change the license how they see fit.
