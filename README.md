## Rosmaster_Lib

Python library for Yahboom Rosmaster based robot driver board.

### Install from PyPI (preferred once published)

```
pip install Rosmaster_Lib
```

Then in code:

```python
from Rosmaster_Lib import Rosmaster
rm = Rosmaster(com="COM5")  # or Linux device path
```

### Install directly from GitHub (before PyPI release)

```
pip install git+https://github.com/RobLibs/Rosmaster_Lib@V3.3.9
```

### Local development install

```
git clone https://github.com/RobLibs/Rosmaster_Lib.git
cd Rosmaster_Lib
pip install -e .
```

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

### Minimal Example

```python
from Rosmaster_Lib import Rosmaster
rm = Rosmaster(com="/dev/ttyUSB0", debug=True)
rm.create_receive_threading()
rm.set_beep(100)  # 100 ms beep
```

### Dependencies
* pyserial

### License
Proprietary – usage restricted to authorized Yahboom / Rosmaster hardware contexts (see LICENSE).

### Notes
Windows users: replace the serial `com` argument with the actual COM port (e.g. `COM7`).


