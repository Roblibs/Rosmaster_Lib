from setuptools import find_packages, setup
from pathlib import Path

ROOT = Path(__file__).parent
README = (ROOT / "README.md").read_text(encoding="utf-8") if (ROOT / "README.md").exists() else "Rosmaster driver library"

setup(
    name='Rosmaster_Lib',                     # Import: from Rosmaster_Lib import Rosmaster
    version='3.3.9',                          # Keep in sync with git tag V3.3.9
    author='Yahboom Team',
    author_email='support@example.com',       # TODO: replace with real contact or remove
    description='Python library for controlling Yahboom Rosmaster based robot driver board.',
    long_description=README,
    long_description_content_type='text/markdown',
    url='https://github.com/RobLibs/Rosmaster_Lib',
    project_urls={
        'Source': 'https://github.com/RobLibs/Rosmaster_Lib',
        'Issues': 'https://github.com/RobLibs/Rosmaster_Lib/issues',
    },
    license='Proprietary',
    packages=find_packages(include=["Rosmaster_Lib", "Rosmaster_Lib.*"]),
    python_requires='>=3.7',
    install_requires=[
        'pyserial>=3.0',
    ],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
    'License :: Other/Proprietary License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3 :: Only',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Topic :: Software Development :: Libraries',
        'Topic :: Scientific/Engineering :: Interface Engine/Protocol Translator',
    ],
    keywords='rosmaster robotics yahboom serial driver',
)

# Local (legacy) install fallback examples:
#   python -m pip install .
# Build & publish (modern):
#   python -m pip install --upgrade build twine
#   python -m build
#   twine upload dist/*
