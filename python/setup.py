import sys
from cx_Freeze import setup, Executable

# Dependencies are automatically detected, but it might need fine tuning.
build_exe_options = {
    "excludes": ["unittest", "lib2to3", "email", "pydoc_data"],
    "zip_include_packages": [],
    "include_files": ['images'],
    "bin_excludes" : ['libcrypto-1_1.dll']
}

# base="Win32GUI" should be used only for Windows GUI app
base = "Win32GUI" if sys.platform == "win32" else None

setup(
    name="EEZYbotARM control",
    version="0.1",
    description="Contrôle d'un EEZYbotARM",
    options={"build_exe": build_exe_options},
    executables=[Executable("EEZYbotARM.py", icon="eezybotarm.ico", base=base)],
)