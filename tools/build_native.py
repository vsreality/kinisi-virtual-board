"""Build the standalone C++ app and selected C firmware (Python is build-time only)."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from kinisi_virtual_board import Schema
from kinisi_virtual_board.firmware_build import build_firmware, DEFAULT_FIRMWARE


def windows_toolchain(msvc=None, sdk=None):
    if msvc is None:
        msvc=os.environ.get("KINISI_MSVC")
    if not msvc:
        parent=Path("C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC")
        versions=sorted(parent.glob("*"))
        if versions:msvc=versions[-1]
    if not msvc:raise RuntimeError("Set KINISI_MSVC or --msvc to the MSVC version directory containing bin/Hostx64/x64/cl.exe")
    msvc=Path(msvc).resolve()
    sdk=Path(sdk or os.environ.get("KINISI_WINDOWS_SDK","C:/Program Files (x86)/Windows Kits/10"))
    versions=sorted((sdk/"Include").glob("10.*"))
    if not versions:raise RuntimeError("Windows SDK headers not found")
    version=versions[-1].name
    includes=[msvc/"include",*(sdk/"Include"/version/d for d in ("ucrt","shared","um","winrt"))]
    libraries=[msvc/"lib/x64",sdk/"Lib"/version/"ucrt/x64",sdk/"Lib"/version/"um/x64"]
    binary=msvc/"bin/Hostx64/x64"
    if not (binary/"cl.exe").is_file():raise RuntimeError(f"MSVC compiler missing in {binary}")
    environment=dict(os.environ,PATH=str(binary)+os.pathsep+os.environ.get("PATH",""),
                     INCLUDE=os.pathsep.join(map(str,includes)),LIB=os.pathsep.join(map(str,libraries)))
    return binary,environment,sdk,version


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware",type=Path,default=DEFAULT_FIRMWARE)
    parser.add_argument("--commands",type=Path)
    parser.add_argument("--revision",default="0.4.0",choices=["0.3.0","0.4.0"])
    parser.add_argument("--out",type=Path,default=ROOT/"build/native")
    parser.add_argument("--compiler",help="Host C compiler for production firmware")
    parser.add_argument("--cxx",help="Linux C++ compiler")
    parser.add_argument("--msvc",type=Path)
    args=parser.parse_args()
    args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=True)
    schema=Schema(args.commands or args.firmware/"commands.json")
    library,manifest=build_firmware(schema,args.firmware,args.revision,args.compiler)
    shutil.copyfile(library,args.out/library.name)
    source=ROOT/"native"
    if os.name=="nt":
        binary,environment,_,_=windows_toolchain(args.msvc)
        command=[str(binary/"cl.exe"),"/nologo","/std:c++17","/EHsc","/MT","/O2","/W4","/WX",
                 "/D_WIN32_WINNT=0x0A00","/DNTDDI_VERSION=0x0A000000",str(source/"main.cpp"),
                 str(source/"windows/port_windows.cpp"),"/Fe:kinisi-sim.exe","/link","swdevice.lib","cfgmgr32.lib","advapi32.lib"]
    else:
        compiler=args.cxx or os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:raise RuntimeError("Install a C++17 compiler or set CXX")
        environment=os.environ
        command=[compiler,"-std=c++17","-O2","-Wall","-Wextra","-Werror",str(source/"main.cpp"),
                 str(source/"port_posix.cpp"),"-ldl","-pthread","-o","kinisi-sim"]
    subprocess.run(command,cwd=args.out,env=environment,check=True)
    if os.name=="nt":
        checks=[str(binary/"cl.exe"),"/nologo","/std:c++17","/EHsc","/MT","/O2","/W4","/WX",
                str(source/"tests.cpp"),"/Fe:native-tests.exe"]
        test_binary=args.out/"native-tests.exe"
    else:
        checks=[compiler,"-std=c++17","-O2","-Wall","-Wextra","-Werror",str(source/"tests.cpp"),"-o","native-tests"]
        test_binary=args.out/"native-tests"
    subprocess.run(checks,cwd=args.out,env=environment,check=True)
    subprocess.run([str(test_binary)],check=True)
    manifest["runtime"]="native-cpp"
    (args.out/"manifest.json").write_text(json.dumps(manifest,indent=2))
    shutil.copyfile(schema.path,args.out/"commands.json")
    print(args.out/("kinisi-sim.exe" if os.name=="nt" else "kinisi-sim"))


if __name__=="__main__":main()
