"""Build and validate the UMDF 2 driver package. Does not sign, trust or install it."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
from build_native import ROOT, windows_toolchain


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--msvc",type=Path)
    parser.add_argument("--wdk",type=Path,default=os.environ.get("KINISI_WDK"),
                        help="WDK root containing Include/wdf and Lib/wdf (NuGet: package/c)")
    parser.add_argument("--out",type=Path,default=ROOT/"build/driver")
    args=parser.parse_args()
    if os.name!="nt":parser.error("Driver build requires Windows")
    if not args.wdk:parser.error("Set --wdk or KINISI_WDK")
    args.wdk=args.wdk.resolve();args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=True)
    binary,environment,sdk,version=windows_toolchain(args.msvc)
    wdf=args.wdk/"Include/wdf/umdf/2.33"
    driver_include=args.wdk/"Include"/version
    environment["INCLUDE"]=os.pathsep.join(map(str,[wdf,driver_include/"um",driver_include/"shared",driver_include/"km"]))+os.pathsep+environment["INCLUDE"]
    environment["LIB"]=str(args.wdk/"Lib/wdf/umdf/x64/2.33")+os.pathsep+environment["LIB"]
    subprocess.run([str(binary/"cl.exe"),"/nologo","/std:c++17","/W4","/WX","/MT","/O2","/LD",
                    "/DUMDF_VERSION_MAJOR=2","/DUMDF_VERSION_MINOR=33","/D_WIN32_WINNT=0x0A00",
                    "/DWINVER=0x0A00","/DUNICODE","/D_UNICODE",
                    str(ROOT/"native/windows/driver/driver.cpp"),"/Fe:kinisiport.dll",
                    "/link","/DEF:"+str(ROOT/"native/windows/driver/driver.def"),
                    "WdfDriverStubUm.lib","ntdll.lib","mincore.lib"],cwd=args.out,env=environment,check=True)
    shutil.copyfile(ROOT/"native/windows/driver/kinisiport.inf",args.out/"kinisiport.inf")
    # Prefer WDK's actual validator; never substitute a text-only INF check.
    validators=list(args.wdk.rglob("infverif.exe"))
    if not validators:raise RuntimeError("WDK InfVerif not found: package is not validated")
    validator=next((p for p in validators if "/x64/" in p.as_posix().lower()),validators[0])
    subprocess.run([str(validator),"/w",str(args.out/"kinisiport.inf")],check=True)
    catalog_tools=list(args.wdk.rglob("Inf2Cat.exe"))
    if not catalog_tools:raise RuntimeError("WDK Inf2Cat not found: package has no catalog")
    subprocess.run([str(catalog_tools[0]),"/driver:"+str(args.out),"/os:10_X64","/uselocaltime"],check=True)
    print("Built unsigned driver package:",args.out)


if __name__=="__main__":main()
