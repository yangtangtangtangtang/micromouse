from pathlib import Path
import ctypes
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'STM32工程'
REPORTS = ROOT.parent / '交付说明与验证'


def build():
    keil = Path(os.environ.get('KEIL_UV4', 'C:/Keil_v5/UV4/UV4.exe'))
    if not keil.is_file():
        raise RuntimeError('未找到Keil。请安装Keil MDK、ARM Compiler 5和STM32F1xx_DFP 2.2.0，或设置KEIL_UV4。')
    work = Path(tempfile.mkdtemp(prefix='mazemouse_build_'))
    for source in SOURCE.rglob('*'):
        if source.is_file() and source.suffix.lower() in ('.c', '.h', '.s', '.txt', '.ioc', '.uvprojx'):
            target = work / source.relative_to(SOURCE)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    log = work / 'build.log'
    process = subprocess.run(
        [str(keil), '-r', str(work / 'MDK-ARM/DZXTSJ.uvprojx'), '-o', str(log)],
        timeout=240, creationflags=subprocess.CREATE_NO_WINDOW)
    text = log.read_bytes().decode('gb18030', errors='replace') if log.exists() else ''
    binary = work / 'MDK-ARM/Build/DZXTSJ.hex'
    passed = process.returncode == 0 and '0 Error(s), 0 Warning(s)' in text and binary.is_file()
    REPORTS.mkdir(exist_ok=True)
    (REPORTS / '下位机编译记录.txt').write_text(text, encoding='utf-8')
    if not passed:
        raise RuntimeError('编译未通过，请查看交付说明与验证中的下位机编译记录.txt。')
    firmware = ROOT / '固件'
    firmware.mkdir(exist_ok=True)
    shutil.copy2(binary, firmware / 'DZXTSJ.hex')
    manifest = {str(p.relative_to(SOURCE)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in SOURCE.rglob('*') if p.is_file() and 'Build' not in p.parts}
    proof = {'status': 'PASS', 'scope': 'compile_and_link_only_no_physical_device',
             'compiler': 'ARM Compiler 5.06 update 6 build 750',
             'hex_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
             'source_sha256': manifest}
    (REPORTS / '固件与源码对应.json').write_text(json.dumps(proof, ensure_ascii=False, indent=2), encoding='utf-8')
    return '编译通过：0错误、0警告。已生成固件/DZXTSJ.hex。未进行实物烧录或运动验证。'


if __name__ == '__main__':
    try:
        message = build()
        print(message)
        if '--quiet' not in sys.argv:
            ctypes.windll.user32.MessageBoxW(0, message, '电脑鼠编译验证', 64)
    except Exception as error:
        if '--quiet' not in sys.argv:
            ctypes.windll.user32.MessageBoxW(0, str(error), '编译验证失败', 16)
        raise
