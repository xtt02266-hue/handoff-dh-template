"""Verify blank rejection and a synthetic Keil build in a disposable copy.

No real robot parameters are used. Never flash the synthetic firmware.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--keil-root', type=Path, default=Path('F:/Keil5'))
    args = parser.parse_args()
    compiler = args.keil_root/'ARM/ARMCLANG/bin/armclang.exe'
    blank = subprocess.run([str(compiler),'--target=arm-arm-none-eabi','-E','-x','c',
                            str(ROOT/'User/gravity/gravity_config.h')],
                           capture_output=True,text=True)
    assert blank.returncode != 0 and 'Fill robot_parameters.h' in blank.stderr
    print('PASS: unfilled parameters reject firmware compilation',flush=True)

    with tempfile.TemporaryDirectory(prefix='handoff_build_') as directory:
        copy = Path(directory)/'project'
        shutil.copytree(ROOT,copy,ignore=shutil.ignore_patterns('__pycache__','*.log','*.hex','*.axf'))
        path = copy/'User/gravity/robot_parameters.h'
        text = path.read_text(encoding='utf-8').replace('ROBOT_PARAMETERS_READY 0',
                                                     'ROBOT_PARAMETERS_READY 1')
        def fixture(match):
            axis,field=int(match[1]),match[2]
            if field in ['POSITION_SIGN','TORQUE_SIGN']:
                return '1.0f'
            if field == 'LINK':
                return str(axis-1)
            if field == 'MASS':
                return '0.2f'
            if field == 'A':
                return '0.05f'
            if field == 'ALPHA':
                return '0.3f'
            return '0.0f'
        text = re.sub(r'FILL_J([0-5])_([A-Z_]+)',fixture,text)
        path.write_text(text,encoding='utf-8')
        shell = shutil.which('pwsh') or shutil.which('powershell')
        subprocess.run([shell,'-NoProfile','-File',str(copy/'tools/check_environment.ps1'),
                        '-KeilRoot',str(args.keil_root)],cwd=copy,check=True)
        subprocess.run([str(args.keil_root/'UV4/UV4.exe'),'-r',
                        str(copy/'MDK-ARM/CtrBoard.uvprojx'),'-t','CtrBoard','-o',
                        str(copy/'MDK-ARM/build_check.log')],cwd=copy,timeout=60)
        log=(copy/'MDK-ARM/build_check.log').read_text(errors='replace')
        print(log,flush=True)
        assert '0 Error(s), 0 Warning(s)' in log
        print('PASS: synthetic filled template builds with 0 errors/warnings',flush=True)
    print('PASS: temporary parameters and firmware removed; delivery template remains blank')


if __name__ == '__main__':
    main()
