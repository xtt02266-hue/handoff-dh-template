"""Run the production protection/home C functions with a fake clock and motors.

Requires an MSVC installation and Windows SDK. No hardware is accessed.
"""
from pathlib import Path
import argparse
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PREAMBLE = r'''
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
enum { motor_num = 6 };
typedef struct { float pos, vel; } feedback_t;
typedef struct { float pos_set, vel_set, tor_set, kp_set, kd_set; } command_t;
typedef struct { feedback_t para; command_t cmd; int id; } motor_t;
typedef struct { float max_tor, max_vel, max_pos, min_pos; } motor_limit_t;
motor_t motor[motor_num];
uint32_t now_ms;
uint8_t ready = 1;
int hcan1, hcan2;
#define MOTOR_LOCK_KP_MIN 0.5f
#define MOTOR_LOCK_KP_PER_TORQUE 0.35f
#define MOTOR_LOCK_KP_MAX 12.0f
#define MOTOR_LOCK_KP_SMOOTHING 0.30f
#define MOTOR_LOCK_KD 0.02f
uint32_t HAL_GetTick(void) { return now_ms; }
uint8_t motor_feedback_ready(void) { return ready; }
void ctrl_set(void) {}
void ctrl_send(void) {}
void save_pos_zero(int *bus, int id, int mode) {}
#define ROBOT_DOF 6
float model_torque[6];
void GravityComp_Calculate(const float *q, float *t) {
    for(int i=0;i<6;i++)t[i]=model_torque[i];
}
void Set_MIT_PVT(motor_t *m,float p,float v,float t) {
    m->cmd.pos_set=p; m->cmd.vel_set=v; m->cmd.tor_set=t;
}
void Set_MIT_PD(motor_t *m,float p,float d) { m->cmd.kp_set=p; m->cmd.kd_set=d; }
'''
TESTS = r'''
static void reset_test(void) {
    memset(motor,0,sizeof(motor));
    memset(model_torque,0,sizeof(model_torque));
    memset(in_vel_limit,0,sizeof(in_vel_limit));
    global_system_halt_flag=0;
    now_ms=1000; ready=1; lock_flag=0;
    motor_pos_init_reset(); motor_pos_run_reset();
}
int main(void) {
    reset_test();
    motor[0].para.vel=11;
    motor[0].cmd.pos_set=1;
    motor[0].cmd.kp_set=5;
    motor[0].cmd.kd_set=2.5f;
    motor[0].cmd.tor_set=0.5f;
    motor_protect_run();
    assert(!global_system_halt_flag);
    assert(motor[0].cmd.vel_set==0 && motor[0].cmd.kd_set==2.5f);
    assert(motor[0].cmd.kp_set==0 && motor[0].cmd.tor_set==0);
    assert(motor_speed_limited());
    motor[0].para.vel=9; motor_protect_run();
    assert(motor_speed_limited());
    motor[0].para.vel=7; motor[0].cmd.kp_set=0; motor[0].cmd.kd_set=0;
    motor_protect_run();
    assert(!motor_speed_limited() && motor[0].cmd.kd_set==0);

    reset_test();
    motor[0].para.vel=-11; motor[0].cmd.pos_set=-1;
    motor[0].cmd.kp_set=5; motor[0].cmd.tor_set=-0.5f;
    motor_protect_run();
    assert(motor[0].cmd.vel_set==0 && motor[0].cmd.kd_set==2);
    assert(motor[0].cmd.kp_set==0 && motor[0].cmd.tor_set==0);

    /* Clamp feedforward before deciding whether the P + FF sum brakes. */
    reset_test(); motor[0].para.vel=11;
    motor[0].cmd.kp_set=5; motor[0].cmd.pos_set=1; motor[0].cmd.tor_set=-6;
    motor_protect_run();
    assert(motor[0].cmd.kp_set==0 && motor[0].cmd.tor_set==0);

    reset_test();
    motor[0].para.pos=2.1f; motor[0].para.vel=11;
    motor_protect_run();
    assert(motor[0].cmd.pos_set==2 && motor[0].cmd.kp_set>=10);
    motor[0].para.pos=1.9f; motor[0].para.vel=0;
    motor[0].cmd.kp_set=0; motor[0].cmd.kd_set=0;
    motor_protect_run();
    assert(motor[0].cmd.kp_set==0 && motor[0].cmd.kd_set==0);

    reset_test(); motor[0].para.vel=16; motor_protect_run();
    assert(global_system_halt_flag && motor[0].cmd.kp_set==0 && motor[0].cmd.kd_set==2);
    reset_test(); model_torque[3]=INFINITY; Gravity_Arm_Run(); motor_protect_run();
    assert(global_system_halt_flag && motor[0].cmd.tor_set==0);
    reset_test(); model_torque[3]=NAN; Gravity_Arm_Run(); motor_protect_run();
    assert(global_system_halt_flag && motor[3].cmd.tor_set==0);
    reset_test(); ready=0; motor_protect_run();
    assert(global_system_halt_flag);
    reset_test(); motor[0].cmd.tor_set=5; motor_protect_run();
    assert(fabsf(motor[0].cmd.tor_set-0.6f)<0.0001f);

    reset_test(); motor[3].para.pos=0.7f;
    assert(!motor_pos_init());
    assert(motor[3].cmd.pos_set==0.7f && motor[3].cmd.kp_set==0);
    now_ms+=10; assert(!motor_pos_init());
    assert(motor[3].cmd.pos_set<0.7f && motor[3].cmd.pos_set>0.699f);
    for(int k=0;k<500;k++) {
        now_ms+=10;
        for(int i=0;i<6;i++) motor[i].para.pos=motor[i].cmd.pos_set;
        if(motor_pos_init()) break;
    }
    assert(!global_system_halt_flag && motor_pos_init());

    reset_test(); motor[0].para.pos=1; assert(!motor_pos_init());
    for(int k=0;k<400 && !global_system_halt_flag;k++) { now_ms+=10; assert(!motor_pos_init()); }
    assert(global_system_halt_flag);
    reset_test(); assert(!motor_pos_init());
    motor[0].para.vel=1; now_ms+=10;
    assert(!motor_pos_init());
    reset_test(); ready=0; assert(!motor_pos_init() && global_system_halt_flag);
    reset_test(); motor[0].para.pos=0.4f; motor_lock();
    assert(motor[0].cmd.pos_set==0.4f);
    lock_flag=0; motor[0].para.pos=0.8f; motor_lock();
    assert(motor[0].cmd.pos_set==0.8f);
    puts("PASS: speed/braking/hysteresis, mode gains, limits, faults, home and lock");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--msvc', default=r'F:\vs2026\VC\Tools\MSVC\14.50.35717')
    parser.add_argument('--sdk', default=r'C:\Program Files (x86)\Windows Kits\10')
    args = parser.parse_args()
    msvc, sdk = Path(args.msvc), Path(args.sdk)
    version = sorted((sdk / 'Include').iterdir())[-1].name
    env = os.environ.copy()
    env['PATH'] = str(msvc / 'bin/Hostx64/x64') + os.pathsep + env['PATH']
    env['INCLUDE'] = ';'.join(str(p) for p in [msvc / 'include'] +
                             [sdk / 'Include' / version / n for n in ['ucrt', 'shared', 'um']])
    env['LIB'] = ';'.join(str(p) for p in [msvc / 'lib/x64'] +
                         [sdk / 'Lib' / version / n / 'x64' for n in ['ucrt', 'um']])
    sources = []
    for name in ['User/2-MWL(Middleware Layer)/src/motor_protect.c',
                 'User/gravity/gravity.c',
                 'User/2-MWL(Middleware Layer)/src/motor_function.c']:
        content = (ROOT / name).read_text(encoding='utf-8')
        sources.append(re.sub(r'^#include[^\n]*', '', content, flags=re.M))
    with tempfile.TemporaryDirectory(prefix='arm_control_test_') as directory:
        folder = Path(directory)
        (folder / 'test.c').write_text(PREAMBLE + '\n'.join(sources) + TESTS, encoding='utf-8')
        subprocess.run([str(msvc / 'bin/Hostx64/x64/cl.exe'), '/nologo', '/utf-8', '/std:c11', '/Od', 'test.c', '/Fe:test.exe'],
                       cwd=folder, env=env, check=True)
        subprocess.run([str(folder / 'test.exe')], cwd=folder, check=True)


if __name__ == '__main__':
    main()
