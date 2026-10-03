"""Test production DH/gravity C with synthetic models, never with arm answers."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TESTS = r'''
static void setup(void) {
    memset(GRAVITY_DH,0,sizeof(GRAVITY_DH));
    memset(GRAVITY_MOTOR_MASS,0,sizeof(GRAVITY_MOTOR_MASS));
    for(int i=0;i<6;i++) {
        GRAVITY_JOINT_MAP[i]=(JointMap){0,1,1};
        GRAVITY_MOTOR_MASS[i].link_index=-1;
    }
    /* Artificial horizontal axis: world Z rotated onto -Y. */
    GRAVITY_DH[0].alpha=1.5707963267948966f;
}
static float energy(const float q[6]) {
    Mat4 frames[6]; GravityComp_ForwardKinematics(q,frames);
    float u=0;
    for(int i=0;i<6;i++) {
        int link=GRAVITY_MOTOR_MASS[i].link_index;
        if(link<0)continue;
        Vec3 p=GravityMath_TransformPoint(frames[link],GRAVITY_MOTOR_MASS[i].com_local);
        u+=GRAVITY_MOTOR_MASS[i].mass*GRAVITY_ACCEL*p.z;
    }
    return u;
}
int main(void) {
    float q[6]={0},t[6],enc[6]={0};
    setup();
    GRAVITY_MOTOR_MASS[0]=(MotorMassParam){2,0,{0.4f,0,0}};
    assert(GravityComp_ModelTorque(q,t));
    assert(fabsf(t[0]-2*9.8f*0.4f)<0.0001f);
    for(int i=1;i<6;i++)assert(fabsf(t[i])<0.0001f);
    q[0]=1.5707963267948966f;
    assert(GravityComp_ModelTorque(q,t)); assert(fabsf(t[0])<0.0001f);
    q[0]=0;
    GRAVITY_MOTOR_MASS[0].link_index=-1;
    assert(GravityComp_ModelTorque(q,t)); assert(t[0]==0);
    GRAVITY_MOTOR_MASS[0]=(MotorMassParam){2,0,{0,0,0.4f}};
    assert(GravityComp_ModelTorque(q,t)); assert(fabsf(t[0])<0.0001f);
    GRAVITY_MOTOR_MASS[0].mass=0;
    assert(GravityComp_ModelTorque(q,t)); assert(t[0]==0);
    GRAVITY_MOTOR_MASS[0]=(MotorMassParam){1,1,{0.3f,0,0}};
    GRAVITY_MOTOR_MASS[1]=(MotorMassParam){2,1,{0.3f,0,0}};
    assert(GravityComp_ModelTorque(q,t));
    assert(fabsf(t[0]-3*9.8f*0.3f)<0.0001f);
    assert(fabsf(t[1]-3*9.8f*0.3f)<0.0001f);
    for(int i=2;i<6;i++)assert(t[i]==0);
    GRAVITY_MOTOR_MASS[0].link_index=6; assert(!GravityComp_ModelTorque(q,t));
    GRAVITY_MOTOR_MASS[0].link_index=-2; assert(!GravityComp_ModelTorque(q,t));
    setup(); GRAVITY_JOINT_MAP[0]=(JointMap){0.2f,-1,-1};
    enc[0]=0.7f; GravityComp_EncoderToModelQ(enc,q);
    assert(fabsf(q[0]+0.5f)<0.00001f);
    enc[0]=0.2f; GravityComp_EncoderToModelQ(enc,q);
    assert(fabsf(q[0])<0.00001f);
    GRAVITY_JOINT_MAP[0].position_sign=0; assert(!GravityComp_ModelValid());
    setup();
    for(int i=0;i<6;i++) {
        GRAVITY_DH[i]=(DHParam){0.08f*(i+1),0.3f*(i-2),0.04f,0.1f};
        GRAVITY_MOTOR_MASS[i]=(MotorMassParam){0.1f*(i+1),i,{0.03f,0.02f,0.01f}};
    }
    /* Independent potential-energy derivative checks sign and accumulation. */
    for(int pose=0;pose<30;pose++) {
        for(int i=0;i<6;i++)q[i]=sinf((float)(pose*6+i))*1.1f;
        assert(GravityComp_ModelTorque(q,t));
        for(int i=0;i<6;i++) {
            float saved=q[i];
            q[i]=saved+0.001f;float up=energy(q);
            q[i]=saved-0.001f;float down=energy(q);
            q[i]=saved;
            assert(fabsf(t[i]-(up-down)/0.002f)<0.008f);
        }
    }
    GRAVITY_JOINT_MAP[0].torque_sign=-1;
    GravityComp_EncoderToModelQ(enc,q);assert(GravityComp_ModelTorque(q,t));
    float sent[6];GravityComp_Calculate(enc,sent);
    assert(fabsf(sent[0]+t[0])<0.0001f);
    puts("PASS: analytical torque, ownership, base/zero/axis mass, mapping, energy derivative");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--msvc', default=r'F:\vs2026\VC\Tools\MSVC\14.50.35717')
    parser.add_argument('--sdk', default=r'C:\Program Files (x86)\Windows Kits\10')
    parser.add_argument('--reference-root', type=Path,
                        help='Optional private old model for migration equivalence; never copied into template')
    args = parser.parse_args()
    msvc, sdk = Path(args.msvc), Path(args.sdk)
    version = sorted((sdk / 'Include').iterdir())[-1].name
    env = os.environ.copy()
    env['PATH'] = str(msvc / 'bin/Hostx64/x64') + os.pathsep + env['PATH']
    env['INCLUDE'] = ';'.join(str(p) for p in [msvc / 'include'] +
        [sdk / 'Include' / version / n for n in ['ucrt','shared','um']])
    env['LIB'] = ';'.join(str(p) for p in [msvc / 'lib/x64'] +
        [sdk / 'Lib' / version / n / 'x64' for n in ['ucrt','um']])
    header = (ROOT/'User/gravity/gravity_config.h').read_text(encoding='utf-8')
    types = header.split('#include "robot_parameters.h"')[0]
    prototypes = header[header.index('Mat4 GravityMath_Identity'):]
    preamble = '#include <assert.h>\n#include <math.h>\n#include <string.h>\n#include <stdio.h>\n'
    preamble += types + '''
#define GRAVITY_COMP_GAIN 1.0f
DHParam GRAVITY_DH[ROBOT_DOF];
JointMap GRAVITY_JOINT_MAP[ROBOT_DOF];
MotorMassParam GRAVITY_MOTOR_MASS[MOTOR_MASS_COUNT];
'''+prototypes
    source = preamble
    for name in ['gravity_math.c','gravity_comp.c']:
        content = (ROOT/'User/gravity'/name).read_text(encoding='utf-8')
        source += re.sub(r'^#include[^\n]*','',content,flags=re.M)
    with tempfile.TemporaryDirectory(prefix='gravity_test_') as directory:
        folder = Path(directory)
        tests = TESTS
        extra = []
        if args.reference_root:
            old = args.reference_root/'User/gravity'
            reference = (old/'gravity_config.h').read_text(encoding='utf-8')
            reference += re.sub(r'^#include[^\n]*','',
                (old/'gravity_comp.c').read_text(encoding='utf-8'),flags=re.M)
            reference = reference.replace('GravityComp_Calculate','Reference_Calculate')
            reference = reference.replace('GravityComp_EncoderToModelQ','Reference_EncoderToModelQ')
            reference += r'''
float Reference_Load(float *dh, float *map, float *mass) {
    for(int i=0;i<6;i++) {
        dh[4*i]=GRAVITY_DH[i].a; dh[4*i+1]=GRAVITY_DH[i].alpha;
        dh[4*i+2]=GRAVITY_DH[i].d;dh[4*i+3]=GRAVITY_DH[i].theta_offset;
        map[4*i]=GRAVITY_JOINT_MAP[i].encoder_zero;
        map[4*i+1]=GRAVITY_JOINT_MAP[i].position_sign;
        map[4*i+2]=GRAVITY_JOINT_MAP[i].model_offset;
        map[4*i+3]=GRAVITY_JOINT_MAP[i].torque_sign;
        mass[4*i]=GRAVITY_LINK[i].mass;
        mass[4*i+1]=GRAVITY_LINK[i].com_local.x;
        mass[4*i+2]=GRAVITY_LINK[i].com_local.y;
        mass[4*i+3]=GRAVITY_LINK[i].com_local.z;
    }
    return GRAVITY_COMP_GAIN;
}
'''
            (folder/'reference.c').write_text(reference,encoding='utf-8')
            extra = ['reference.c']
            source += 'float Reference_Load(float*,float*,float*);\nvoid Reference_Calculate(const float*,float*);\n'
            comparison = r'''
    float dh[24],map[24],mass[24],before[6],after[6];
    float reference_gain=Reference_Load(dh,map,mass);
    for(int i=0;i<6;i++) {
        GRAVITY_DH[i]=(DHParam){dh[4*i],dh[4*i+1],dh[4*i+2],dh[4*i+3]};
        GRAVITY_DH[i].theta_offset+=map[4*i+2];
        GRAVITY_JOINT_MAP[i]=(JointMap){map[4*i],map[4*i+1],map[4*i+3]};
        GRAVITY_MOTOR_MASS[i]=(MotorMassParam){mass[4*i],i,{mass[4*i+1],mass[4*i+2],mass[4*i+3]}};
    }
    for(int pose=0;pose<100;pose++) {
        for(int i=0;i<6;i++)enc[i]=sinf((float)(pose*6+i))*1.1f;
        Reference_Calculate(enc,before);GravityComp_Calculate(enc,after);
        for(int i=0;i<6;i++)assert(fabsf(before[i]-after[i]*reference_gain)<0.00002f);
    }
    puts("PASS: 100-pose old-model migration equivalence (not a physical-model validation)");
'''
            tests = tests.replace('    return 0;\n}',comparison+'    return 0;\n}')
        (folder/'test.c').write_text(source+tests,encoding='utf-8')
        subprocess.run([str(msvc/'bin/Hostx64/x64/cl.exe'),'/nologo','/utf-8',
                        '/std:c11','/Od','test.c',*extra,'/Fe:test.exe'],cwd=folder,env=env,check=True)
        subprocess.run([str(folder/'test.exe')],cwd=folder,check=True)


if __name__ == '__main__':
    main()
