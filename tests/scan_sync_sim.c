// 主机端仿真(有线连接版): 检验同步模块配出的θ和x是否正确 以及拆包、丢点统计
// 编译: gcc -O1 -I"Robot Project/RM开发板A型/User" tests/scan_sync_sim.c "Robot Project/RM开发板A型/User/Scan_Sync.c" -lm -o sim
// 运行: ./sim [丢帧率] [测距板发送时刻的抖动上限s]   例: ./sim 0.01 100e-6
// 模型: 测距板每1ms采样一次 采完立刻发7字节帧 主控在帧收完约一个字节时间后进中断
#include "Scan_Sync.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static double frand(void){ return rand() / (double)RAND_MAX; }
#define TWO_PI 6.283185307179586
// 测距舱真实角度(圈): 5圈/秒 带2%低频波动
static double theta_rev(double t_s){ return 5.0*t_s + 0.02*5.0/(TWO_PI*0.7)*sin(TWO_PI*0.7*t_s); }
static double x_m(double t_s){ return 0.05*t_s; }
#define RAW1(i) ((uint16_t)(((i)*7)%3700))    // 每个采样一个可预测的原始值 用来检验拆包
#define RAW2(i) ((uint16_t)(((i)*13+5)%3700))

int main(int argc, char **argv){
    double loss = argc>1 ? atof(argv[1]) : 0.0, jitter = argc>2 ? atof(argv[2]) : 50e-6;
    double T = 1e-3*1.01, dur = 150.0;            // 测距板时钟快慢偏1% 150秒让16位序号绕回两次
    int n = (int)(dur/T);
    double next_pose = 0.0, sum=0, sum2=0, maxe=0, xs2=0; long cnt=0, bad=0, lost_true=0;
    double *err = malloc(sizeof(double)*n);
    srand(1);
    Scan_Sync_Init();
    for(int i=0;i<n;i++){
        double ts = 0.5 + i*T;                     // 真实采样时刻
        if(frand() < loss){ lost_true++; continue; }
        uint16_t r1 = RAW1(i), r2 = RAW2(i);
        uint8_t f[SCAN_FRAME_LENGTH] = {0xA5, i&0xFF, (i>>8)&0xFF, r1&0xFF, ((r1>>8)&0x0F)|((r2&0x0F)<<4), r2>>4, 0};
        f[SCAN_FRAME_LENGTH-1] = Scan_Sync_Frame_Checksum(f);
        double arrival = ts + jitter*frand() + SCAN_FRAME_LENGTH*86.8e-6 + 86.8e-6;   // 发完一帧 再过一个字节时间触发空闲中断
        while(next_pose <= arrival){ double pt = next_pose + 30e-6*frand();
            Scan_Sync_Push_Pose((uint32_t)(pt*1e6), ((uint32_t)(fmod(theta_rev(pt),1.0)*8192))&8191, (float)x_m(pt)); next_pose += 1e-3; }
        Scan_Sync_Rx_Data(f, SCAN_FRAME_LENGTH, (uint32_t)(arrival*1e6));
        Struct_Scan_Record r;
        while(Scan_Sync_Pop_Record(&r)){
            if(r.Rho1_um != Scan_Sync_Raw_To_Rho_um(r1) || r.Rho2_um != Scan_Sync_Raw_To_Rho_um(r2)) bad++;
            double e = r.Theta/65536.0 - fmod(theta_rev(ts),1.0); e -= floor(e+0.5); e *= 360.0;
            err[cnt++] = e; sum += e;
            double xe = r.X_m - x_m(ts); xs2 += xe*xe;
        }
    }
    double mean = sum/cnt;
    for(long i=0;i<cnt;i++){ double d = err[i]-mean; sum2 += d*d; if(fabs(d)>maxe) maxe=fabs(d); }
    Struct_Scan_Statistics s; Scan_Sync_Get_Statistics(&s);
    printf("丢帧率=%g 发送抖动=%.0fus | 测点=%ld | 角度误差 平均=%.3f度 波动RMS=%.3f度 最大=%.3f度 | 里程RMS=%.1fum | 拆包错=%ld 丢点统计=%u(真值%ld) 校验错=%u 无位姿=%u 溢出=%u\n",
        loss, jitter*1e6, cnt, mean, sqrt(sum2/cnt), maxe, sqrt(xs2/cnt)*1e6, bad, s.Lost_Samples, lost_true, s.Checksum_Errors, s.No_Pose_Drops, s.Overflow_Drops);
    return 0;
}
