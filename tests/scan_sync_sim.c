// 主机端仿真: 模拟测距板时钟偏差 + 无线突发延时 检验同步模块还原的θ误差
// 编译: gcc -O1 -I"Robot Project/RM开发板A型/User" tests/scan_sync_sim.c "Robot Project/RM开发板A型/User/Scan_Sync.c" -lm -o sim
// 运行: ./sim [时钟偏差] [突发间隔下限s] [突发间隔上限s] [丢包率]   例: ./sim 0.01 5e-3 25e-3 0.05
// 注意: 无线延时模型是假设的 不是E62模块的实测特性 结果只说明算法本身是否工作
#include "Scan_Sync.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

static double frand(void){ return rand() / (double)RAND_MAX; }
#define TWO_PI 6.283185307179586
// 测距舱真实角度(圈): 5圈/秒 带2%低频波动
static double theta_rev(double t_s){ return 5.0*t_s + 0.02*5.0/(TWO_PI*0.7)*sin(TWO_PI*0.7*t_s); }
static double x_m(double t_s){ return 0.05*t_s; }
#define RAW(i) ((uint16_t)(((i)*7)%3700))   // 每个采样一个可预测的原始值 用来检验拆包是否正确
static long bad_rho = 0;

int main(int argc, char **argv){
    double clock_err = argc>1 ? atof(argv[1]) : 0.01;   // 测距板时钟偏差
    double burst_min = argc>2 ? atof(argv[2]) : 5e-3, burst_max = argc>3 ? atof(argv[3]) : 25e-3;
    double loss = argc>4 ? atof(argv[4]) : 0.0;
    double base_latency = 3e-3, T = 1e-3*(1.0+clock_err), dur = 120.0;
    int n = (int)(dur/T);
    srand(1);
    Scan_Sync_Init();
    double *ts = malloc(sizeof(double)*n);
    double next_pose = 0.0, uart_free = 0.0, burst_t = 0.01;
    int sent = 0;                       // 已交给无线的采样数
    static uint8_t buf[112]; int blen = 0; double last_byte_t = 0;
    double *es=malloc(sizeof(double)*n), *ns=malloc(sizeof(double)*n), *arr=calloc(n,sizeof(double)); long cnt=0;
    double xs2=0;
    for(;;){
        // 下一次无线突发: 把此刻之前(减去基础延时)采到的帧一起送出
        burst_t += burst_min + (burst_max-burst_min)*frand();
        if(burst_t > dur) break;
        // 一帧16个采样 最后一个采样完成后这一帧才交给无线
        int upto = (int)((burst_t - base_latency)/T); if(upto>n) upto=n;
        double t = burst_t > uart_free ? burst_t : uart_free;
        for(; sent + SCAN_SAMPLES_PER_FRAME <= upto; sent += SCAN_SAMPLES_PER_FRAME){
            for(int k=0;k<SCAN_SAMPLES_PER_FRAME;k++) ts[sent+k] = (sent+k)*T;
            if(frand() < loss) continue;
            uint8_t f[SCAN_FRAME_LENGTH] = {0xA5, sent&0xFF, (sent>>8)&0xFF};
            for(int k=0;k<SCAN_SAMPLES_PER_FRAME;k+=2){
                uint16_t ra = RAW(sent+k), rb = RAW(sent+k+1); uint8_t *g = &f[3+(k/2)*3];
                g[0] = ra&0xFF; g[1] = ((ra>>8)&0x0F) | ((rb&0x0F)<<4); g[2] = rb>>4;
            }
            f[SCAN_FRAME_LENGTH-1] = Scan_Sync_Frame_Checksum(f);
            for(int b=0;b<SCAN_FRAME_LENGTH;b++){
                t += 86.8e-6; last_byte_t = t;
                // 字节到达前, 先把到期的位姿反馈推进去
                while(next_pose <= t){ double pt = next_pose + 30e-6*frand();
                    Scan_Sync_Push_Pose((uint32_t)(pt*1e6), ((uint32_t)(fmod(theta_rev(pt),1.0)*8192))&8191, (float)x_m(pt)); next_pose += 1e-3; }
                buf[blen++] = f[b];
                if(blen==112){ Scan_Sync_Rx_Data(buf, blen, (uint32_t)(t*1e6)); blen=0; }
            }
            for(int k=0;k<SCAN_SAMPLES_PER_FRAME;k++) arr[sent+k]=t;
        }
        uart_free = t;
        if(blen){ double ti = last_byte_t + 100e-6; // 空闲中断
            while(next_pose <= ti){ double pt = next_pose; Scan_Sync_Push_Pose((uint32_t)(pt*1e6), ((uint32_t)(fmod(theta_rev(pt),1.0)*8192))&8191, (float)x_m(pt)); next_pose += 1e-3; }
            Scan_Sync_Rx_Data(buf, blen, (uint32_t)(ti*1e6)); blen=0; }
        Struct_Scan_Record r;
        while(Scan_Sync_Pop_Record(&r)){
            // 仿真里没有重启, 32位序号 = 原始序号 - 首帧序号
            
            uint32_t idx = r.Sequence; // 首帧序号为0时两者相等
            if(idx >= (uint32_t)n) continue;
            if(r.Rho_um != Scan_Sync_Raw_To_Rho_um(RAW(idx))) bad_rho++;
            double tt = ts[idx]; if(tt < 40.0) continue;      // 前10秒为收敛期
            double truth = fmod(theta_rev(tt),1.0), got = r.Theta/65536.0;
            double e = got - truth; e -= floor(e+0.5);
            es[cnt]=e*360.0; double ne=fmod(theta_rev(arr[idx]),1.0)-truth; ne-=floor(ne+0.5); ns[cnt]=ne*360.0;
            double xe = r.X_m - x_m(tt); xs2 += xe*xe;
            cnt++;
        }
    }
    Struct_Scan_Statistics s; Scan_Sync_Get_Statistics(&s);
    double m=0,nm=0,sum2=0,maxe=0,nsum2=0,nmaxe=0; for(long i=0;i<cnt;i++){m+=es[i];nm+=ns[i];} m/=cnt; nm/=cnt;
    for(long i=0;i<cnt;i++){ double d=es[i]-m; sum2+=d*d; if(fabs(d)>maxe)maxe=fabs(d); d=ns[i]-nm; nsum2+=d*d; if(fabs(d)>nmaxe)nmaxe=fabs(d);}
    printf("时钟偏差=%+.1f%% 突发=%g~%gms 丢包=%g | 点数=%ld | 同步后角度误差 RMS=%.3f度 最大=%.3f度 (固定偏置%.1f度) | 按到达时刻配 RMS=%.2f度 最大=%.2f度 | 里程RMS=%.0fum | 估计周期=%.3fus(真值%.3f) 拆包错=%ld 帧=%u 校验错=%u 丢点=%u 无位姿=%u 溢出=%u\n",
        clock_err*100, burst_min*1e3, burst_max*1e3, loss, cnt, sqrt(sum2/cnt), maxe, m, sqrt(nsum2/cnt), nmaxe, sqrt(xs2/cnt)*1e6,
        s.Period_Q16/65536.0, T*1e6, bad_rho, s.Frames_Ok, s.Checksum_Errors, s.Lost_Samples, s.No_Pose_Drops, s.Overflow_Drops);
    return 0;
}
