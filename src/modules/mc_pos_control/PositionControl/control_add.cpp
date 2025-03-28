#include <math.h>
#include <stdio.h>


// 控制器函数
void sliding_mode_controller(
    // 输入参数
    double Iex, double Iey, double Iez,
    double xd, double yd, double zd,
    double xe, double ye, double ze,
    double dxe, double dye, double dze,
    // 滑模参数
    double sa1, double sa2, double sa3,
    double sa4, double sa5, double sa6,
    // 输出参数
    double u_o[3]  // 3维控制输出
) {



    /* 1. 矩阵初始化（使用静态存储避免栈溢出）*/
    double xa[9] = {Iex, Iey, Iez, xe, ye, ze, dxe, dye, dze};  // 增广状态向量

    //printf("%f", (double)xa[6]);

    // 滑模矩阵 Sa[3][9]
     const double Sa[3][9] = {
        {sa1, 0,   0,   sa4, 0,   0,   1, 0, 0},  // 第1行
        {0,   sa2, 0,   0,   sa5, 0,   0, 1, 0},  // 第2行
        {0,   0,   sa3, 0,   0,   sa6, 0, 0, 1}   // 第3行
    };



    // 系统矩阵 Aa[9][9]（静态存储）
     double Aa[9][9] = {0};
    // 初始化Aa矩阵（修正初始化逻辑）
    for(int i=0; i<9; i++) {
        if(i < 3) {
            // 前3行：行0-2的列3-5设为-1
            Aa[i][i+3] = -1.0;
        } else {
            // 后6行：行3-8的对角线设为1
            Aa[i][i] = 1.0;     // 修正：正确初始化单位矩阵部分
        }
    }

    // 控制矩阵 Bc[9][3]（静态存储）
     const double Bc[9][3] = {
        {1,0,0}, {0,1,0}, {0,0,1},  // 前3行
        {0,0,0}, {0,0,0}, {0,0,0},  // 中间3行
        {0,0,0}, {0,0,0}, {0,0,0}   // 后3行
    };

    /* 2. 计算滑模面 sa = Sa * xa */
    double sa[3] = {0};
    for(int i=0; i<3; i++) {
        for(int j=0; j<9; j++) {
            sa[i] += Sa[i][j] * xa[j];
        }
    }

    /* 3. 计算线性控制部分 u_ol */
    // PHI_a矩阵（恒定参数）
     const double PHI_a[3][3] = {
        {-2.0, 0.0,  0.0},
        {0.0,  -2.0, 0.0},
        {0.0,  0.0, -2.0}
    };

    // 中间矩阵计算（使用静态存储）
     double SaAa[3][9] = {0};
     double PHI_aSa[3][9] = {0};

    // 矩阵乘法 Sa * Aa
    for(int i=0; i<3; i++) {
        for(int j=0; j<9; j++) {
            for(int k=0; k<9; k++) {
                SaAa[i][j] += Sa[i][k] * Aa[k][j];
            }
        }
    }

    // 矩阵乘法 PHI_a * Sa
    for(int i=0; i<3; i++) {
        for(int j=0; j<9; j++) {
            for(int k=0; k<3; k++) {
                PHI_aSa[i][j] += PHI_a[i][k] * Sa[k][j];
            }
        }
    }

    // 计算La = -(SaAa - PHI_aSa)
     double La[3][9] = {0};
    for(int i=0; i<3; i++) {
        for(int j=0; j<9; j++) {
            La[i][j] = -(SaAa[i][j] - PHI_aSa[i][j]);
        }
    }

    // 计算 Lc = -Sa * Bc
     double Lc[3][3] = {0};
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            for(int k=0; k<9; k++) {
                Lc[i][j] -= Sa[i][k] * Bc[k][j];
            }
        }
    }

    // 计算线性控制量 u_ol = La*xa + Lc*yc
    double u_ol[3] = {0};
    const double yc[3] = {xd, yd, zd};
    for(int i=0; i<3; i++) {
        // La*xa 部分
        for(int j=0; j<9; j++) {
            u_ol[i] += La[i][j] * xa[j];
        }
        // Lc*yc 部分
        for(int j=0; j<3; j++) {
            u_ol[i] += Lc[i][j] * yc[j];
        }
    }

    /* 4. 计算非线性控制部分（增强数值稳定性）*/
    double u_on[3] = {0};
    const double epsilon = 1e-10;  // 更严格的防零阈值
    const double sa_norm = sqrt(sa[0]*sa[0] + sa[1]*sa[1] + sa[2]*sa[2]);

    if(sa_norm > epsilon) {
        const double inv_norm = 1.0 / sa_norm;
        for(int i=0; i<3; i++) {
            u_on[i] = -0.02 * (sa[i] * inv_norm + 0.1);
        }
    } else {
        // 滑模面接近零时的处理
        for(int i=0; i<3; i++) {
            u_on[i] = -0.02 * 0.1;  // 保持基本阻尼项
        }
    }

    /* 5. 合成最终控制量 */
    for(int i=0; i<3; i++) {
        u_o[i] = u_ol[i] + u_on[i];
        if(u_o[i] < -100 || u_o[i] >100){
            u_o[i] = 100 * u_o[i]/abs(u_o[i]);
        }
    }



}
