/*sunyi******************************************* */
#include <math.h>
#include <string.h>

void fcn(double Ie_phi, double Ie_theta, double phi, double theta, double p, double q,
         double m11, double m13, double m22, double m24, double v_hat[2]) {
    // 状态向量构造
    double x1[4] = {Ie_phi, Ie_theta, phi, theta};
    double x2[2] = {p, q};
    double x[6] = {x1[0], x1[1], x1[2], x1[3], x2[0], x2[1]};

    // 系统矩阵定义
    double A11[4][4] = {{0,0,-1,0}, {0,0,0,-1}, {0,0,0,0}, {0,0,0,0}};
    double A12[4][2] = {{0,0}, {0,0}, {1,0}, {0,1}};
    //double A21[2][4] = {{0}};
    //double A22[2][2] = {{0}};

    // 动态构造M矩阵（使用输入参数）
    double M[2][4] = {{m11, 0, m13, 0},
                      {0, m22, 0, m24}};

    // 矩阵运算：A12*M
    double temp1[4][4] = {{0}};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 2; k++) {
                temp1[i][j] += A12[i][k] * M[k][j];
            }
        }
    }

    // 计算A11_hat = A11 - temp1
    double A11_hat[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            A11_hat[i][j] = A11[i][j] - temp1[i][j];
        }
    }

    // 计算A21_hat = M*A11_hat
    double A21_hat[2][4] = {{0}};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                A21_hat[i][j] += M[i][k] * A11_hat[k][j];
            }
        }
    }

    // 计算A22_hat = M*A12
    double A22_hat[2][2] = {{0}};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 4; k++) {
                A22_hat[i][j] += M[i][k] * A12[k][j];
            }
        }
    }

    // 动态构造S矩阵 [M|I]
    double S[2][6];
    for (int i = 0; i < 2; i++) {
        S[i][0] = M[i][0];    // 填充M部分
        S[i][1] = M[i][1];
        S[i][2] = M[i][2];
        S[i][3] = M[i][3];
        S[i][4] = (i == 0) ? 1.0 : 0.0;  // 单位矩阵部分
        S[i][5] = (i == 1) ? 1.0 : 0.0;
    }

    // 计算滑动模量s
    double s[2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 6; j++) {
            s[i] += S[i][j] * x[j];
        }
    }

    // 计算线性控制量v_hat_l
    double term1[2] = {0};  // A21_hat*x1
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            term1[i] += A21_hat[i][j] * x1[j];
        }
    }

    double temp4[2][2];  // A22_hat-PHI
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            temp4[i][j] = A22_hat[i][j] - ((i == j) ? -20.0 : 0.0);
        }
    }

    double term2[2] = {0};  // (A22_hat-PHI)*s
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            term2[i] += temp4[i][j] * s[j];
        }
    }

    double v_hat_l[2] = {-term1[0]-term2[0], -term1[1]-term2[1]};

    // 计算非线性控制量v_hat_n
    double P2[2][2] = {{0.025,0}, {0,0.025}};
    double P2_s[2] = {P2[0][0]*s[0] + P2[0][1]*s[1],
                     P2[1][0]*s[0] + P2[1][1]*s[1]};
    double norm_P2_s = sqrt(P2_s[0]*P2_s[0] + P2_s[1]*P2_s[1]);
    double s_norm = sqrt(s[0]*s[0] + s[1]*s[1]);

    double v_hat_n[2] = {0};
    if (s_norm < 1e-9) {  // 处理零值
        v_hat_n[0] = v_hat_n[1] = 0.0;
    } else if (norm_P2_s < 1e-9) {
        v_hat_n[0] = v_hat_n[1] = -0.02;  // 安全值
    } else {
        double inv_norm = 1.0 / norm_P2_s;
        v_hat_n[0] = -2.0 * (P2_s[0] * inv_norm + 0.01);
        v_hat_n[1] = -2.0 * (P2_s[1] * inv_norm + 0.01);
    }

    // 合成最终控制量
    v_hat[0] = v_hat_l[0] + v_hat_n[0];
    v_hat[1] = v_hat_l[1] + v_hat_n[1];
}


/********************************姿态控制2**************************************************** */


#define PI 3.14159265358979323846

// 辅助函数声明
void matrix_multiply(const double *A, int rowsA, int colsA,
                    const double *B, int rowsB, int colsB, double *result);
void matrix_transpose(const double *A, int rows, int cols, double *result);
int matrix_inverse_2x2(const double *A, double *inv);
void matrix_pinv(const double *A, int rows, int cols, double *result);

void fcn_2(
    // 输入参数
    double b,
    double p, double q, double r,
    double phi, double theta,
    const double v_hat_t[2],
    // 输出参数
    double *L, double *M
) {
    // 内置常量参数
    const double x_A[8] = {
        sqrt(3)*0.104,   // x_A1
        sqrt(3)*0.104,   // x_A2
        (sqrt(3)/2)*0.104, // x_A3
        0.0,            // x_A4
        -(sqrt(3)/2)*0.104,// x_A5
        0.0,            // x_A6
        -sqrt(3)*0.104, // x_A7
        -sqrt(3)*0.104  // x_A8
    };

    const double y_A[8] = {
        -1.5*0.104,     // y_A1
        1.5*0.104,      // y_A2
        0.0,            // y_A3
        -1.5*0.104,     // y_A4
        0.0,            // y_A5
        1.5*0.104,      // y_A6
        -1.5*0.104,     // y_A7
        1.5*0.104       // y_A8
    };

    const double J_params[9] = {
        13.694292985564024, 0.0, 0.0,          // inv_J_A前三元素
        0.0, 13.692396907752903, 0.0,          // inv_J_A中三元素
        0.0, 0.0, 6.847838937075155            // inv_J_A后三元素
    };

    const double Jxx_A = 0.07302312;
    const double Jyy_A = 0.07303323199999998;
    const double Jzz_A = 0.14603147199999997;
    const double Jxy_A = 0.0;
    const double Jyx_A = 0.0;

    /* 开始核心计算 */
    // 构造invJ (2x2)
    double invJ[2][2] = {{J_params[0], J_params[1]},
                         {J_params[3], J_params[4]}};

    // 构造Bp_tau (4x2)
    double Bp_tau[4][2] = {{0}};
    memcpy(Bp_tau[2], invJ[0], 2*sizeof(double));  // 第三行
    memcpy(Bp_tau[3], invJ[1], 2*sizeof(double));  // 第四行

    // 构造Bp_omega (2x8)
    double Bp_omega[2][8];
    for(int i=0; i<8; i++) {
        Bp_omega[0][i] = b * y_A[i];
        Bp_omega[1][i] = b * x_A[i];
    }

    // 计算Bp (4x8)
    double Bp[4][8] = {{0}};
    matrix_multiply(&Bp_tau[0][0], 4, 2, &Bp_omega[0][0], 2, 8, &Bp[0][0]);

    // 构造B矩阵 (6x8)
    double B[6][8] = {{0}};
    memcpy(&B[2][0], &Bp[0][0], 4*8*sizeof(double));

    // 构造Wi矩阵 (8x8单位矩阵)
    double Wi[8][8] = {{0}};
    for(int i=0; i<8; i++) Wi[i][i] = 1.0;

    // 计算角动量
    double Angular_Momentum[3] = {
        q*r*Jzz_A + r*p*Jyx_A - r*q*Jyy_A,
        r*p*Jxx_A - r*q*Jxy_A - p*r*Jzz_A,
        p*q*Jyy_A + q*q*Jxy_A - p*p*Jyx_A - p*q*Jxx_A
    };

    // 计算非线性项
    double nonliner[3];
    const double inv_J_A[3][3] = {{J_params[0], J_params[1], J_params[2]},
                                 {J_params[3], J_params[4], J_params[5]},
                                 {J_params[6], J_params[7], J_params[8]}};
    matrix_multiply(&inv_J_A[0][0], 3, 3, Angular_Momentum, 3, 1, nonliner);

    // 计算zeta
    double zeta[4] = {
        q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta),
        (cos(phi)-1)*q - sin(phi)*r,
        nonliner[0],
        nonliner[1]
    };

    // 计算inv_B = Wi*B'*pinv(B*Wi*Wi*B')
    double B_T[8][6], BWiWiB_T[6][6];
    double BWi[2][8];
    matrix_transpose(&B[0][0], 6, 8, &B_T[0][0]);
    matrix_multiply(&B[0][0], 6, 8, &Wi[0][0], 8, 8, &BWi[0][0]);
    matrix_multiply(&BWi[0][0], 6, 8, &B_T[0][0], 8, 6, &BWiWiB_T[0][0]);

    double pinv_BWiWiB_T[6][6];
    matrix_pinv(&BWiWiB_T[0][0], 6, 6, &pinv_BWiWiB_T[0][0]);

    double inv_B[8][6];
    matrix_multiply(&B_T[0][0], 8, 6, &pinv_BWiWiB_T[0][0], 6, 6, &inv_B[0][0]);

    // 计算u_F
    double E_zeta[6] = {0};
    const double E[6][4] = {{0}, {0}, {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
    matrix_multiply(&E[0][0], 6, 4, zeta, 4, 1, E_zeta);

    double u_F[8] = {0};
    matrix_multiply(&inv_B[0][0], 8, 6, E_zeta, 6, 1, u_F);
    for(int i=0; i<8; i++) u_F[i] *= -1;

    // 提取B的最后两行 (5、6行)
    double B2[2][8];
    memcpy(B2[0], B[4], 8*sizeof(double));
    memcpy(B2[1], B[5], 8*sizeof(double));

    // 计算inv_B2

    double B2WiWiB2_T[2][2];
    double B2Wi[2][8];
    matrix_multiply(&B2[0][0], 2, 8, &Wi[0][0], 8, 8, &B2Wi[0][0]);
    matrix_multiply(&B2Wi[0][0], 2, 8, &B2[0][0], 8, 2, &B2WiWiB2_T[0][0]);

    double pinv_B2WiWiB2_T[2][2];
    matrix_pinv(&B2WiWiB2_T[0][0], 2, 2, &pinv_B2WiWiB2_T[0][0]);

    double inv_B2[8][2];
    matrix_multiply(&B2[0][0], 2, 8, &pinv_B2WiWiB2_T[0][0], 2, 2, &inv_B2[0][0]);

    // 计算最终输出
    double u[8] = {0};
    matrix_multiply(&inv_B2[0][0], 8, 2, v_hat_t, 2, 1, u);

    double up[8];
    for(int i=0; i<8; i++) up[i] = u[i] + u_F[i];

    // 计算tau_p
    double tau_p[2] = {0};
    matrix_multiply(&Bp_omega[0][0], 2, 8, up, 8, 1, tau_p);

    *L = tau_p[0];
    *M = tau_p[1];
}

// 矩阵乘法
void matrix_multiply(const double *A, int rowsA, int colsA,
                    const double *B, int rowsB, int colsB, double *result) {
    memset(result, 0, rowsA*colsB*sizeof(double));
    for(int i=0; i<rowsA; i++) {
        for(int k=0; k<colsA; k++) {
            for(int j=0; j<colsB; j++) {
                result[i*colsB + j] += A[i*colsA + k] * B[k*colsB + j];
            }
        }
    }
}

// 矩阵转置
void matrix_transpose(const double *A, int rows, int cols, double *result) {
    for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
            result[j*rows + i] = A[i*cols + j];
        }
    }
}

// 2x2矩阵求逆
int matrix_inverse_2x2(const double *A, double *inv) {
    double det = A[0]*A[3] - A[1]*A[2];
    if(fabs(det) < 1e-12) return 0;
    double inv_det = 1.0 / det;
    inv[0] =  A[3] * inv_det;
    inv[1] = -A[1] * inv_det;
    inv[2] = -A[2] * inv_det;
    inv[3] =  A[0] * inv_det;
    return 1;
}

// 伪逆计算（仅处理满秩情况）
void matrix_pinv(const double *A, int rows, int cols, double *result) {
    // 假设矩阵可逆
    double A_T[cols*rows];
    matrix_transpose(A, rows, cols, A_T);

    if(rows >= cols) {
        double ATA[cols*cols];
        matrix_multiply(A_T, cols, rows, A, rows, cols, ATA);

        double inv_ATA[cols*cols];
        if(cols == 2) matrix_inverse_2x2(ATA, inv_ATA);

        matrix_multiply(inv_ATA, cols, cols, A_T, cols, rows, result);
    } else {
        double AAT[rows*rows];
        matrix_multiply(A, rows, cols, A_T, cols, rows, AAT);

        double inv_AAT[rows*rows];

        matrix_multiply(A_T, cols, rows, inv_AAT, rows, rows, result);
    }
}
