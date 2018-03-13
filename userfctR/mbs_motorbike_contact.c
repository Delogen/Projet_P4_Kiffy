#include <math.h>
#include <stdio.h>

#include "useful_functions.h"
#include "mbs_motorbike_contact.h"

#define sign(a) ( (a) < 0 )

int F_lat(double *F_y0, double *B_y, double *D_y, double *K_yalpha, double beta, double gamma, double kappa, double F_z, double df_z, TIRE_param_strct* tire_param_strct);

TIRE_param_strct* init_TIRE_param_strct_120_70()
{

    TIRE_param_strct* tire_param_strct;

    tire_param_strct = (TIRE_param_strct*)malloc(sizeof(TIRE_param_strct));

    // Longitudinal parameter for the tire

    tire_param_strct->C_x = 1.6064; // value for 160/70

    tire_param_strct->p_Dx1 = 1.381;
    tire_param_strct->p_Dx2 = -0.04143;

    tire_param_strct->p_Ex1 = 0.0263; // value for 160/70
    tire_param_strct->p_Ex2 = 0.27056; // value for 160/70
    tire_param_strct->p_Ex3 = -0.0769; // value for 160/70
    tire_param_strct->p_Ex4 = 1.1268; // value for 160/70

    tire_param_strct->p_Kx1 = 25.94; // value for 160/70
    tire_param_strct->p_Kx2 = -4.233; // value for 160/70
    tire_param_strct->p_Kx3 = 0.3369; // value for 160/70


    tire_param_strct->F_z0 = 1600.0; // value for 160/70

    // Lateral parameter for the tire

    tire_param_strct->C_y = 0.8327;

    tire_param_strct->p_Dy1 = 1.3;
    tire_param_strct->p_Dy2 = 0.0;
    tire_param_strct->p_Dy3 = 0.0;

    tire_param_strct->p_Ey1 = -1.2556;
    tire_param_strct->p_Ey2 = -3.2068;
    tire_param_strct->p_Ey4 = -3.998;

    tire_param_strct->p_Ky1 = 22.841;
    tire_param_strct->p_Ky2 = 2.1578;
    tire_param_strct->p_Ky3 = 2.5058;
    tire_param_strct->p_Ky4 = -0.08088;
    tire_param_strct->p_Ky5 = -0.22882;
    tire_param_strct->p_Ky6 = 0.69677;
    tire_param_strct->p_Ky7 = -0.03077;

    tire_param_strct->C_gamma = 0.86765;
    tire_param_strct->E_gamma = -15.815;

    // Aligning parameter for the tire

    tire_param_strct->C_t = 1.0917;

    tire_param_strct->q_Bz1 = 10.486;
    tire_param_strct->q_Bz2 = -0.001154;
    tire_param_strct->q_Bz5 = -0.68973;
    tire_param_strct->q_Bz6 = 1.0411;
    tire_param_strct->q_Bz9 = 27.445;
    tire_param_strct->q_Bz10 = -1.0792;

    tire_param_strct->q_Dz1 = 0.19796;
    tire_param_strct->q_Dz2 = 0.06563;
    tire_param_strct->q_Dz3 = 0.2199;
    tire_param_strct->q_Dz4 = 0.21866;
    tire_param_strct->q_Dz8 = 0.3682;
    tire_param_strct->q_Dz9 = 0.1218;
    tire_param_strct->q_Dz10 = 0.25439;
    tire_param_strct->q_Dz11 = -0.17873;

    tire_param_strct->q_Ez1 = -0.91586;
    tire_param_strct->q_Ez2 = 0.11625;
    tire_param_strct->q_Ez5 = 1.4387;

    tire_param_strct->q_Hz3 = -0.003789;
    tire_param_strct->q_Hz4 = -0.01557;

    tire_param_strct->R_0 = 0.060; // CAUTION ONLY FOR 120/70 TIRE

    // Combined slip parameter

    tire_param_strct->r_Bx1 = 13.476; // value for 160/70
    tire_param_strct->r_Bx2 = 11.354; // value for 160/70
    tire_param_strct->C_xalpha = 1.1231; // value for 160/70

    tire_param_strct->r_By1 = 7.7856; // value for 160/70
    tire_param_strct->r_By2 = 8.1697; // value for 160/70
    tire_param_strct->r_By3 = -0.05914; // value for 160/70
    tire_param_strct->C_ykappa = 1.0533; // value for 160/70

    return tire_param_strct;
}
TIRE_param_strct* init_TIRE_param_strct_140_70_beta()
{

    TIRE_param_strct* tire_param_strct;

    tire_param_strct = (TIRE_param_strct*)malloc(sizeof(TIRE_param_strct));

    // Longitudinal parameter for the tire

    tire_param_strct->C_x = 1.6064;

    tire_param_strct->p_Dx1 = 1.2017;
    tire_param_strct->p_Dx2 = -0.0922;

    tire_param_strct->p_Ex1 = 0.0263;
    tire_param_strct->p_Ex2 = 0.27056;
    tire_param_strct->p_Ex3 = -0.0769;
    tire_param_strct->p_Ex4 = 1.1268;

    tire_param_strct->p_Kx1 = 25.94;
    tire_param_strct->p_Kx2 = -4.233;
    tire_param_strct->p_Kx3 = 0.3369;


    tire_param_strct->F_z0 = 1600.0;

    // Lateral parameter for the tire

    tire_param_strct->C_y = 0.93921;

    tire_param_strct->p_Dy1 = 1.1524;
    tire_param_strct->p_Dy2 = -0.01794;
    tire_param_strct->p_Dy3 = -0.06531;

    tire_param_strct->p_Ey1 = -0.94635;
    tire_param_strct->p_Ey2 = -0.09845;
    tire_param_strct->p_Ey4 = -1.6416;

    tire_param_strct->p_Ky1 = 26.601;
    tire_param_strct->p_Ky2 = 1.0167;
    tire_param_strct->p_Ky3 = 1.4989;
    tire_param_strct->p_Ky4 = 0.52567;
    tire_param_strct->p_Ky5 = -0.24064;
    tire_param_strct->p_Ky6 = 0.7667;
    tire_param_strct->p_Ky7 = 0.0;

    tire_param_strct->C_gamma = 0.50732;
    tire_param_strct->E_gamma = -4.7481;

    // Aligning parameter for the tire

    tire_param_strct->C_t = 1.3115;

    tire_param_strct->q_Bz1 = 10.354;
    tire_param_strct->q_Bz2 = 4.3004;
    tire_param_strct->q_Bz5 = -0.34033;
    tire_param_strct->q_Bz6 = -0.13202;
    tire_param_strct->q_Bz9 = 10.118;
    tire_param_strct->q_Bz10 = -1.0508;

    tire_param_strct->q_Dz1 = 0.20059;
    tire_param_strct->q_Dz2 = 0.05282;
    tire_param_strct->q_Dz3 = -0.21116;
    tire_param_strct->q_Dz4 = -0.15941;
    tire_param_strct->q_Dz8 = 0.30941;
    tire_param_strct->q_Dz9 = 0.0;
    tire_param_strct->q_Dz10 = 0.10037;
    tire_param_strct->q_Dz11 = 0.0;

    tire_param_strct->q_Ez1 = -0.39247; // caution: fault in article
    tire_param_strct->q_Ez2 = 0.10809; // caution: fault in article
    tire_param_strct->q_Ez5 = 0.9836;

    tire_param_strct->q_Hz3 = -0.04908;
    tire_param_strct->q_Hz4 = 0.0;

    tire_param_strct->R_0 = 0.070; // CAUTION ONLY FOR 140/70 TIRE

    // Combined slip parameter

    tire_param_strct->r_Bx1 = 13.476;
    tire_param_strct->r_Bx2 = 11.354;
    tire_param_strct->C_xalpha = 1.1231;

    tire_param_strct->r_By1 = 7.7856;
    tire_param_strct->r_By2 = 8.1697;
    tire_param_strct->r_By3 = -0.05914;
    tire_param_strct->C_ykappa = 1.0533;

    return tire_param_strct;
}
TIRE_param_strct* init_TIRE_param_strct_160_70()
{

    TIRE_param_strct* tire_param_strct;

    tire_param_strct = (TIRE_param_strct*)malloc(sizeof(TIRE_param_strct));

    // Longitudinal parameter for the tire

    tire_param_strct->C_x = 1.6064;

    tire_param_strct->p_Dx1 = 1.2017;
    tire_param_strct->p_Dx2 = -0.0922;

    tire_param_strct->p_Ex1 = 0.0263;
    tire_param_strct->p_Ex2 = 0.27056;
    tire_param_strct->p_Ex3 = -0.0769;
    tire_param_strct->p_Ex4 = 1.1268;

    tire_param_strct->p_Kx1 = 25.94;
    tire_param_strct->p_Kx2 = -4.233;
    tire_param_strct->p_Kx3 = 0.3369;


    tire_param_strct->F_z0 = 1600.0;

    // Lateral parameter for the tire

    tire_param_strct->C_y = 0.93921;

    tire_param_strct->p_Dy1 = 1.1524;
    tire_param_strct->p_Dy2 = -0.01794;
    tire_param_strct->p_Dy3 = -0.06531;

    tire_param_strct->p_Ey1 = -0.94635;
    tire_param_strct->p_Ey2 = -0.09845;
    tire_param_strct->p_Ey4 = -1.6416;

    tire_param_strct->p_Ky1 = 26.601;
    tire_param_strct->p_Ky2 = 1.0167;
    tire_param_strct->p_Ky3 = 1.4989;
    tire_param_strct->p_Ky4 = 0.52567;
    tire_param_strct->p_Ky5 = -0.24064;
    tire_param_strct->p_Ky6 = 0.7667;
    tire_param_strct->p_Ky7 = 0.0;

    tire_param_strct->C_gamma = 0.50732;
    tire_param_strct->E_gamma = -4.7481;

    // Aligning parameter for the tire

    tire_param_strct->C_t = 1.3115;

    tire_param_strct->q_Bz1 = 10.354;
    tire_param_strct->q_Bz2 = 4.3004;
    tire_param_strct->q_Bz5 = -0.34033;
    tire_param_strct->q_Bz6 = -0.13202;
    tire_param_strct->q_Bz9 = 10.118;
    tire_param_strct->q_Bz10 = -1.0508;

    tire_param_strct->q_Dz1 = 0.20059;
    tire_param_strct->q_Dz2 = 0.05282;
    tire_param_strct->q_Dz3 = -0.21116;
    tire_param_strct->q_Dz4 = -0.15941;
    tire_param_strct->q_Dz8 = 0.30941;
    tire_param_strct->q_Dz9 = 0.0;
    tire_param_strct->q_Dz10 = 0.10037;
    tire_param_strct->q_Dz11 = 0.0;

    tire_param_strct->q_Ez1 = -0.39247; // caution: fault in article
    tire_param_strct->q_Ez2 = 0.10809; // caution: fault in article
    tire_param_strct->q_Ez5 = 0.9836;

    tire_param_strct->q_Hz3 = -0.04908;
    tire_param_strct->q_Hz4 = 0.0;

    tire_param_strct->R_0 = 0.080; // CAUTION ONLY FOR 160/70 TIRE

    // Combined slip parameter

    tire_param_strct->r_Bx1 = 13.476;
    tire_param_strct->r_Bx2 = 11.354;
    tire_param_strct->C_xalpha = 1.1231;

    tire_param_strct->r_By1 = 7.7856;
    tire_param_strct->r_By2 = 8.1697;
    tire_param_strct->r_By3 = -0.05914;
    tire_param_strct->C_ykappa = 1.0533;

    return tire_param_strct;
}
void free_TIRE_param_strct(TIRE_param_strct* tire_param_strct)
{
    free(tire_param_strct);
}

int mbs_motorbike_contact(double F[4], 
                          double M[4],
                          double beta,
                          double gamma,
                          double kappa,
                          double Vct_geo_s[4],
                          TIRE_param_strct* tire_param_strct)
{
    int res1, res2;
    double pre_1;
    double F_z, df_z, D_x, E_x, K_xkappa, B_x, F_x0;
    double F_y0, B_y, D_y, K_yalpha;
    double F_y0_gamma0, B_y_gamma0, D_y_gamma0, K_yalpha_gamma0;
    double S_Hr, B_t, D_t, E_t, B_r, D_r;
    double F_x, F_y, B_xalpha, B_ykappa;
    double F_y_gamma0, lambda_t, lambda_r, M_zr, M_z;

    F_z = F[3];

    if (Vct_geo_s[1] < 0.0)
    {
        beta = -beta;
    }

    // Longitudinal force
    df_z = (F_z - tire_param_strct->F_z0)/tire_param_strct->F_z0;
    D_x = (tire_param_strct->p_Dx1 + tire_param_strct->p_Dx2 * df_z)*F_z;
    E_x = (tire_param_strct->p_Ex1 + tire_param_strct->p_Ex2 * df_z + tire_param_strct->p_Ex3 * df_z * df_z) * (1.0-tire_param_strct->p_Ex3 * sign(kappa));
    K_xkappa = F_z * (tire_param_strct->p_Kx1+tire_param_strct->p_Kx2 * df_z)*exp(tire_param_strct->p_Kx3 * df_z);
    B_x = K_xkappa / (tire_param_strct->C_x*D_x);

    F_x0 = D_x * sin(tire_param_strct->C_x*atan(B_x*kappa-E_x*(B_x*kappa-atan(B_x*kappa))));

    // Lateral force
    res1 =  F_lat(&F_y0, &B_y, &D_y, &K_yalpha, beta, gamma, kappa,  F_z, df_z, tire_param_strct);
    res2 =  F_lat(&F_y0_gamma0, &B_y_gamma0, &D_y_gamma0, &K_yalpha_gamma0, beta, 0.0, kappa, F_z, df_z, tire_param_strct); // call with null camber for aligning moment calculation

    // Aligning moment
    S_Hr = (tire_param_strct->q_Hz3 + tire_param_strct->q_Hz4 * df_z) * gamma;
    B_t = (tire_param_strct->q_Bz1 + tire_param_strct->q_Bz2 * df_z) * (1.0 + tire_param_strct->q_Bz5 * fabs(gamma) + tire_param_strct->q_Bz6 * gamma * gamma);
    D_t = F_z * (tire_param_strct->R_0 / tire_param_strct->F_z0) * (tire_param_strct->q_Dz1 + tire_param_strct->q_Dz2 * df_z) * (1.0 + tire_param_strct->q_Dz3 * fabs(gamma) + tire_param_strct->q_Dz4 * gamma * gamma);
    E_t = (tire_param_strct->q_Ez1 + tire_param_strct->q_Ez2 * df_z) * (1.0 + tire_param_strct->q_Ez5 * gamma * (2.0 /3.141592654) * atan(B_t * tire_param_strct->C_t * beta));
    B_r = tire_param_strct->q_Bz9 + tire_param_strct->q_Bz10 * B_y * tire_param_strct->C_y;
    D_r = F_z * tire_param_strct->R_0 * ((tire_param_strct->q_Dz8 + tire_param_strct->q_Dz9 * df_z) * gamma + (tire_param_strct->q_Dz10 + tire_param_strct->q_Dz11 * df_z) * gamma * fabs(gamma)) / sqrt(1.0 + beta * beta);

    // combined result
    B_xalpha = tire_param_strct->r_Bx1 * cos(atan(tire_param_strct->r_Bx2 * kappa));
    F_x = cos(tire_param_strct->C_xalpha * atan(B_xalpha * beta)) * F_x0;

    B_ykappa = tire_param_strct->r_By1 * cos(atan(tire_param_strct->r_By2 * (beta - tire_param_strct->r_By3)));
    F_y = cos(tire_param_strct->C_ykappa * atan(B_ykappa * kappa)) * F_y0;

    F_y_gamma0 = cos(tire_param_strct->C_ykappa * atan(B_ykappa* kappa)) * F_y0_gamma0;
    pre_1 = (K_xkappa * kappa / K_yalpha_gamma0) * (K_xkappa * kappa / K_yalpha_gamma0);
    lambda_t = sqrt(beta * beta + pre_1)  * sign(beta);
    lambda_r = sqrt((beta + S_Hr) * (beta + S_Hr) + pre_1) * sign(beta + S_Hr);

    M_zr = D_r * cos(atan(B_r * lambda_r));
    M_z = -D_t * cos(tire_param_strct->C_t * atan(B_t * lambda_t - E_t * (B_t * lambda_t - atan(B_t * lambda_t)))) / sqrt(1.0 + beta * beta) * F_y_gamma0 + M_zr;
    
    if (Vct_geo_s[1] < 0.0)
    {
        M_z = -M_z;
    }

    // Filling Force and Torque vector
    F[1] = F_x;
    F[2] = -F_y; //negative sign because Robotran use ISO convention and article use SAE adapted convention
    F[3] = F_z;

    M[1] = 0.0;
    M[2] = 0.0;
    M[3] = -M_z; //negative sign because Robotran use ISO convention and article use SAE adapted convention

    return 1;
}

int F_lat(double *F_y0,double *B_y, double *D_y, double *K_yalpha,  double beta, double gamma, double kappa, double F_z, double df_z, TIRE_param_strct* tire_param_strct)
{
    double  E_y, K_ygamma, B_gamma;

    *D_y = F_z * tire_param_strct->p_Dy1 * exp(tire_param_strct->p_Dy2 * df_z) / (1.0 + tire_param_strct->p_Dy3 * gamma * gamma);
    E_y = tire_param_strct->p_Ey1 + tire_param_strct->p_Ey2 * gamma * gamma + tire_param_strct->p_Ey4 * gamma * sign(beta);
    *K_yalpha = tire_param_strct->p_Ky1 * tire_param_strct->F_z0 * sin(tire_param_strct->p_Ky2 * atan(F_z / ((tire_param_strct->p_Ky3 + tire_param_strct->p_Ky4 * gamma * gamma) * tire_param_strct->F_z0))) / (1.0 + tire_param_strct->p_Ky5 * gamma * gamma);
    *B_y = *K_yalpha / (tire_param_strct->C_y * *D_y);
    K_ygamma = (tire_param_strct->p_Ky6 + tire_param_strct->p_Ky7 * df_z) * F_z;
    B_gamma = K_ygamma / (tire_param_strct->C_gamma * *D_y);

    *F_y0 = *D_y * sin(tire_param_strct->C_y * atan(*B_y * beta - E_y * (*B_y * beta - atan(*B_y * beta))) + tire_param_strct->C_gamma * atan(B_gamma * gamma - tire_param_strct->E_gamma * (B_gamma * gamma - atan(B_gamma * gamma))));

    return 1;
}


