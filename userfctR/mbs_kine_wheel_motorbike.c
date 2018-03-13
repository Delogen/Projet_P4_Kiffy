//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"
#include "MBSfun.h"
#include "mbs_data.h"
#include "mbs_matrix.h"
#include "mbs_message.h"

#include "mbs_motorbike_contact.h"

void mbs_kine_wheel_motorbike(double Pw[4],double Rw[4][4],
                       double Vw[4],double OMw[4],
                       double tsim,int iwhl, double r_rim, double r_t_tire,
                       double *penp, double *rzp, double *anglisp, double *angcambp,
                       double *glissp, double Vct[4], double Vct_geo_s[4], double Rtsol[4][4], double dxF[4])
{
    double Rtw[4][4], Rsol[4][4], Rtg[4][4], Rttg[4][4];
    double pen, rz, anglis, angcamb, gliss;

    double ex[4], ey[4], eyP[4], ez[4];
    double OMw_w[4], OMwf[4], OMwf_w[4];
    double Vcts[4], Vct_geo[4], Vws[4];
    double vrz_I[4], vrz[4], vx1[4];

    double sinf, tg_anglis;
    double Zgnd, Pct[4];

    double z[4]={0,0,0,1};

    double vr_rim[4], vr_rim_I[4], P_C_I[4], CG[4], vct_I[4];

    /* Basic hypothesis
     *  > ground is (locally) flat and horizontal
     *  > The camber angle is loawer thant pi/4
     *
     *
     * Conventions :
     *  > The local wheel axis is Y (joint R2)
     *  > Frames:
     *    > [Rw]  = Wheel Frame
     *    > [I]   = Inertial Frame
     *    > [Rtg] = Frame tangential to the wheel
     *    > [Rsol]= Frame on the ground aligned to the wheel
     *
     * Remark: [Rw] = Rw*[I]
     */
    transpose(Rw,Rtw);

/*    Frames definition */

    //ey = Wheel axis in [I]
    ey[1] = Rw[2][1];
    ey[2] = Rw[2][2];
    ey[3] = Rw[2][3];

    //eyP = Projection on the ground of the wheel axis in [I]
    eyP[1] = Rw[2][1];
    eyP[2] = Rw[2][2];
    eyP[3] = 0;
    normalize(eyP,eyP);

    //ex = Vector tangential to the Wheel on the ground in [I]
    cross_product(eyP,z,ex);

    //[Rsol]=Rsol*[I]
    Rsol[1][1] = ex[1];
    Rsol[1][2] = ex[2];
    Rsol[1][3] = ex[3];

    Rsol[2][1] = eyP[1];
    Rsol[2][2] = eyP[2];
    Rsol[2][3] = eyP[3];

    Rsol[3][1] = 0;
    Rsol[3][2] = 0;
    Rsol[3][3] = 1;

    transpose(Rsol,Rtsol);

    //ez = Vector to the Wheel center in [I]
    cross_product(ex,ey,ez);
    
    //[Rtg]=Rtg*[I]
    Rtg[1][1] = ex[1];
    Rtg[1][2] = ex[2];
    Rtg[1][3] = ex[3];

    Rtg[2][1] = ey[1];
    Rtg[2][2] = ey[2];
    Rtg[2][3] = ey[3];

    Rtg[3][1] = ez[1];
    Rtg[3][2] = ez[2];
    Rtg[3][3] = ez[3];

    transpose(Rtg,Rttg);


/*    CAMBER ANGLE */
    sinf=Rw[2][3];      // Only valid on horizontal ground !
    angcamb=asin(sinf);
    *angcambp = angcamb;
    
/*    CONTACT POINT AND VELOCITIES */
// POSITIONS
    // Ground Z level; assumed to be 0.0.
    Zgnd = 0.0;
    // To define another value user can define a function like : double user_GroundLevel(Pw[1],Pw[2],MBSdata,tsim,iwhl);

    // rz: distance in the wheel plane (radius) between wheel center and ground.
    rz = (Pw[3]-Zgnd)/cos(angcamb);
    *rzp = rz;

    // vr_rim: Rim radius vector in [Rtg]
    vr_rim[1] = 0.0;
    vr_rim[2] = 0.0;
    vr_rim[3] = -r_rim;

    // vr_rim_I: Rim radius vector in [I]
    matrix_product(Rttg,vr_rim,vr_rim_I);

    // P_C_I: Closest point on rim from ground in [I]
    vector_sum(Pw,vr_rim_I,P_C_I);

    // Pct: Contact point, located at the vertical of P_C_I in [I]
    Pct[1] = P_C_I[1];
    Pct[2] = P_C_I[2];
    Pct[3] = Zgnd;

    // Computation of the deflection of the tire
    pen = -P_C_I[3] + Zgnd + r_t_tire;
    *penp = pen;

    // vct_I: Vector from wheel center to contact point in [I]
    CG[1] = 0.0;
    CG[2] = 0.0;
    CG[3] = -P_C_I[3];
    vector_sum(vr_rim_I,CG,vct_I);

    // dxF: Vector from wheel center to contact point in [Rw]
    matrix_product(Rw,vct_I,dxF);

// VELOCITIES
    // vx1: Velocity of the material contact point (on the tire) due to angular velocity of the wheel in [I]
    cross_product(OMw,vct_I,vx1);

    // Vct: Velocity of the material contact point (on the tire) in [I]
    vector_sum(Vw,vx1,Vct);

    // Vcts: Velocity of the material contact point (on the tire) in [Rsol]
    matrix_product(Rsol,Vct,Vcts);

/* SLIDESLIP ANGLE (angliss) */
// Angle between the velocity of the geometrical point and the plane of the wheel

    // Vws: Velocity of the wheel center in [Rsol]
    matrix_product(Rsol,Vw,Vws);

// Velocity of the geometrial point:
    // OMwf: Angular velocity vector of the "frozen wheel" in [I]
    matrix_product(Rw,OMw,OMw_w);
    OMwf_w[1] = OMw_w[1];
    OMwf_w[2] = 0;
    OMwf_w[3] = OMw_w[3];
    matrix_product(Rtw,OMwf_w,OMwf);

    // vx1: Velocity of the geometrical contact point due to OMwf in [I]
    cross_product(OMwf,vct_I,vx1);

    // Vct_geo: Velocity of the geometrical contact point in [I]
    vector_sum(Vw,vx1,Vct_geo);

    // Vct_geo_s: Velocity of the geometrical contact point in [Rsol]
    matrix_product(Rsol,Vct_geo,Vct_geo_s);
    
    // Slideslip angle
    if ((Vct_geo_s[1]>=1e-3) | (Vct_geo_s[1]<=-1e-3))
    {
        tg_anglis = Vct_geo_s[2]/Vct_geo_s[1];
    }
    else
    {
        tg_anglis = 0;
    }

    anglis = atan(tg_anglis);
    *anglisp = anglis;

/* LONGITUDINAL SLIPING */
// Ration between the velocity of the contact point and the composant of the wheel velocity in the axis of the wheel.

    // Vcts[1]: Velocity of the contact point
    // Vws[1]: Velcity of the wheel center
    if ((Vws[1]>=1e-3) | (Vws[1]<=-1e-3))
    {
        gliss = -Vcts[1]/fabs(Vws[1]);
    }
    else
    {
        mbs_warning_msg("mbs_kine_wheel_motorbike, Wheel velocity is too small, imposing null longitudinal slip.\n");
        gliss = 0;
    }

    *glissp = gliss;
}

