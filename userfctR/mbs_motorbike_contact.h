#ifndef MBS_MOTORBIKE_CONTACT_h
#define MBS_MOTORBIKE_CONTACT_h

#include "mbs_data.h"

/*!
 * Structure containing all parameters for tire-ground contact.
 * This structure is tire-specific and dedicated to motorbike tires.
 */
typedef struct TIRE_param_strct
{
    // Longitudinal parameter for the tire

    double C_x;

    double p_Dx1;
    double p_Dx2;

    double p_Ex1;
    double p_Ex2;
    double p_Ex3;
    double p_Ex4;

    double p_Kx1;
    double p_Kx2;
    double p_Kx3;

    double F_z0;

    // Lateral parameter for the tire

    double C_y;

    double p_Dy1;
    double p_Dy2;
    double p_Dy3;

    double p_Ey1;
    double p_Ey2;
    double p_Ey4;

    double p_Ky1;
    double p_Ky2;
    double p_Ky3;
    double p_Ky4;
    double p_Ky5;
    double p_Ky6;
    double p_Ky7;

    double C_gamma;
    double E_gamma;

    // Aligning parameter for the tire

    double C_t;

    double q_Bz1;
    double q_Bz2;
    double q_Bz5;
    double q_Bz6;
    double q_Bz9;
    double q_Bz10;

    double q_Dz1;
    double q_Dz2;
    double q_Dz3;
    double q_Dz4;
    double q_Dz8;
    double q_Dz9;
    double q_Dz10;
    double q_Dz11;

    double q_Ez1;
    double q_Ez2;
    double q_Ez5;

    double q_Hz3;
    double q_Hz4;

    double R_0;

    // Combined slip parameter

    double r_Bx1;
    double r_Bx2;
    double C_xalpha;

    double r_By1;
    double r_By2;
    double r_By3;
    double C_ykappa;

} TIRE_param_strct;

/*!
 * \brief Compute the forces and torques based on the contact kinematics
 *
 * \param[in, out] F Forces (in the ground contact frame) applied at the contact. The vertical component (F[4]) has to be provided
 * \param[out] M Torques (in the ground contact frame) applied at the contact. The model only computes the torque around the vertical axis (M[4]), other torques are considered null.
 * \param[in] beta Slip angle
 * \param[in] gamma Camber angle
 * \param[in] kappa longitudinal slip
 * \param[in] Vct_geo_s, geometrical contact point velocity in the Rsol frame
 * \param[in] tire_param_strct Structure containing the contact parameters of the tire
 * \return 1 (one) if the process succeed
 */
int mbs_motorbike_contact(double F[4], double M[4], double beta, double gamma, double kappa, double Vct_geo_s[4], TIRE_param_strct* tire_param_strct);

/*!
 * \brief Compute the kinematics (local frames, slip, angles...) of the wheel-ground contact
 *
 * Assumptions:
 *  1. Ground is (locally) flat
 *  2. Ground is (locally) horizontal
 *  3. The wheel camber angle is lower than pi/4 (angcambp)
 *  4. The wheel longitudinal velocity is greather that 1e-3 or lower than -1e-3
 *
 * \param[in] Pw Position of the wheel center in the inertial frame
 * \param[in] Rw Rotation matrix between inertial and wheel frame
 * \param[in] Vw Velocity of the wheel center in the inertial frame
 * \param[in] OMw Angular velocity of the wheel center in the inertial frame
 * \param[in] tsim current simulation time
 * \param[in] iwhl id of the force sensor related to this wheel
 * \param[in] r_rim radius of the wheel (without the tire)
 * \param[in] r_t_tire thickness (in radius) of the tire.
 * \param[out] penp pointer to the vertical tire-ground penetration
 * \param[out] rzp pointer to the effective wheel radius in the wheel plane
 * \param[out] anglisp pointer to the slip angle (beta)
 * \param[out] angcambp pointer to the camber angle (gamma)
 * \param[out] glissp pointer to the longitudinal slipping (kappa), set at 0 if hypothesis 4 is not meeted
 * \param[out] Vct velocity of the material contact point in the inertial frame
 * \param[out] Vct_geo_s, geometrical contact point velocity in the Rsol frame
 * \param[out] Rtsol Rotation matrix between Inertial and contact frame
 * \param[out] dxF vector between the wheel center and the force application point expressed in the wheel frame
 */
void mbs_kine_wheel_motorbike(double Pw[4],double Rw[4][4],
                       double Vw[4],double OMw[4],
                       double tsim,int iwhl, double r_rim, double r_t_tire,
                       double *penp, double *rzp, double *anglisp, double *angcambp,
                       double *glissp, double Vct[4], double Vct_geo_s[4], double Rtsol[4][4], double dxF[4]);

/*!
 * \brief Create and initialize a TIRE_param_strct for a 120/70 motorbike tire
 *
 * The width of the tire is 120 mm.
 * The aspect ration of the tire is 70%.
 *
 * \return a pointer to the fully filled TIRE_param_strct
 */
TIRE_param_strct* init_TIRE_param_strct_120_70();

/*!
 * \brief Create and initialize a TIRE_param_strct for a 140/70 motorbike tire
 *
 * The width of the tire is 140 mm.
 * The aspect ration of the tire is 70%.
 * The parameters are not confirmed (BETA)
 *
 * \return a pointer to the fully filled TIRE_param_strct
 */
TIRE_param_strct* init_TIRE_param_strct_140_70_beta();

/*!
 * \brief Create and initialize a TIRE_param_strct for a 160/70 motorbike tire
 *
 * The width of the tire is 160 mm.
 * The aspect ration of the tire is 70%.
 *
 * \return a pointer to the fully filled TIRE_param_strct
 */
TIRE_param_strct* init_TIRE_param_strct_160_70();

/*!
 * \brief Release the momory allocated for a TIRE_param_strct
 */
void free_TIRE_param_strct(TIRE_param_strct* tire_param_strct);

#endif
