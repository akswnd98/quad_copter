/*
 * dynamics.h
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#ifndef INC_DYNAMICS_H_
#define INC_DYNAMICS_H_

extern float g;

void mul_mat_vec_3d (float mat[3][3], float vec[3], float rst[3]);
void mul_mat_vec_4d (float mat[4][4], float vec[4], float rst[4]);
void dot_vec_3d (float vec1[3], float vec2[3], float rst[3]);
void dot_vec_4d (float vec1[4], float vec2[4], float rst[4]);
void add_vec_3d (float vec1[3], float vec2[3], float rst[3]);
void sub_vec_3d (float vec1[3], float vec2[3], float rst[3]);
void add_vec_4d (float vec1[4], float vec2[4], float rst[4]);
void sub_vec_4d (float vec1[4], float vec2[4], float rst[4]);
void prod_vec_3d (float scalar, float vec[3], float rst[3]);
void prod_vec_4d (float scalar, float vec[4], float rst[4]);
void sqrt_vec_4d (float vec[4], float rst[4]);
void get_R (float eta[3], float R[3][3]);
void get_R_inv (float eta[3], float R_inv[3][3]);
void get_C (float phi, float theta, float C[3][3]);
void get_C_inv (float phi, float theta, float C_inv[3][3]);
void get_C_dot (float phi, float theta, float eta_dot[3], float C_dot[3][3]);
float get_error (float ref, float cur_val);
float get_dot_ref_by_pd (float error, float P, float dot_val, float D);
void get_eta_dot_dot_ref (float phi_dot_dot_ref, float theta_dot_dot_ref, float psi_dot_dot_ref, float eta_dot_dot_ref[3]);
float get_z_dot_error (float z_dot_ref_signal, float z_dot_signal);
float get_z_dot_dot_ref (float z_dot_signal_error, float P);
void get_force_vector (float eta_dot_dot_ref[3], float T_inv[4][4], float I[3][3], float C[3][3], float rst[4]);
void get_esc_vector (float force_vector[4], float rst[4]);

extern float I[3][3];
extern float T_inv[4][4];

extern float P_phi;
extern float D_phi;
extern float P_phi_dot;
extern float D_phi_dot;
extern float P_theta;
extern float D_theta;
extern float P_theta_dot;
extern float D_theta_dot;
extern float P_psi_dot;
extern float D_psi_dot;

extern float phi;
extern float phi_dot;
extern float phi_dot_dot;
extern float theta;
extern float theta_dot;
extern float theta_dot_dot;
extern float psi_dot;
extern float psi_dot_dot;

extern float dt;
extern float H;
extern float K;
void update_euler_state ();

#endif /* INC_DYNAMICS_H_ */
