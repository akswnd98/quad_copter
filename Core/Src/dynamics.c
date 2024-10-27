/*
 * dynamics.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include "dynamics.h"

/*
 * dynamics.c
 *
 *  Created on: 2023. 3. 27.
 *      Author: akswnd98
 */

#include "dynamics.h"
#include "sensor.h"
#include "math.h"
#include "control_receive.h"

float g = 9.80665;

void mul_mat_vec_3d (float mat[3][3], float vec[3], float rst[3]) {
  for (int i = 0; i < 3; i++) {
    rst[i] = 0;
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      rst[i] += mat[i][j] * vec[j];
    }
  }
}

void mul_mat_vec_4d (float mat[4][4], float vec[4], float rst[4]) {
  for (int i = 0; i < 4; i++) {
    rst[i] = 0;
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      rst[i] += mat[i][j] * vec[j];
    }
  }
}

void dot_vec_3d (float vec1[3], float vec2[3], float rst[3]) {
  for (int i = 0; i < 3; i++) {
    rst[i] = vec1[i] * vec2[i];
  }
}

void dot_vec_4d (float vec1[4], float vec2[4], float rst[4]) {
  for (int i = 0; i < 4; i++) {
    rst[i] = vec1[i] * vec2[i];
  }
}

void add_vec_3d (float vec1[3], float vec2[3], float rst[3]) {
    for (int i = 0; i < 3; i++) {
        rst[i] = vec1[i] + vec2[i];
    }
}

void sub_vec_3d (float vec1[3], float vec2[3], float rst[3]) {
  for (int i = 0; i < 3; i++) {
    rst[i] = vec1[i] - vec2[i];
  }
}

void add_vec_4d (float vec1[4], float vec2[4], float rst[4]) {
  for (int i = 0; i < 4; i++) {
    rst[i] = vec1[i] + vec2[i];
  }
}

void sub_vec_4d (float vec1[4], float vec2[4], float rst[4]) {
  for (int i = 0; i < 4; i++) {
    rst[i] = vec1[i] - vec2[i];
  }
}

void prod_vec_3d (float scalar, float vec[3], float rst[3]) {
  for (int i = 0; i < 3; i++) {
    rst[i] = scalar * vec[i];
  }
}

void prod_vec_4d (float scalar, float vec[4], float rst[4]) {
  for (int i = 0; i < 4; i++) {
    rst[i] = scalar * vec[i];
  }
}

void sqrt_vec_4d (float vec[4], float rst[4]) {
  for (int i = 0; i < 4; i++) {
    rst[i] = sqrt(vec[i]);
  }
}

void get_R (float eta[3], float R[3][3]) {
  R[0][0] = cos(eta[2]) * cos(eta[1]);
  R[0][1] = sin(eta[0]) * sin(eta[1]) * cos(eta[2]) - sin(eta[2]) * cos(eta[0]);
  R[0][2] = sin(eta[0]) * sin(eta[2]) + sin(eta[1]) * cos(eta[0]) * cos(eta[2]);

  R[1][0] = sin(eta[2]) * cos(eta[1]);
  R[1][1] = sin(eta[0]) * sin(eta[2]) * sin(eta[1]) + cos(eta[0]) * cos(eta[2]);
  R[1][2] = -sin(eta[0]) * cos(eta[2]) + sin(eta[2]) * sin(eta[1]) * cos(eta[0]);

  R[2][0] = -sin(eta[1]);
  R[2][1] = sin(eta[0]) * cos(eta[1]);
  R[2][2] = cos(eta[0]) * cos(eta[1]);
}

void get_R_inv (float eta[3], float R_inv[3][3]) {
  R_inv[0][0] = cos(eta[2]) * cos(eta[1]);
  R_inv[0][1] = sin(eta[2]) * cos(eta[1]);
  R_inv[0][2] = -sin(eta[1]);

  R_inv[1][0] = sin(eta[0]) * sin(eta[1]) * cos(eta[2]) - sin(eta[2]) * cos(eta[0]);
  R_inv[1][1] = sin(eta[0]) * sin(eta[2]) * sin(eta[1]) + cos(eta[0]) * cos(eta[2]);
  R_inv[1][2] = sin(eta[0]) * cos(eta[1]);

  R_inv[2][0] = sin(eta[0]) * sin(eta[2]) + sin(eta[1]) * cos(eta[0]) * cos(eta[2]);
  R_inv[2][1] = -sin(eta[0]) * cos(eta[2]) + sin(eta[2]) * sin(eta[1]) * cos(eta[0]);
  R_inv[2][2] = cos(eta[0]) * cos(eta[1]);
}

void get_C (float phi, float theta, float C[3][3]) {
  C[0][0] = 1;
  C[0][1] = 0;
  C[0][2] = -sin(theta);
  C[1][0] = 0;
  C[1][1] = cos(phi);
  C[1][2] = sin(phi) * cos(theta);
  C[2][0] = 0;
  C[2][1] = -sin(phi);
  C[2][2] = cos(phi) * cos(theta);
}

void get_C_inv (float phi, float theta, float C_inv[3][3]) {
  C_inv[0][0] = 1;
  C_inv[0][1] = sin(phi) * tan(theta);
  C_inv[0][2] = cos(phi) * tan(theta);
  C_inv[1][0] = 0;
  C_inv[1][1] = cos(phi);
  C_inv[1][2] = -sin(phi);
  C_inv[2][0] = 0;
  C_inv[2][1] = sin(phi) / cos(theta);
  C_inv[2][2] = cos(phi) / cos(theta);
}

void get_C_dot (float phi, float theta, float eta_dot[3], float C_dot[3][3]) {
  C_dot[0][0] = 0;
  C_dot[0][1] = 0;
  C_dot[0][2] = -eta_dot[1] * cos(theta);
  C_dot[1][0] = 0;
  C_dot[1][1] = -eta_dot[0] * sin(phi);
  C_dot[1][2] = eta_dot[0] * cos(phi) * cos(theta) - eta_dot[1] * sin(phi) * sin(theta);
  C_dot[2][0] = 0;
  C_dot[2][1] = -eta_dot[0] * cos(phi);
  C_dot[2][2] = -eta_dot[0] * sin(phi) * cos(theta) - eta_dot[1] * sin(theta) * cos(phi);
}

float get_error (float ref, float cur_val) {
  return ref - cur_val;
}

float get_dot_ref_by_pd (float error, float P, float dot_val, float D) {
  return error * P - dot_val * D;
}

void get_eta_dot_dot_ref (float phi_dot_dot_ref, float theta_dot_dot_ref, float psi_dot_dot_ref, float eta_dot_dot_ref[3]) {
  eta_dot_dot_ref[0] = phi_dot_dot_ref;
  eta_dot_dot_ref[1] = theta_dot_dot_ref;
  eta_dot_dot_ref[2] = psi_dot_dot_ref;
}

float get_z_dot_error (float z_dot_ref_signal, float z_dot_signal) {
  return z_dot_ref_signal - z_dot_signal;
}

float get_z_dot_dot_ref (float z_dot_signal_error, float P) {
  return z_dot_signal_error * P;
}

void get_force_vector (float eta_dot_dot_ref[3], float T_inv[4][4], float I[3][3], float C[3][3], float rst[4]) {
  float a[3];

  mul_mat_vec_3d(C, eta_dot_dot_ref, a);
  float b[4] = {a[0], a[1], a[2], command[2] * 4};
  mul_mat_vec_4d(T_inv, b, rst);
  /* for (int i = 0; i < 4; i++) {
    rst[i] += command[2] * 4;
  } */
}

void get_esc_vector (float force_vector[4], float rst[4]) {
  for (int i = 0; i < 4; i++) {
    if (force_vector[i] < 0) {
      rst[i] = 0;
    } else {
      rst[i] = force_vector[i];
    }
  }
}

float I[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

float T_inv[4][4] = {
  {-1, -1, -1, 1},
  {-1, 1, 1, 1},
  {1, 1, -1, 1},
  {1, -1, 1, 1}
};

float P_phi = 160;
float D_phi = 20;
float P_phi_dot = 10;
float D_phi_dot = 4;
float P_theta = 160;
float D_theta = 20;
float P_theta_dot = 10;
float D_theta_dot = 4;
float P_psi_dot = 300;
float D_psi_dot = 0;

float phi = 0;
float phi_dot = 0;
float phi_dot_dot = 0;
float theta = 0;
float theta_dot = 0;
float theta_dot_dot = 0;
float psi_dot = 0;
float psi_dot_dot = 0;

float dt = 0.001;
float H = 0.004;
float K = 1;
float dot_dot_K = 1;
void update_euler_state () {
  float new_phi = 0;
  float new_theta = 0;
  float new_phi_dot = 0;
  float new_theta_dot = 0;
  float new_psi_dot = 0;

  float phi_low_pass = atan(acc_y_sensor / acc_z_sensor);
  float theta_low_pass = atan(-acc_x_sensor / sqrt(acc_y_sensor * acc_y_sensor + acc_z_sensor * acc_z_sensor));
  /* float phi_low_pass = acc_y_sensor / acc_z_sensor;
  float theta_low_pass = -acc_x_sensor / acc_z_sensor; */

  float phi_high_pass = phi + dt * phi_dot;
  float theta_high_pass = theta + dt * theta_dot;

  new_phi = phi_low_pass * H + phi_high_pass * (1.0 - H);
  new_theta = theta_low_pass * H + theta_high_pass * (1.0 - H);

  float cur_C_inv[3][3];
  float cur_w[3] = {gyro_x_sensor, gyro_y_sensor, gyro_z_sensor};
  get_C_inv(new_phi, new_theta, cur_C_inv);
  float cur_eta_dot[3];
  mul_mat_vec_3d(cur_C_inv, cur_w, cur_eta_dot);
  // float cur_eta_dot[3] = {gyro_x_sensor, gyro_y_sensor, gyro_z_sensor};

  new_phi_dot = phi_dot * (1.0 - K) + cur_eta_dot[0] * K;
  new_theta_dot = theta_dot * (1.0 - K) + cur_eta_dot[1] * K;
  new_psi_dot = psi_dot * (1.0 - K) + cur_eta_dot[2] * K;

  float raw_phi_dot_dot = (new_phi_dot - phi_dot) / dt;
  float new_phi_dot_dot = phi_dot_dot * (1.0 - dot_dot_K) + raw_phi_dot_dot * dot_dot_K;
  float raw_theta_dot_dot = (new_theta_dot - theta_dot) / dt;
  float new_theta_dot_dot = theta_dot_dot * (1.0 - dot_dot_K) + raw_theta_dot_dot * dot_dot_K;
  float raw_psi_dot_dot = (new_psi_dot - psi_dot) / dt;
  float new_psi_dot_dot = psi_dot_dot * (1.0 - dot_dot_K) + raw_psi_dot_dot * dot_dot_K;

  phi = new_phi;
  phi_dot = new_phi_dot;
  phi_dot_dot = new_phi_dot_dot;
  theta = new_theta;
  theta_dot = new_theta_dot;
  theta_dot_dot = new_theta_dot_dot;
  psi_dot = new_psi_dot;
  psi_dot_dot = new_psi_dot_dot;
}
