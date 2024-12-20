#include <Wire.h>
#include <Math.h>

// Declare global variables and constants
float pomega = 10000; // variance between omega
float pb = 10; // variance in bias
float qb = 1; // variance
float qomega = 100000000000000;
float romega = 10000;
float rtheta = 10000;

float mu_b[3] = {0, 0, 0};
float mu_omega[72];
float mu_theta[72];
float mu_0[3] = {0, 0, 0};
float mu_prev[3] = {0, 0, 0};

const float T = 1;

float Ad[3][3] = {
  {1, 0, 0},
  {T, 1, 0},
  {0, 0, 1}
};

float Qd[3][3] = {
  {T * qomega, 0.5 * T * T * qomega, 0},
  {0.5 * T * T * qomega, (1.0 / 3) * T * T * T * qomega, 0},
  {0, 0, T * qb}
};

float P[3][3] = {
  {T * pomega, 0.5 * T * T * pomega, 0},
  {0.5 * T * T * pomega, (1.0 / 3) * T * T * T * pomega, 0},
  {0, 0, T * pb}
};

float P_prev[3][3];

float R[2][2] = {
  {rtheta, 0},
  {0, romega}
};

float Rd[2][2];
float Cd[2][3] = {
  {0, 1, 0},
  {1, 0, 1}
};

float I[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

float mus[3][73]; // Store predicted values
float ys[2][73]; // Store actual values

// Helper function to print matrix
void printMatrix(float matrix[][3], int rows, int cols) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      Serial.print(matrix[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(9600);
  delay(2000); // wait for the serial monitor to open
  
  // Initialize matrices
  for (int i = 0; i < 3; i++) {
    mu_prev[i] = mu_0[i];
    for (int j = 0; j < 3; j++) {
      P_prev[i][j] = P[i][j];
    }
  }
  Rd[0][0] = R[0][0] / T;
  Rd[1][1] = R[1][1] / T;

  // Load your data here, this example uses dummy data
  // You should replace this with actual data reading code
  for (int i = 0; i < 72; i++) {
    mu_omega[i] = 0; // Replace with actual gyro data
    mu_theta[i] = 0; // Replace with actual accel data
  }

  // Main Kalman Filter loop
  for (int i = 0; i < 72; i++) {
    float omegat = mu_omega[i] * (PI / 180);
    float yomega = omegat;
    float ar = mu_theta[i]; // Replace with actual data
    float at = mu_theta[i + 1]; // Replace with actual data
    float ytheta = atan2(-ar, at);
    float yk[2] = {omegat, ytheta};

    // Prediction step
    float mu_pred[3] = {0, 0, 0};
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        mu_pred[row] += Ad[row][col] * mu_prev[col];
      }
    }

    // P_pred = Ad * P_prev * Ad' + Qd
    float P_pred[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        for (int k = 0; k < 3; k++) {
          P_pred[row][col] += Ad[row][k] * P_prev[k][col];
        }
        P_pred[row][col] += Qd[row][col];
      }
    }

    // Kk = P_pred * Cd' * inv(Cd * P_pred * Cd' + Rd)
    float Kk[3][2] = {{0, 0}, {0, 0}, {0, 0}};
    float CdT[3][2] = {{0, 0}, {1, 0}, {0, 1}}; // Transpose of Cd
    float temp[2][2] = {{0, 0}, {0, 0}};
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 2; col++) {
        for (int k = 0; k < 3; k++) {
          temp[row][col] += Cd[row][k] * P_pred[k][col];
        }
        temp[row][col] += Rd[row][col];
      }
    }

    // Compute the inverse of the 2x2 matrix temp
    float det = temp[0][0] * temp[1][1] - temp[0][1] * temp[1][0];
    float inv_temp[2][2] = {{temp[1][1] / det, -temp[0][1] / det},
                            {-temp[1][0] / det, temp[0][0] / det}};

    // Kk = P_pred * CdT * inv_temp
    float temp2[3][2] = {{0, 0}, {0, 0}, {0, 0}};
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 2; col++) {
        for (int k = 0; k < 3; k++) {
          temp2[row][col] += P_pred[row][k] * CdT[k][col];
        }
      }
    }

    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 2; col++) {
        for (int k = 0; k < 2; k++) {
          Kk[row][col] += temp2[row][k] * inv_temp[k][col];
        }
      }
    }

    // mu_k = mu_pred + Kk * (yk - Cd * mu_pred)
    float yk_minus_Cd_mu_pred[2] = {0, 0};
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 3; col++) {
        yk_minus_Cd_mu_pred[row] += Cd[row][col] * mu_pred[col];
      }
      yk_minus_Cd_mu_pred[row] = yk[row] - yk_minus_Cd_mu_pred[row];
    }

    float mu_k[3] = {0, 0, 0};
    for (int row = 0; row < 3; row++) {
      mu_k[row] = mu_pred[row];
      for (int col = 0; col < 2; col++) {
        mu_k[row] += Kk[row][col] * yk_minus_Cd_mu_pred[col];
      }
    }

    // Pk = (I - Kk * Cd) * P_pred
    float I_minus_Kk_Cd[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        I_minus_Kk_Cd[row][col] = I[row][col];
        for (int k = 0; k < 2; k++) {
          I_minus_Kk_Cd[row][col] -= Kk[row][k] * Cd[k][col];
        }
      }
    }

    float Pk[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        for (int k = 0; k < 3; k++) {
          Pk[row][col] += I_minus_Kk_Cd[row][k] * P_pred[k][col];
        }
      }
    }

    // Update previous state
    for (int j = 0; j < 3; j++) {
      mu_prev[j] = mu_pred[j];
      for (int k = 0; k < 3; k++) {
        P_prev[j][k] = P_pred[j][k];
      }
    }

    // Store results for printing
    for (int j = 0; j < 3; j++) {
      mus[j][i + 1] = mu_k[j];
    }
    ys[0][i + 1] = yomega;
    ys[1][i + 1] = ytheta;
  }

  // Print results
  Serial.println("Predicted Omegas, Measured Omegas, Predicted Thetas, Measured Thetas");
  for (int i = 0; i < 73; i++) {
    Serial.print(mus[1][i]);
    Serial.print(", ");
    Serial.print(ys[0][i]);
    Serial.print(", ");
    Serial.print(mus[0][i]);
    Serial.print(", ");
    Serial.println(ys[1][i]);
  }
}

void loop() {
  // Empty loop
}
