#ifndef HELPER_H
#define HELPER_H

typedef struct {
    float x;  // İleri (+) / Geri (-)
    float y;  // Sağ (+) / Sol (-)
    float z;  // Aşağı (+) / Yukarı (-)
} ImuOffset_t;

// Örnek: IMU merkezden 3cm arkada, 2cm sağda
// ImuOffset_t imu_offset = {
//     .x = -0.03f,  // 3cm arkada
//     .y =  0.02f,  // 2cm sağda  
//     .z =  0.0f    // Aynı yükseklikte
// };

#endif  // HELPER_H
