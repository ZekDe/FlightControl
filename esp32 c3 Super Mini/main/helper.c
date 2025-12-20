#include "helper.h"

void compensateImuOffset(float *accel_corrected, 
                         const float *accel_raw,
                         const float *gyro,
                         const float *gyro_dot,  // Açısal ivme (opsiyonel)
                         const ImuOffset_t *offset)
{
    // Merkezcil ivme: a_c = ω × (ω × r)
    // Teğetsel ivme:  a_t = α × r  (gyro_dot varsa)
    
    float wx = gyro[0], wy = gyro[1], wz = gyro[2];
    float rx = offset->x, ry = offset->y, rz = offset->z;
    
    // ω × r
    float wxr[3];
    wxr[0] = wy*rz - wz*ry;
    wxr[1] = wz*rx - wx*rz;
    wxr[2] = wx*ry - wy*rx;
    
    // ω × (ω × r) = merkezcil ivme
    float centripetal[3];
    centripetal[0] = wy*wxr[2] - wz*wxr[1];
    centripetal[1] = wz*wxr[0] - wx*wxr[2];
    centripetal[2] = wx*wxr[1] - wy*wxr[0];
    
    // Düzeltme (merkezcil ivmeyi çıkar)
    accel_corrected[0] = accel_raw[0] - centripetal[0];
    accel_corrected[1] = accel_raw[1] - centripetal[1];
    accel_corrected[2] = accel_raw[2] - centripetal[2];
}


