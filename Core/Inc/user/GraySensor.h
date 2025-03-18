#ifndef __GRAYSENSOR_H
#define __GRAYSENSOR_H



#ifdef __cplusplus
extern "C" {
#endif

void Gray_Get_TTL(void);
float Gray_control(void);
uint8_t CrossDetect(void);
float degrees_to_rad(float degrees);

#ifdef __cplusplus
}
#endif




#endif
