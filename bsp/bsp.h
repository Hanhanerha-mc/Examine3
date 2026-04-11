#ifndef _BSP_H_
#define _BSP_H_

#define REGISTER(T) \
    T *T##_instance = (T*)malloc(sizeof(T)); \
    if memset(T_##instance, 0, sizeof(T))

#endif