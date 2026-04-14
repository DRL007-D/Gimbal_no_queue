
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported types ------------------------------------------------------------*/
typedef struct 
{
    float Last_P;
    float Now_P;
    float OUT;
    float Kg;
    float Q;
    float R;
}Kalman_Filter_t;

float Kalman_Filter(Kalman_Filter_t *ekf, float input);
/* Exported constants --------------------------------------------------------*/
#endif 

