/*
 * Manyetometre.h
 *
 *  Created on: Sep 11, 2024
 *      Author: Furkan Gündoğdu
 */

#ifndef MANYETOMETRE_H_
#define MANYETOMETRE_H_

#include "BMM150/BMM150.h"
#include "BMP180/BMP180.h"

extern I2C_HandleTypeDef hi2c1;

#define I2C1_HANDLE   &hi2c1

//typedef struct
//{
//	BMM150_Typedef_t  magneto;
//	BMM150_Settings_t settings;
//	CircularBuffer_t collectedData;
//}manyetometre_t;
//
//
//uint8_t manyeto_baslat(manyetometre_t *manyetometre_st);
//bool manyeto_veri_topla(manyetometre_t *manyetometre_st);


typedef struct
{
    float x;
    float y;
    float z;
    float kalibreli_heading;
    float kalibresiz_heading;
} Manyetometre;

/* Manyetometre işlemleri */
void manyetometre_init(void);
void manyetometre_oku(Manyetometre *manyeto);
void manyetometre_cevrim(Manyetometre *manyeto);





#endif /* MANYETOMETRE_H_ */
