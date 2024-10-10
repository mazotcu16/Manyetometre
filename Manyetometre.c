/*
 * Manyetometre.c
 *
 *  Created on: Sep 11, 2024
 *      Author: Furkan Gündoğdu
 */
#include "Manyetometre.h"
//Manyetometre manyeto;

float soft_iron_matrix[3][3] =
	{
        {3.6040206, 0.179675058, -0.279114366 },
        {0.179675058, 3.56232357, -0.279114366},
        {-0.279114366, 0.116861321, 2.06771994}
    };

float bias[3] = {-3.42443657,-0.515112698,-3.15804315};



/* Global manyetometre yapısı */
static BMM150_Typedef_t bmm150_dev;
static CircularBuffer_t circularBuffer;

void manyetometre_init(void)
{
    /* Manyetometre sensörünü başlat */
    BMM150_Settings_t settings;
    BMM150_Initialise(&bmm150_dev,I2C1_HANDLE, &settings);
    CircularBuffer_Init(&circularBuffer);
}

void manyetometre_oku(Manyetometre *manyeto)
{
    /* Manyetometre'den verileri oku */
    magneto_collect_gauss(&circularBuffer, &bmm150_dev);
    int16_t last_index = (circularBuffer.head == 0) ? SAMPLE_COUNT - 1 : circularBuffer.head - 1;
    /* Verileri CircularBuffer'dan çek */
    manyeto->x = (circularBuffer.data[last_index].x);		//Olcek carpanlari kaldirildi
    manyeto->y = (circularBuffer.data[last_index].y);
    manyeto->z = (circularBuffer.data[last_index].z);
}

void manyetometre_cevrim(Manyetometre *manyeto)
{
    /* Manyetometre verileri, kalibrasyonu uygula ve heading açısı */

    manyetometre_oku(manyeto);

    /* Kalibrasyon ve heading hesaplama */
//    int16_t raw_measurement[3] = { manyeto.x, manyeto.y, manyeto.z };
    float raw_measurement2[3] = { (manyeto->x), (manyeto->y), (manyeto->z) };
    float corrected_measurement[3];

    ApplyMagCalibration(raw_measurement2, bias, soft_iron_matrix, corrected_measurement);

    /* Heading hesaplama */
    float heading = CalculateHeading(corrected_measurement[0], corrected_measurement[1], DECLINATION_DEG);	//Dinamik olarak hesaplanabilir.lookup table yapılabilir.
    float heading2 = CalculateHeading(manyeto->x, manyeto->y, DECLINATION_DEG);

//    uint16_t heading_kalibresiz_olcekli = heading2 * 100;
//    uint16_t heading_kalibreli_olcekli  =  heading * 100;

    manyeto->kalibresiz_heading  = heading2;
    manyeto->kalibreli_heading = heading;
}



