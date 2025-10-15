#include "calibration.h"
#include "foc_cfg.h"
#include "foc_ctrl.h"
#include "util.h"

uint8_t cali_flag = 0;

float end_dir_pos;
float start_pos, end_pos;
float raw_diff;

uint8_t pn;
int8_t dir;
double e_theta;
uint32_t i, j, k, l;

double theta_ref = 0.0;
float now_theta, lst_theta;
float error_angle;

uint32_t n  = 256 * 7;
uint32_t n2 = 40;
double delta;

float p_error_array[7 * 2 * 256];
float error[256];


float E_OFFSET, _PS_OFFSET;
float mean = 0.0f, offset = 0.0f;

int ind;
int raw_offset;
uint32_t p_raw[2];

void calibrate_mt_encoder(float vd_set, float pos) {
    int n_lut = 256;

    int N      = pn * 256;// npp*256
    int window = 256 / 2;

    delta = M_2PI * pn / (n * n2);

    switch (cali_flag) {
        case 0:
            if (++i < 20000)// align rotor
            {
                FocVolt(vd_set, 0.0f, 0.0f);
                start_pos = enc_para.pos;
            } else {
                i         = 0;
                start_pos = enc_para.pos;
                cali_flag = 1;
            }
            break;
        case 1:
            e_theta += 0.003f;

            FocVolt(vd_set, 0.f, e_theta);

            if (e_theta > 0.5f * M_PI) {
                if (e_theta < 0.55f * M_PI) end_dir_pos = enc_para.pos;

                if (fabs(start_pos - enc_para.pos) < 0.001f) {// 比较位置
                    cali_flag = 2;
                }
            }

            break;
        case 2: {
            pn = e_theta / M_2PI;

            raw_diff = end_dir_pos - start_pos;
            if (raw_diff > M_PI) {
                raw_diff -= M_2PI;
            } else if (raw_diff < -M_PI) {
                raw_diff += M_2PI;
            }

            if (raw_diff > 0) dir = 1;
            else
                dir = -1;

            foc_param.v_d = 2.5f;
            foc_param.v_q = 0;
            cali_flag     = 3;
            break;
        }
        case 3:
            if (++i < 20000)// align rotor
            {
                FocVolt(vd_set, 0.f, 0.0f);
            } else {
                i         = 0;
                j         = 40;
                cali_flag = 4;
            }
            break;
        case 4: {
            if (i < (n * n2)) {
                if (j == 40) {
                    j = 0;

                    now_theta   = enc_para.pos;
                    error_angle = theta_ref / (pn) -now_theta;// rad
                    WRAP_PM_PI(error_angle);

                    if (k == 0) p_raw[0] = enc_para.raw_data;

                    p_error_array[k] = error_angle;

                    k++;
                }
                theta_ref += delta;// theta_ref∈[0,2π]*POLE
                FocVolt(vd_set, 0.f, theta_ref);
                i++;
                j++;
            } else {
                i         = 0;
                j         = 40;// k=0;
                cali_flag = 5;
            }
            break;
        }
        case 5: {
            if (i < (n * n2)) {
                if (j == 40) {
                    j           = 0;
                    now_theta   = enc_para.pos;
                    error_angle = theta_ref / (pn) -now_theta;// rad
                    WRAP_PM_PI(error_angle);

                    if (k == 2 * pn * 256 - 1) p_raw[1] = enc_para.raw_data;
                    p_error_array[k] = error_angle;
                    k++;
                }
                theta_ref -= delta;// theta_ref∈[0,2π]*POLE
                FocVolt(vd_set, 0.f, theta_ref);
                i++;
                j++;
            } else {
                i         = 0;
                j         = 0;
                cali_flag = 6;
                FocVolt(0.0f, 0.0f, 0.0f);
            }
            break;
        }
        case 6: {
            // TODO_1: The reference value of the first index here is not 0, but
            // the value of the last index is 0, which needs to be corrected.
            for (i = 0; i < N; i++) {
                p_error_array[i]
                        = 0.5f
                          * (p_error_array[i] + p_error_array[2 * N - i - 1]);
                offset = offset + p_error_array[i] / N;
            }
            E_OFFSET   = fmodf(offset * pn, M_2PI);
            _PS_OFFSET = fmodf(offset, M_2PI);
            WRAP_PM_PI(E_OFFSET);
            cali_flag = 7;
            break;
        }
        case 7: {
            // TODO_3: Whether to use filtering is to be tested.
            for (i = 0; i < N; i++) {
                float temp = 0.0f;
                for (int j = 0; j < window; j++) {
                    ind = -window / 2 + j
                          + i;// Indexes from -window/2 to + window/2
                    if (ind < 0) {
                        ind += N;
                    }// Moving average wraps around
                    else if (ind > (N - 1)) {
                        ind -= N;
                    }
                    //							error_filt[i] +=
                    // p_error_array[ind] / (float)window;
                    temp += p_error_array[ind] / (float) window;
                }
                //					mean += error_filt[i] / N;
                if (i % pn == 0) {
                    error[i / pn] = temp;// 转存到error数组中
                }

                mean += temp / N;
            }
            i         = 0;
            cali_flag = 8;
            break;
        }
        case 8: {
            // TODO_4: Rebuild lut using error_filt
            raw_offset = (p_raw[0] + p_raw[1])
                         * 0.5f;// Insensitive to errors in this direction, so 2
                                // points is plenty
            for (int i = 0; i < n_lut; i++) {// build lookup table
                ind = (raw_offset >> 6) + i;
                if (ind > (n_lut - 1)) ind -= n_lut;
                // lut[ind] = (error[i] - mean) * 2607.5945876176f;  //
                // 16384/two_pi printf("%f\r\n", lut[ind]);
            }
            //			printf("\r\n");
            cali_flag = 9;
            break;
        }
        case 9:
            for (int i = 0; i < n_lut; i++) {// build lookup table
                // printf("%f,\r\n", lut[i]);
            }
            cali_flag = 10;
            break;
    }
}
