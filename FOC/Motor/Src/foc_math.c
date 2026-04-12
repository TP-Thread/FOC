/*!
    \file    foc_math.c
    \brief   foc basic mathematical algorithm source file

    \version 2025-06-30, V1.0.0, GDFOC2.0
*/

#include "foc_math.h"

static const float sin_table[SIN_TABLE_SIZE] = {
   -0.0122715f, 0.0000000f, 0.0122715f, 0.0245412f, 0.0368072f, 0.0490676f,
    0.0613207f, 0.0735645f, 0.0857973f, 0.0980171f, 0.1102222f, 0.1224106f,
    0.1345807f, 0.1467304f, 0.1588581f, 0.1709618f, 0.1830398f, 0.1950903f,
    0.2071113f, 0.2191012f, 0.2310581f, 0.2429801f, 0.2548656f, 0.2667127f,
    0.2785196f, 0.2902846f, 0.3020059f, 0.3136817f, 0.3253102f, 0.3368898f,
    0.3484186f, 0.3598950f, 0.3713171f, 0.3826834f, 0.3939920f, 0.4052413f,
    0.4164295f, 0.4275550f, 0.4386162f, 0.4496113f, 0.4605387f, 0.4713967f,
    0.4821837f, 0.4928981f, 0.5035383f, 0.5141027f, 0.5245896f, 0.5349976f,
    0.5453249f, 0.5555702f, 0.5657318f, 0.5758081f, 0.5857978f, 0.5956993f,
    0.6055110f, 0.6152315f, 0.6248594f, 0.6343932f, 0.6438315f, 0.6531728f,
    0.6624157f, 0.6715589f, 0.6806010f, 0.6895405f, 0.6983762f, 0.7071067f,
    0.7157308f, 0.7242470f, 0.7326542f, 0.7409511f, 0.7491363f, 0.7572088f,
    0.7651672f, 0.7730104f, 0.7807372f, 0.7883464f, 0.7958369f, 0.8032075f,
    0.8104572f, 0.8175848f, 0.8245893f, 0.8314696f, 0.8382247f, 0.8448535f,
    0.8513551f, 0.8577286f, 0.8639728f, 0.8700869f, 0.8760700f, 0.8819212f,
    0.8876396f, 0.8932243f, 0.8986744f, 0.9039892f, 0.9091679f, 0.9142097f,
    0.9191138f, 0.9238795f, 0.9285060f, 0.9329928f, 0.9373390f, 0.9415440f,
    0.9456073f, 0.9495281f, 0.9533060f, 0.9569403f, 0.9604305f, 0.9637760f,
    0.9669764f, 0.9700312f, 0.9729399f, 0.9757021f, 0.9783173f, 0.9807852f,
    0.9831054f, 0.9852776f, 0.9873014f, 0.9891765f, 0.9909026f, 0.9924795f,
    0.9939069f, 0.9951847f, 0.9963126f, 0.9972904f, 0.9981181f, 0.9987954f,
    0.9993223f, 0.9996988f, 0.9999247f, 1.0000000f, 0.9999247f,
   };

static const float atan_table[ATAN_TABLE_SIZE] = {
    0.0000000f, 0.0099996f, 0.0199973f, 0.0299910f, 0.0399786f, 0.0499583f,
    0.0599281f, 0.0698860f, 0.0798299f, 0.0897581f, 0.0996686f, 0.1095595f,
    0.1194289f, 0.1292750f, 0.1390959f, 0.1488899f, 0.1586552f, 0.1683901f,
    0.1780929f, 0.1877619f, 0.1973955f, 0.2069921f, 0.2165503f, 0.2260683f,
    0.2355449f, 0.2449786f, 0.2543680f, 0.2637118f, 0.2730087f, 0.2822574f,
    0.2914567f, 0.3006056f, 0.3097029f, 0.3187475f, 0.3277385f, 0.3366748f,
    0.3455555f, 0.3543799f, 0.3631470f, 0.3718560f, 0.3805063f, 0.3890972f,
    0.3976279f, 0.4060980f, 0.4145068f, 0.4228539f, 0.4311387f, 0.4393608f,
    0.4475199f, 0.4556156f, 0.4636476f, 0.4716155f, 0.4795192f, 0.4873585f,
    0.4951332f, 0.5028432f, 0.5104883f, 0.5180685f, 0.5255837f, 0.5330341f,
    0.5404195f, 0.5477400f, 0.5549957f, 0.5621867f, 0.5693131f, 0.5763752f,
    0.5833730f, 0.5903067f, 0.5971766f, 0.6039829f, 0.6107259f, 0.6174058f,
    0.6240230f, 0.6305777f, 0.6370703f, 0.6435011f, 0.6498704f, 0.6561787f,
    0.6624262f, 0.6686135f, 0.6747409f, 0.6808088f, 0.6868176f, 0.6927678f,
    0.6986598f, 0.7044940f, 0.7102710f, 0.7159911f, 0.7216548f, 0.7272626f,
    0.7328151f, 0.7383125f, 0.7437555f, 0.7491446f, 0.7544801f, 0.7597627f,
    0.7649928f, 0.7701709f, 0.7752974f, 0.7803730f, 0.7853981f, 0.7903732f,
};

/*!
    \brief      store sample value to the specified array
    \param[in]  *buffer: the pointer of the destination array
    \param[in]  sample: the value which will be stored in the array
    \param[in]  length_bit: the length of the array, expressed in the length_bit power of 2
    \param[in]  *index: the index of the array
    \param[out] none
    \retval     none
*/
void utils_add_sample(int16_t *buffer, int16_t sample, uint8_t length_bit, uint8_t *index)
{
    uint8_t cnt_mask = 0xFF >> (8 - length_bit);
    buffer[*index] = sample;
    *index += 1;
    *index &= cnt_mask;
}

/*!
    \brief      get the average value of all the data in the array
    \param[in]  *buffer: the pointer of the destination array
    \param[in]  length_bit: the length of the array, expressed in the length_bit power of 2
    \param[out] none
    \retval     the average value
*/
int16_t utils_average_sample(int16_t *buffer, uint8_t length_bit)
{
    uint8_t i, max = (uint8_t)(1 << length_bit);
    int32_t sum = 0;
    for(i=0; i<max; i++){
        sum += buffer[i];
    }
    sum /= max;
    
    return (int16_t)(sum);
}

/*!
    \brief      initialize pid structure
    \param[in]  pid_para: pointer to pid_parameter structure
    \param[out] none
    \retval     none
*/
void pid_init(pid_t *pid_para)
{
    pid_para->error = 0;
    pid_para->integral_sum = 0;
    pid_para->output = 0;
}

/*!
    \brief      pid control algorithm
    \param[in]  ref: the reference of the regulator
    \param[in]  now: the feedback of the regulator
    \param[in]  pid_para: pointer to pid_parameter structure
    \param[out] none
    \retval     the output of the regulator
*/
float pid_regulation(float ref, float back, pid_t *pid_para)
{
    float output_temp = 0;

    /* calculate the error */
    pid_para->error = (ref - back);

    pid_para->integral_sum += pid_para->error * pid_para->ki;

    /* integration anti-windup */
    if(pid_para->integral_sum > pid_para->integral_sum_upper_limit){
        pid_para->integral_sum = pid_para->integral_sum_upper_limit;
    }else if(pid_para->integral_sum < pid_para->integral_sum_lower_limit){
        pid_para->integral_sum = pid_para->integral_sum_lower_limit;
    }

    output_temp = pid_para->error * pid_para->kp  + pid_para->integral_sum;

    /* output anti-windup */
    if(output_temp < pid_para->output_lower_limit){
        output_temp = pid_para->output_lower_limit;
    }else if(output_temp > pid_para->output_upper_limit){
        output_temp = pid_para->output_upper_limit;
    }
    
    pid_para->output = output_temp;

    return (pid_para->output);
}

/*!
    \brief      equal amplitude clarke transformation
    \param[out] alpha: current alpha of alpha_beta-axis
    \param[out] beta: current beta of alpha_beta-axis
    \param[in]  a: current a of a-axis
    \param[in]  b: current b of b-axis
    \retval     none
*/
void clarke_amplitude(float *alpha, float *beta, float a, float b)
{
    *alpha = a;
    *beta = ONE_DIVIDE_SQRT3 * a + TWO_DIVIDE_SQRT3 * b;
}

/*!
    \brief      equal amplitude clarke transformation
    \param[out] alpha: current alpha of alpha_beta-axis
    \param[out] beta: current beta of alpha_beta-axis
    \param[in]  a: current a of a-axis
    \param[in]  b: current b of b-axis
    \retval     none
*/
void clarke_power(float *alpha, float *beta, float a, float b)
{
    *alpha = SQRT6_DIVIDE_2*a;
    *beta = SQRT2_DIVIDE_2 * a + SQRT2 * b;
}

/*!
    \brief      reverse equal amplitude clarke transformation
    \param[out] a: current a of abc-axis
    \param[out] b: current b of abc-axis
    \param[in]  alpha: current alpha of alpha_beta-axis
    \param[in]  beta: current beta of alpha_beta-axis
    \retval     none
*/
void rev_clarke_amplitude(float *a, float *b, float alpha, float beta)
{
    *a = alpha;
    *b = -0.5f * alpha + SQRT3_DIVIDE_2 * beta;
}

/*!
    \brief      reverse constant power clarke transformation
    \param[out] a: current a of abc-axis
    \param[out] b: current b of abc-axis
    \param[in]  alpha: current alpha of alpha_beta-axis
    \param[in]  beta: current beta of alpha_beta-axis
    \retval     none
*/
void rev_clarke_power(float *a, float *b, float alpha, float beta)
{
    *a = SQRT6_DIVIDE_3 * alpha;
    *b = -SQRT6_DIVIDE_6 * alpha + SQRT2_DIVIDE_2 * beta;
}

/*!
    \brief      park transformation
    \param[out] q: current q of q-axis
    \param[out] d: current d of d-axis
    \param[in]  alpha: current alpha of alpha_beta-axis
    \param[in]  beta: current beta of alpha_beta-axis
    \retval     none
*/
void park(float *d, float *q, float alpha, float beta, triangle_struct theta)
{
    *d = (theta.cos * alpha + theta.sin * beta);
    *q = (-theta.sin * alpha + theta.cos * beta);
}

/*!
    \brief      reverse park transformation
    \param[out] alpha: current alpha of alpha_beta-axis
    \param[out] beta: current beta of alpha_beta-axis
    \param[in]  q: current q of qd-axis
    \param[in]  d: current d of qd-axis
    \retval     none
*/
void rev_park(float *alpha, float *beta, float d, float q,triangle_struct theta)
{
    *alpha = -theta.sin * q + theta.cos * d ;
    *beta  = theta.cos * q + theta.sin * d;
}

/*!
    \brief      get the linear mapping fixed value from one space to another
    \param[in]  input_data: the input data
    \param[in]  in_min: the minimum value of the primary space
    \param[in]  in_max: the maximum value of the primary space
    \param[in]  out_min: the minimum value of the secondary space
    \param[in]  out_max: the maximum value of the secondary space
    \param[out] none
    \retval     the mapping value
*/
int16_t map_fixed(int16_t input_data, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
    return (input_data - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*!
    \brief      get the linear mapping float value from one space to another
    \param[in]  input_data: the input data
    \param[in]  in_min: the minimum value of the primary space
    \param[in]  in_max: the maximum value of the primary space
    \param[in]  out_min: the minimum value of the secondary space
    \param[in]  out_max: the maximum value of the secondary space
    \param[out] none
    \retval     the mapping value
*/
float map_float(float input_data, float in_min, float in_max, float out_min, float out_max)
{
    return (input_data - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*!
    \brief      get the average fixed value of all the data in the array
    \param[in]  *buffer: the pointer of the destination array
    \param[in]  length: the length of the array
    \param[in]  sum: sum of array values
    \param[in]  input_data: latest input data
    \param[in]  index: array index
    \param[out] none
    \retval     the average value
*/
int16_t moving_average_fixed(int16_t *buffer,uint8_t length,int32_t *sum, int16_t input_data, uint8_t *index)
{
    uint32_t average_value;
    (*index)++;
    if(*index==length){
        *index=0;
    }
    *sum=*sum+input_data-buffer[*index];
    average_value=(uint32_t)(*sum/length);
    buffer[*index] = input_data;

    return (int16_t)average_value;
}

/*!
    \brief      get the average float value of all the data in the array
    \param[in]  *buffer: the pointer of the destination array
    \param[in]  length: the length of the array
    \param[in]  sum: sum of array values
    \param[in]  input_data: latest input data
    \param[in]  index: array index
    \param[out] none
    \retval     the average value
*/
float moving_average_float(float *buffer,uint8_t length,float *sum, float input_data, uint8_t *index)
{
    float average_value;
    (*index)++;
    if(*index==length){
        *index=0;
    }
    *sum=*sum+input_data-buffer[*index];
    average_value=*sum/length;
    buffer[*index] = input_data;

    return average_value;
}

/*!
    \brief      get the mean square error (MSE) of fixed average value
    \param[in]  *buffer: the pointer of the destination array
    \param[in]  length: the length of the array
    \param[in]  average_value: the average value
    \param[out] none
    \retval     mean square error (MSE)
*/
uint16_t mean_square_error_fixed(int16_t *buffer,uint8_t length, int16_t average_value)
{
    uint8_t i;
    uint32_t sum=0;
    uint16_t temp=0;

    for(i=0; i<length; i++){
        temp=(uint16_t)(buffer[i]-average_value);
        sum += (temp*temp);
    }

    return (uint16_t)(sum/length);
}

/*!
    \brief      get the mean square error (MSE) of float average value
    \param[in]  *buffer: the pointer of the destination array
    \param[in]  length: the length of the array
    \param[in]  average_value: the average value
    \param[out] none
    \retval     mean square error (MSE)
*/
float mean_square_error_float(float *buffer,uint8_t length, float average_value)
{
    uint8_t i;
    float sum=0;
    float temp=0;

    for(i=0; i<length; i++){
        temp=buffer[i]-average_value;
        sum += (temp*temp);
    }

    return sum/length;
}

/*!
    \brief      give the source fixed data a step operation which makes it towards the destination fixed data
    \param[in]  value: the source data
    \param[in]  goal: the destination data
    \param[in]  step: the magnitude of the step operation
    \param[out] none
    \retval     the data stepped increased or decreased
*/
int16_t step_towards_fixed(int16_t value, int16_t goal, int16_t step)
{
    if(value < goal){
        if((value + step) < goal){
            value += step;
        }else{
            value = goal;
        }
    }else if (value > goal){
        if((value - step) > goal){
            value -= step;
        }else{
            value = goal;
        }
    }
    return (value);
}

/*!
    \brief      give the source float data a step operation which makes it towards the destination float data
    \param[in]  value: the source data
    \param[in]  goal: the destination data
    \param[in]  step: the magnitude of the step operation
    \param[out] none
    \retval     the data stepped increased or decreased
*/
float step_towards_float(float value, float goal, float step)
{
    if(value < goal){
        if((value + step) < goal){
            value += step;
        }else{
            value = goal;
        }
    }else if (value > goal){
        if((value - step) > goal){
            value -= step;
        }else{
            value = goal;
        }
    }
    return (value);
}

/*!
    \brief      nomalize the input angle
    \param[in]  angle: the angle to be nomalized
    \param[out] none
    \retval     the nomalized angle
*/
float angle_nomalize_float(float angle)
{
    if(angle > MOTOR_PI_2) angle -= MOTOR_PI_2;
    else if(angle < 0.0f) angle += MOTOR_PI_2;

    return (angle);
}

/*!
    \brief      filter the input_data
    \param[in]  b: feed-forward coefficients,the length of the array is equal to filter_length
    \param[in]  a: feed-back coefficients, the length of the array is equal to filter_length
    \param[in]  input_buffer: save previous input data, the length of the array is equal to filter_length
    \param[in]  output_buffer: save previous output data, the length of the array is equal to filter_length-1
    \param[in]  input_data: the latest input data
    \param[in]  filter_length: filter order

    \param[out] none
    \retval     the latest filtered output data
*/
float iir_filter_float(float *b, float *a,float *input_buffer,float *output_buffer,float input_data,uint8_t filter_length)
{
    float data_filter,temp_a = 0,temp_b = 0;
    uint8_t i = 0;

    input_buffer[filter_length-1] = input_data;
    
    temp_a+=input_buffer[filter_length-1]*b[0];
    for(i = 1; i<filter_length; i++){
        temp_a += input_buffer[filter_length-i-1]*b[i];
        temp_b += output_buffer[filter_length-i-1]*a[i];
    }

    data_filter=(temp_a-temp_b)/a[0];

    for(i = 0;i < filter_length-2;i++){
        input_buffer[i] = input_buffer[i+1];
        output_buffer[i] = output_buffer[i+1];
    }

    input_buffer[filter_length-2] = input_buffer[filter_length-1];
    output_buffer[filter_length-2] = data_filter;

    return data_filter;
}

/*!
    \brief      get the sqrt value of the float data
    \param[in]  data: data to be squared
    \param[out] none
    \retval     the sqrt value
*/
float sqrt_float( float input_data)
{
    int i;
    float x, y;
    const float threehalfs = 1.5f;

    x = input_data * 0.5f;
    y  = input_data;
    i  = * ( int * ) &y;
    i  = 0x5f375a86 - ( i >> 1 );
    y  = * ( float * ) &i;

    y  = y * ( threehalfs - ( x * y * y ) );
    y  = y * ( threehalfs - ( x * y * y ) );
    y  = y * ( threehalfs - ( x * y * y ) );

    return input_data*y;
}

/*!
    \brief      floating-point sin function
    \param[in]  input_data: input value in radian (0~2PI)
    \param[out] none
    \retval     points to processed sin output
*/
float sin_float(float input_data)
{
    uint16_t index=0;
    float index_dec=0.0f;
    float sin_value;

    if(input_data<0){
        input_data += MOTOR_PI_2;
    }else if(input_data >= MOTOR_PI_2){
        input_data -= MOTOR_PI_2;
    }

    if((input_data<MOTOR_PI)){
        if(input_data<MOTOR_PI_BY_2){
            index_dec = input_data*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            sin_value=sin_table[index+1]+(sin_table[index+2]-sin_table[index+1])*index_dec;
        }else{
            
            index_dec = (MOTOR_PI-input_data)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            sin_value=sin_table[index+1]+(sin_table[index+2]-sin_table[index+1])*index_dec;
        }
    }else{
        if(input_data<MOTOR_PI_3_BY_2){
            index_dec = (input_data-MOTOR_PI)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            sin_value=-1.0f*(sin_table[index+1]+(sin_table[index+2]-sin_table[index+1])*index_dec);
        }else{
            index_dec = (MOTOR_PI_2-input_data)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            sin_value=-1.0f*(sin_table[index+1]+(sin_table[index+2]-sin_table[index+1])*index_dec);
        }
    }

    return sin_value;
}

/*!
    \brief      floating-point cosine function
    \param[in]  input_data: input value in radian (0~2PI)
    \param[out] none
    \retval     points to processed cosine output
*/
float cos_float(float input_data)
{
    uint16_t index=0;
    float index_dec=0.0f;
    float cos_value;

    if(input_data<0){
        input_data += MOTOR_PI_2;
    }else if(input_data >= MOTOR_PI_2){
        input_data -= MOTOR_PI_2;
    }

    if((input_data<MOTOR_PI)){
        if(input_data<MOTOR_PI_BY_2){
            index_dec = input_data*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            cos_value = sin_table[129-index]+(sin_table[128-index]-sin_table[129-index])*index_dec;
        }else{
            index_dec = (MOTOR_PI-input_data)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            cos_value = -1.0f*(sin_table[129-index]+(sin_table[128-index]-sin_table[129-index])*index_dec);
        }
    }else{
        if(input_data<MOTOR_PI_3_BY_2){
            index_dec = (input_data-MOTOR_PI)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            cos_value = -1.0f*(sin_table[129-index]+(sin_table[128-index]-sin_table[129-index])*index_dec);
        }else{
            index_dec = (MOTOR_PI_2-input_data)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            cos_value = sin_table[129-index]+(sin_table[128-index]-sin_table[129-index])*index_dec;
        }
    }

    return cos_value;
}

/*!
    \brief      floating-point sin and cosine function
    \param[in]  input_data: input value in radian (0~2PI)
    \param[out] sin_value: points to processed sine output
    \param[out] cos_value: points to processed cosine output
    \retval     none
*/
void sin_cos_float(float input_data,float *sin_value,float *cos_value)
{
    uint16_t index=0;
    float index_dec=0.0f;

    if(input_data<0){
        input_data += MOTOR_PI_2;
    }else if(input_data >= MOTOR_PI_2){
        input_data -= MOTOR_PI_2;
    }

    if((input_data<MOTOR_PI)){
        if(input_data<MOTOR_PI_BY_2){
            index_dec = input_data*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            *sin_value = sin_table[index+1]+(sin_table[index+2]-sin_table[index+1])*index_dec;
            *cos_value = sin_table[129-index]+(sin_table[128-index]-sin_table[129-index])*index_dec;
        }else{
            index_dec = (MOTOR_PI-input_data)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            *sin_value = sin_table[index+1]+(sin_table[index+2]-sin_table[index+1])*index_dec;
            *cos_value = -1.0f*(sin_table[129-index]+(sin_table[128-index]-sin_table[129-index])*index_dec);
        }
    }else{
        if(input_data<MOTOR_PI_3_BY_2){
            index_dec = (input_data-MOTOR_PI)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            *sin_value = -1.0f*(sin_table[index+1]+(sin_table[index+2]-sin_table[index+1])*index_dec);
            *cos_value = -1.0f*(sin_table[129-index]+(sin_table[128-index]-sin_table[129-index])*index_dec);
        }else{
            index_dec = (MOTOR_PI_2-input_data)*81.4873308f;
            index = (uint16_t)index_dec;
            index_dec = index_dec-index;
            *sin_value = -1.0f*(sin_table[index+1]+(sin_table[index+2]-sin_table[index+1])*index_dec);
            *cos_value = sin_table[129-index]+(sin_table[128-index]-sin_table[129-index])*index_dec;
        }
    }
}


/*!
    \brief      return radian value (0~2PI) of atan2
    \param[in]  input_data_y: vector ordinate value
    \param[in]  input_data_x: vector abscissa value
    \param[out] none
    \retval     the radian value (0~2PI)
*/
float atan2_float(float input_data_y,float input_data_x)
{
    uint8_t index=0;
    float index_dec=0.0f;
    float angle;

    if(input_data_x>0){
        if(input_data_y>=0){
            if(input_data_y<=input_data_x){
                index_dec = input_data_y*100.0f/input_data_x;
                index = (uint8_t)index_dec;
                index_dec = index_dec - index;
                angle = atan_table[index] + (atan_table[index + 1] - atan_table[index]) * index_dec;
            }
            else{
                index_dec = input_data_x*100.0f/input_data_y;
                index = (uint8_t)index_dec;
                index_dec = index_dec - index;
                angle = MOTOR_PI_BY_2-(atan_table[index] + (atan_table[index + 1] - atan_table[index]) * index_dec);
            }
        }else{
            input_data_y=-input_data_y;
            if(input_data_y<=input_data_x){
                index_dec = input_data_y*100.0f/input_data_x;
                index = (uint8_t)index_dec;
                index_dec = index_dec - index;
                angle = MOTOR_PI_2-(atan_table[index] + (atan_table[index + 1] - atan_table[index]) * index_dec);
            }else{
                index_dec = input_data_x*100.0f/input_data_y;
                index = (uint8_t)index_dec;
                index_dec = index_dec - index;
                angle = MOTOR_PI_3_BY_2+(atan_table[index] + (atan_table[index + 1] - atan_table[index]) * index_dec);
            }
        }
    }else if(input_data_x<0){
        if(input_data_y>=0){
            input_data_x=-input_data_x;
            if(input_data_y<=input_data_x){
                index_dec = input_data_y*100.0f/input_data_x;
                index = (uint8_t)index_dec;
                index_dec = index_dec - index;
                if(index>ATAN_TABLE_SIZE-2){
                    index = ATAN_TABLE_SIZE-2;
                }
                angle = MOTOR_PI - (atan_table[index] + (atan_table[index + 1] - atan_table[index]) * index_dec);
            }else{
                index_dec = input_data_x*100.0f/input_data_y;
                index = (uint8_t)index_dec;
                index_dec = index_dec - index;
                if(index>ATAN_TABLE_SIZE-2){
                    index = ATAN_TABLE_SIZE-2;
                }
                angle = MOTOR_PI_BY_2+(atan_table[index] + (atan_table[index + 1] - atan_table[index]) * index_dec);
            }
        }else{
            if(input_data_y > input_data_x){
                index_dec = input_data_y*100.0f/input_data_x;
                index = (uint8_t)index_dec;
                index_dec = index_dec - index;
                if(index>ATAN_TABLE_SIZE-2){
                    index = ATAN_TABLE_SIZE-2;
                }
                angle = MOTOR_PI + atan_table[index] + (atan_table[index + 1] - atan_table[index]) * index_dec;
            }else{
                index_dec = input_data_x*100.0f/input_data_y;
                index = (uint8_t)index_dec;
                index_dec = index_dec - index;
                angle = MOTOR_PI_3_BY_2-(atan_table[index] + (atan_table[index + 1] - atan_table[index]) * index_dec);
            }
        }
    }else{
        angle = 0;
    }

    return angle;

}

/*!
    \brief      s curve plan
    \param[in]  s_curve: pointer to s_curve_parameter structure
    \param[out] none
    \retval     s curve refernece
*/
float s_curve_plan_float(s_curve_struct *sigmoid_para, float reference)
{
    float time_temp=0.0f;
    
    sigmoid_para->jerk = 800;
    sigmoid_para->fs = 1000;
    
    sigmoid_para->reference_now = (int16_t)reference;
    
    if(sigmoid_para->reference_now != sigmoid_para->reference_previous){
        sigmoid_para->plan_flag = 1;
        sigmoid_para->cnt_flag = 1;
        sigmoid_para->cnt = 0;
    }else{
        if(sigmoid_para->cnt_flag == 1){
            sigmoid_para->cnt++;
        }
    }
    if((sigmoid_para->cnt>1000)&(sigmoid_para->plan_flag == 1)){
        sigmoid_para->start_plan_flag = 1;
        sigmoid_para->plan_flag = 0;
        sigmoid_para->cnt = 0;
        sigmoid_para->cnt_flag = 0;
    }
    if(sigmoid_para->start_plan_flag == 1){
        sigmoid_para->t = 0;
        sigmoid_para->start_reference = sigmoid_para->output;
        sigmoid_para->end_reference = sigmoid_para->reference_now;
        if(sigmoid_para->end_reference>sigmoid_para->start_reference){
            sigmoid_para->t1 =sqrt_float((sigmoid_para->end_reference-sigmoid_para->start_reference)/sigmoid_para->jerk/3);
            sigmoid_para->plan_add_flag = 1;
        }else {
            sigmoid_para->t1 =sqrt_float((sigmoid_para->start_reference-sigmoid_para->end_reference)/sigmoid_para->jerk/3);
            sigmoid_para->plan_decrease_flag = 1;
        }
        sigmoid_para->t2 =2*sigmoid_para->t1;
        
        sigmoid_para->temp1 = 0.5f*sigmoid_para->jerk;
        sigmoid_para->temp2 = 0.5f*sigmoid_para->jerk*sigmoid_para->t1;
        sigmoid_para->temp3 = sigmoid_para->temp2*sigmoid_para->t1;
        sigmoid_para->temp4 = sigmoid_para->jerk*sigmoid_para->t1;
        sigmoid_para->temp5 = sigmoid_para->jerk*sigmoid_para->t1*sigmoid_para->t2;
    }

    if(sigmoid_para->t<4*sigmoid_para->t1){
        sigmoid_para->t += 1.0f/sigmoid_para->fs;
    }

    if(sigmoid_para->plan_add_flag == 1){
        if(sigmoid_para->t<sigmoid_para->t1){
            sigmoid_para->output = sigmoid_para->start_reference+sigmoid_para->temp1 *sigmoid_para->t*sigmoid_para->t;
        }else if(sigmoid_para->t<(sigmoid_para->t1+sigmoid_para->t2)){
            sigmoid_para->output = sigmoid_para->start_reference+sigmoid_para->temp3+sigmoid_para->temp4*(sigmoid_para->t-sigmoid_para->t1);
        }else if(sigmoid_para->t<(sigmoid_para->t1+sigmoid_para->t2+sigmoid_para->t1)){
            time_temp = sigmoid_para->t-sigmoid_para->t1-sigmoid_para->t2;
            sigmoid_para->output = sigmoid_para->start_reference+sigmoid_para->temp3+sigmoid_para->temp5+sigmoid_para->temp4*time_temp-sigmoid_para->temp1*time_temp*time_temp;
        }else{
            sigmoid_para->output = sigmoid_para->end_reference;
            sigmoid_para->plan_add_flag = 0;
        }
    }
    if(sigmoid_para->plan_decrease_flag == 1){
        if(sigmoid_para->t<sigmoid_para->t1){
            sigmoid_para->output = sigmoid_para->start_reference-sigmoid_para->temp1 *sigmoid_para->t*sigmoid_para->t;
        }else if(sigmoid_para->t<(sigmoid_para->t1+sigmoid_para->t2)){
            sigmoid_para->output = sigmoid_para->start_reference-sigmoid_para->temp3-sigmoid_para->temp4*(sigmoid_para->t-sigmoid_para->t1);
        }else if(sigmoid_para->t<(sigmoid_para->t1+sigmoid_para->t2+sigmoid_para->t1)){
            time_temp = sigmoid_para->t-sigmoid_para->t1-sigmoid_para->t2;
            sigmoid_para->output = sigmoid_para->start_reference-sigmoid_para->temp3-sigmoid_para->temp5-sigmoid_para->temp4*time_temp+sigmoid_para->temp1*time_temp*time_temp;
        }else{
            sigmoid_para->output = sigmoid_para->end_reference;
            sigmoid_para->plan_decrease_flag = 0;
        }
    }
    
    sigmoid_para->start_plan_flag = 0;
    sigmoid_para->reference_previous = sigmoid_para->reference_now;
    
    return sigmoid_para->output;
}
