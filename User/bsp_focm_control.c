#include "bsp_focm_control.h"
#include "main.h"
#include "tim.h"
#include <math.h>

#define FOCM_PI_KP       (2.199f)
#define FOCM_PI_KI       (1282.8f)
#define FOCM_VOLTAGE_MAX (2.0f)

#define FOCM_SAMPLE_TIME (0.0000667f)
#define FOCM_SQRT3_HALF  (0.8660254f)
#define FOCM_INV_SQRT3   (0.577350269f)
#define FOCM_POLE_PAIRS  (4.0f)
#define FOCM_IQ_LIMIT   (0.2f)
#define FOCM_DEG_TO_RAD  (0.017453292519943f)
#define FOCM_RAD_TO_DEG  (57.29577951308232f)
#define FOCM_ANGLE_PERIOD (360.0f)

#define FOCM_IQ_STEP    (2.0f * FOCM_SAMPLE_TIME)

typedef struct { 
	float alpha;
	float beta; 
} focm_alpha_beta_t;

typedef struct { 
	float d; 
	float q; 
} focm_dq_t;

focm_data_t focm;

static float d_integral;
static float q_integral;
static float electrical_speed;
static float target_speed;
static float iq_command;

/** 数值限幅函数。 */
static float focm_limit(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

/** Clarke变换：三相电流转换为alpha-beta电流。 */
static focm_alpha_beta_t focm_clarke(float ia, float ib, float ic)
{
    focm_alpha_beta_t result;
    result.alpha = (2.0f * ia - ib - ic) / 3.0f;
    result.beta = (ib - ic) * FOCM_INV_SQRT3;
    return result;
}

/** Park变换：alpha-beta电流转换为dq电流。 */
static focm_dq_t focm_park(focm_alpha_beta_t alpha_beta, float sin_theta, float cos_theta)
{
    focm_dq_t result;
    result.d = alpha_beta.alpha * cos_theta + alpha_beta.beta * sin_theta;
    result.q = -alpha_beta.alpha * sin_theta + alpha_beta.beta * cos_theta;
    return result;
}

/** 反Park变换：dq电压转换为alpha-beta电压。 */
static focm_alpha_beta_t focm_inverse_park(focm_dq_t dq, float sin_theta, float cos_theta)
{
    focm_alpha_beta_t result;
    result.alpha = dq.d * cos_theta - dq.q * sin_theta;
    result.beta = dq.d * sin_theta + dq.q * cos_theta;
    return result;
}

/** 反Clarke变换：alpha-beta电压转换为三相电压。 */
static void focm_inverse_clarke(focm_alpha_beta_t alpha_beta, float *u, float *v, float *w)
{
    *u = alpha_beta.alpha;
    *v = -0.5f * alpha_beta.alpha + FOCM_SQRT3_HALF * alpha_beta.beta;
    *w = -0.5f * alpha_beta.alpha - FOCM_SQRT3_HALF * alpha_beta.beta;
}

/** SVPWM计算：三相电压转换为PWM占空比。 */
static void focm_svpwm(float u, float v, float w, float udc, float *du, float *dv, float *dw)
{
    float maximum = fmaxf(u, fmaxf(v, w));
    float minimum = fminf(u, fminf(v, w));
    float common = 0.5f * (maximum + minimum);
    *du = focm_limit(0.5f + (u - common) / udc, 0.02f, 0.98f);
    *dv = focm_limit(0.5f + (v - common) / udc, 0.02f, 0.98f);
    *dw = focm_limit(0.5f + (w - common) / udc, 0.02f, 0.98f);
}

/** 初始化FOC状态、积分器和PWM输出。 */
void focm_init(void)
{
    focm = (focm_data_t){0};
    d_integral = 0.0f;
    q_integral = 0.0f;
    electrical_speed = 0.0f;
    target_speed = 0.0f;
    iq_command = 0.0f;
    set_focm_disable();
}

/** 设置d轴和q轴电流给定值。 */
void set_focm_current(float id_ref, float iq_ref)
{
    focm.id_ref = id_ref;
    focm.iq_ref = focm_limit(iq_ref, -FOCM_IQ_LIMIT, FOCM_IQ_LIMIT);
}

/** Set the electrical angle in degrees and wrap it to [0, 360). */
void set_focm_angle(float angle)
{
    angle = fmodf(angle, FOCM_ANGLE_PERIOD);
    if (angle < 0.0f) { angle += FOCM_ANGLE_PERIOD; }
    focm.theta = angle;
}

/** 设置机械角速度给定值。 */
void set_focm_speed(float speed)
{
    target_speed = speed;
}

/** 反转目标机械速度方向。 */
void focm_reverse(void)
{
    target_speed = -target_speed;
}

/** 使能驱动器并启动TIM1三相互补PWM。 */
void set_focm_enable(void)
{
    HAL_GPIO_WritePin(Motor1_SD_GPIO_Port, Motor1_SD_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&htim1);
    focm.enabled = 1U;
    electrical_speed = 0.0f;
}

/** 禁止驱动器并关闭三相PWM输出。 */
void set_focm_disable(void)
{
    focm.enabled = 0U;
    iq_command = 0.0f;
    target_speed = 0.0f;
    electrical_speed = 0.0f;
    d_integral = 0.0f;
    q_integral = 0.0f;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(Motor1_SD_GPIO_Port, Motor1_SD_Pin, GPIO_PIN_RESET);
}

/** 执行一次FOC控制周期并更新三相PWM比较值。 */
void focm_control_step(float ia, float ib, float ic, float udc)
{
    float sin_theta;
    float cos_theta;
    float theta_rad;
    float vd;
    float vq;
    float phase_u;
    float phase_v;
    float phase_w;
    uint32_t pwm_period;
    focm_alpha_beta_t current_alpha_beta;
    focm_alpha_beta_t voltage_alpha_beta;
    focm_dq_t current_dq;
    focm_dq_t voltage_dq;

    focm.ia = ia;
    focm.ib = ib;
    focm.ic = ic;
    focm.udc = udc;
    if (!focm.enabled || udc < 5.0f) { return; }

    /* focm.theta is stored in degrees; the C trigonometric functions use radians. */
    theta_rad = focm.theta * FOCM_DEG_TO_RAD;
    sin_theta = sinf(theta_rad);
    cos_theta = cosf(theta_rad);
    current_alpha_beta = focm_clarke(ia, ib, ic);
    current_dq = focm_park(current_alpha_beta, sin_theta, cos_theta);
    focm.id = current_dq.d;
    focm.iq = current_dq.q;

    if (iq_command < focm.iq_ref) {
        iq_command += FOCM_IQ_STEP;
        if (iq_command > focm.iq_ref) { iq_command = focm.iq_ref; }
    }
    else if (iq_command > focm.iq_ref) {
        iq_command -= FOCM_IQ_STEP;
        if (iq_command < focm.iq_ref) { iq_command = focm.iq_ref; }
    }

    vd = focm_limit(FOCM_PI_KP * (focm.id_ref - focm.id) + d_integral, -FOCM_VOLTAGE_MAX, FOCM_VOLTAGE_MAX);
    vq = focm_limit(FOCM_PI_KP * (iq_command - focm.iq) + q_integral, -FOCM_VOLTAGE_MAX, FOCM_VOLTAGE_MAX);
    d_integral = focm_limit(d_integral + FOCM_PI_KI * (focm.id_ref - focm.id) * FOCM_SAMPLE_TIME, -FOCM_VOLTAGE_MAX, FOCM_VOLTAGE_MAX);
    q_integral = focm_limit(q_integral + FOCM_PI_KI * (iq_command - focm.iq) * FOCM_SAMPLE_TIME, -FOCM_VOLTAGE_MAX, FOCM_VOLTAGE_MAX);

    voltage_dq.d = vd;
    voltage_dq.q = vq;
    voltage_alpha_beta = focm_inverse_park(voltage_dq, sin_theta, cos_theta);
    focm_inverse_clarke(voltage_alpha_beta, &phase_u, &phase_v, &phase_w);
    focm_svpwm(phase_u, phase_v, phase_w, udc, &focm.duty_u, &focm.duty_v, &focm.duty_w);

    pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(focm.duty_u * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(focm.duty_v * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(focm.duty_w * pwm_period));

    if (electrical_speed < target_speed) {
        electrical_speed += 10.0f * FOCM_SAMPLE_TIME;
        if (electrical_speed > target_speed) { electrical_speed = target_speed; }
    }
    else if (electrical_speed > target_speed) {
        electrical_speed -= 10.0f * FOCM_SAMPLE_TIME;
        if (electrical_speed < target_speed) { electrical_speed = target_speed; }
    }

    /* electrical_speed is mechanical rad/s; integrate electrical angle in degrees. */
    focm.theta += electrical_speed * FOCM_POLE_PAIRS * FOCM_SAMPLE_TIME * FOCM_RAD_TO_DEG;
    if (focm.theta >= FOCM_ANGLE_PERIOD) { focm.theta -= FOCM_ANGLE_PERIOD; }
    if (focm.theta < 0.0f) { focm.theta += FOCM_ANGLE_PERIOD; }
}

/** 获取当前FOC状态数据。 */
const focm_data_t *get_focm_data(void)
{
    return &focm;
}


