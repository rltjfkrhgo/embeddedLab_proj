#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"

#include "misc.h"

#include "lcd.h"
#include "touch.h"

// ADC 조도센서 경계값
// 맑음/흐림 구분
#define THRESHOLD 1800

// Button 1 : Up 버튼(현수막을 펼침) 
#define LCD_BUT1_RECT_TOP_X 40
#define LCD_BUT1_RECT_TOP_Y 40
#define LCD_BUT1_RECT_BOT_X 110
#define LCD_BUT1_RECT_BOT_Y 80
// 윗 버튼 글자 위치
#define LCD_BUT1_STR_X 70
#define LCD_BUT1_STR_Y 60

// Button 2 : Down 버튼(현수막을 넣음) 
#define LCD_BUT2_RECT_TOP_X 130
#define LCD_BUT2_RECT_TOP_Y 40
#define LCD_BUT2_RECT_BOT_X 200
#define LCD_BUT2_RECT_BOT_Y 80
// 윗 버튼 글자 위치
#define LCD_BUT2_STR_X 140
#define LCD_BUT2_STR_Y 60

// Button 3 : 자동/수동 제어 버튼
#define LCD_BUT3_RECT_TOP_X 40
#define LCD_BUT3_RECT_TOP_Y 120
#define LCD_BUT3_RECT_BOT_X 200
#define LCD_BUT3_RECT_BOT_Y 180
// 아래 버튼 글자 위치
#define LCD_BUT3_STR_X 60
#define LCD_BUT3_STR_Y 140

// OPEN/CLOSE: 글자 위치
#define LCD_SHADE_X 40
#define LCD_SHADE_Y 220
// 가림막 열림/닫힘 현재 상태 위치
#define LCD_SHADE_STAT_X 140
#define LCD_SHADE_STAT_Y 220

// CONTROL: 글자 위치
#define LCD_CONTROL_X 40
#define LCD_CONTROL_Y 250
// 자동/수동 제어 현재 상태 위치
#define LCD_CONTROL_STAT_X 120
#define LCD_CONTROL_STAT_Y 250

// WEATHER : 글자 위치
#define LCD_WEATHER_X 40
#define LCD_WEATHER_Y 280
// 날씨 현재 상태 위치
#define LCD_WEATHER_STAT_X 120
#define LCD_WEATHER_STAT_Y 280

int angle = 35;        // 서보모터 각도
int illumination = 0;  // 조도센서 값
int automatic = 0;     // 자동/수동 상태(0 : 수동/1 : 자동)
int sun = 0;           // 맑음 여부
int rain = 0;          // 빗방울센서 값 0 / 1

void sendDataUART1(uint16_t data);
void sendDataUART2(uint16_t data);
void upRoof();   // 천막을 올린다.
void downRoof(); // 천막을 내린다.

void RCC_Configure(void)
{
    // USART
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // ADC_IN1, PB0
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // ADC1 Clock Enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* USART1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* USART2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // AF Clock Enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // TIM4 Clcok Enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void GPIO_Configure(void)
{
    //RX PA3, PA10
    GPIO_InitTypeDef GPIO_InitStructure_RX;
    GPIO_InitStructure_RX.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10;
    GPIO_InitStructure_RX.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure_RX);

    //TX PA2, PA9
    GPIO_InitTypeDef GPIO_InitStructure_TX;
    GPIO_InitStructure_TX.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_9;
    GPIO_InitStructure_TX.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure_TX.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure_TX);

    // PA0 (ADC_IN1, 조도센서) 를 analog input으로 설정
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PB1: 빗방울 센서
    // digital pull down 으로 설정
    GPIO_InitTypeDef gpio_ipd;
    gpio_ipd.GPIO_Pin = GPIO_Pin_1;
    gpio_ipd.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_ipd.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &gpio_ipd);

    // PB7: TIM4 ch2
    // 서보 모터
    GPIO_InitTypeDef gpio_motor;
    gpio_motor.GPIO_Pin = GPIO_Pin_7;
    gpio_motor.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_motor.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio_motor);
}

// 외부 인터럽트 설정
void EXTI_Configure()
{
    // PB1: 빗방울 센서
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    EXTI_InitTypeDef exti_pb1;
    exti_pb1.EXTI_Line = EXTI_Line1;
    exti_pb1.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_pb1.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti_pb1.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_pb1);
}

// putty 설정
void USART1_Configure(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART_Cmd(USART1, ENABLE);

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART1_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

// 블루투스 설정
void USART2_Configure(void)
{
    USART_InitTypeDef USART2_InitStructure;

    USART_Cmd(USART2, ENABLE);

    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART2_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

// ADC 설정
void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;

    // ADC1 Configuration
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // interrupt enable
    ADC_Cmd(ADC1, ENABLE); // ADC1 enable
    ADC_ResetCalibration(ADC1);

    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// 서보모터 PWM 동작을 위한 타이머
// 타이머 설정
void TIM_Configure()
{
    // 최종 값이 20ms(50hz)가 되도록
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    TIM_TimeBaseStructInit(&TIM_InitStructure);
    TIM_InitStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000) - 1;
    TIM_InitStructure.TIM_Period = 20000 - 1;
    TIM_InitStructure.TIM_ClockDivision = 0;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM4, &TIM_InitStructure);

    // PWM1 Mode configuration: ch1, ch2
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 2300;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

// 인터럽트 우선순위 설정
void NVIC_Configure(void) {

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // PB1: 빗방울 센서
    NVIC_InitTypeDef nvic_exti1;
    nvic_exti1.NVIC_IRQChannel = EXTI1_IRQn;
    nvic_exti1.NVIC_IRQChannelPreemptionPriority = 0x1;
    nvic_exti1.NVIC_IRQChannelSubPriority = 0x1;
    nvic_exti1.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_exti1);

    // 조도 센서 ADC1
    NVIC_InitTypeDef nvic_adc1;
    nvic_adc1.NVIC_IRQChannel = ADC1_2_IRQn;
    nvic_adc1.NVIC_IRQChannelPreemptionPriority = 0x1;
    nvic_adc1.NVIC_IRQChannelSubPriority = 0x1;
    nvic_adc1.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_adc1);

    // putty
    NVIC_InitTypeDef nvic_usart1;
    NVIC_EnableIRQ(USART1_IRQn);
    nvic_usart1.NVIC_IRQChannel = USART1_IRQn;
    nvic_usart1.NVIC_IRQChannelPreemptionPriority = 0x1;
    nvic_usart1.NVIC_IRQChannelSubPriority = 0x1;
    nvic_usart1.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_usart1);

    // 블루투스
    NVIC_InitTypeDef nvic_usart2;
    NVIC_EnableIRQ(USART2_IRQn);
    nvic_usart2.NVIC_IRQChannel = USART2_IRQn;
    nvic_usart2.NVIC_IRQChannelPreemptionPriority = 0x1;
    nvic_usart2.NVIC_IRQChannelSubPriority = 0x1;
    nvic_usart2.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_usart2);
}

/* ================ 핸들러 파트 ================*/

// ADC 조도센서 핸들러
void ADC1_2_IRQHandler() {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
        illumination = ADC_GetConversionValue(ADC1);

        // 맑으면
        if (illumination < THRESHOLD)
        {
            sun = 1;
        }
        // 흐리면
        else
        {
            sun = 0;
        }

        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

// PB1 빗방울 센서 인터럽트 핸들러
void EXTI1_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET)
        {
            rain = 1;
        }
        else
        {
            rain = 0;
        }

        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// putty 핸들러
void USART1_IRQHandler() {
    uint16_t word;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        word = USART_ReceiveData(USART1);
        sendDataUART2(word);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// 블루투스 핸들러
void USART2_IRQHandler() {
    uint16_t word;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        word = USART_ReceiveData(USART2);
        sendDataUART1(word);
        
		if(automatic==0){
			if (word == 'u')
			{
				upRoof();
			}
			else if (word = 'd')
			{
				downRoof();
			}
		}
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/* ================ 함수 ================== */

// PWM duty cycle 변경
void change_pwm_duty_cycle(int percentx10)
{
    int pwm_pulse;
    pwm_pulse = percentx10 * 2000 / 100;

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pwm_pulse;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
}

void openRoof()
{
    change_pwm_duty_cycle(75);
    angle = 75;
}

void closeRoof()
{
    change_pwm_duty_cycle(115);
    angle = 115;
}

void stopRoof()
{
    change_pwm_duty_cycle(35);
    angle = 35;
}

void upRoof()
{
    angle++;
    if (angle > 115)
        angle = 115;
    change_pwm_duty_cycle(angle);
}

void downRoof()
{
    angle--;
    if (angle < 35)
        angle = 35;
    change_pwm_duty_cycle(angle);
}

// putty 방향으로 data 전송
void sendDataUART1(uint16_t data)
{
    /* Wait till TC is set */
    USART_SendData(USART1, data);
}

// 혹시나 휴대폰 방향으로 데이터를 보내야 한다면 사용
void sendDataUART2(uint16_t data) {
    /* Wait till TC is set */
    USART_SendData(USART2, data);
}

void Delay(void) {
    for (int i = 0; i < 100000; i++) {}
}

int main() {
    uint16_t x;
    uint16_t y;

    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    USART1_Configure();
    USART2_Configure();
    TIM_Configure();
    EXTI_Configure();
    NVIC_Configure();

    // ------------------------------------

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);

    // Up 버튼
    LCD_DrawRectangle(LCD_BUT1_RECT_TOP_X, LCD_BUT1_RECT_TOP_Y, LCD_BUT1_RECT_BOT_X, LCD_BUT1_RECT_BOT_Y);
    LCD_ShowString(LCD_BUT1_STR_X, LCD_BUT1_STR_Y, "UP", BLACK, WHITE);
    // Down 버튼 
    LCD_DrawRectangle(LCD_BUT2_RECT_TOP_X, LCD_BUT2_RECT_TOP_Y, LCD_BUT2_RECT_BOT_X, LCD_BUT2_RECT_BOT_Y);
    LCD_ShowString(LCD_BUT2_STR_X, LCD_BUT2_STR_Y, "DOWN", BLACK, WHITE);
    // 자동/수동 제어 버튼
    LCD_DrawRectangle(LCD_BUT3_RECT_TOP_X, LCD_BUT3_RECT_TOP_Y, LCD_BUT3_RECT_BOT_X, LCD_BUT3_RECT_BOT_Y);
    LCD_ShowString(LCD_BUT3_STR_X, LCD_BUT3_STR_Y, "AUTO/MANUAL", BLACK, WHITE);

    LCD_ShowString(LCD_SHADE_X, LCD_SHADE_Y, "OPEN/CLOSE : ", BLACK, WHITE);
    LCD_ShowString(LCD_CONTROL_X, LCD_CONTROL_Y, "CONTROL : ", BLACK, WHITE);
    LCD_ShowString(LCD_WEATHER_X, LCD_WEATHER_Y, "WEATHER : ", BLACK, WHITE);

    while (1)
    {
        // 날씨 상태 표기
        if (rain)
        LCD_ShowString(LCD_WEATHER_STAT_X, LCD_WEATHER_STAT_Y, " RAINY", BLACK, WHITE);
        else if (sun)
            LCD_ShowString(LCD_WEATHER_STAT_X, LCD_WEATHER_STAT_Y, " SUNNY", BLACK, WHITE);
        else
            LCD_ShowString(LCD_WEATHER_STAT_X, LCD_WEATHER_STAT_Y, "CLOUDY", BLACK, WHITE);

        // 자동 모드
        if (automatic)
        {
            LCD_ShowString(LCD_CONTROL_STAT_X, LCD_CONTROL_STAT_Y, "AUTO  ", BLACK, WHITE);

            // 자동이고 비가 오면
            // 천막이 나온다.
            if (rain)
            {
                openRoof();
                LCD_ShowString(LCD_SHADE_STAT_X, LCD_SHADE_STAT_Y, "OPEN ", BLACK, WHITE);
            }

            // 자동이고 비가 안오면
            else
            {
                // 자동이고 비가 안오고 햇빛이면
                // 천막이 나온다.
                if (sun)
                {
                    openRoof();
                    LCD_ShowString(LCD_SHADE_STAT_X, LCD_SHADE_STAT_Y, "OPEN ", BLACK, WHITE);
                }

                // 자동이고 비가 안오고 흐림이면
                // 천막이 들어간다.
                else
                {
                    closeRoof();
                    LCD_ShowString(LCD_SHADE_STAT_X, LCD_SHADE_STAT_Y, "CLOSE", BLACK, WHITE);
                }
            }

        }

        // 수동 모드
        else
        {
            LCD_ShowString(LCD_CONTROL_STAT_X, LCD_CONTROL_STAT_Y, "MANUAL", BLACK, WHITE);
        }

        // 버튼 터치
        if (!(T_INT)) {
            Touch_GetXY(&x, &y, 0);
            Convert_Pos(x, y, &x, &y);

            // up 버튼 눌렀을 때(수동 제어)
            if (x > LCD_BUT1_RECT_TOP_X && x < LCD_BUT1_RECT_BOT_X &&
                y > LCD_BUT1_RECT_TOP_Y && y < LCD_BUT1_RECT_BOT_Y) {
                upRoof();
            }

            else if (x > LCD_BUT2_RECT_TOP_X && x < LCD_BUT2_RECT_BOT_X &&
                     y > LCD_BUT2_RECT_TOP_Y && y < LCD_BUT2_RECT_BOT_Y) {
                downRoof();
            }
            else if (x > LCD_BUT3_RECT_TOP_X && x < LCD_BUT3_RECT_BOT_X &&
                y > LCD_BUT3_RECT_TOP_Y && y < LCD_BUT3_RECT_BOT_Y) {
                automatic = !automatic;
            }
            Delay();
        }// if (!T_INT)
    }

    return 0;
}

