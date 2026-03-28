/**
 ******************************************************************************
 * 文件名程: usart_task.c
 * 作    者: 浩然
 * 版    本: V1.0
 * 编写日期: 2024-03-29
 * 功    能: 定时执行串口任务
 ******************************************************************************
 */
/* 包含头文件 ----------------------------------------------------------------*/
#include "task_uart.h"
#include "drv_motor.h"

TXDATA TxData;
volatile uint16_t UsartTaskId = 5;
volatile uint16_t UsartTaskTim = 0;

/**
 * 函数功能: 定时执行串口任务
 * 输入参数:
 * 返 回 值:
 * 说    明:
 */
void task_uart(void)
{
    switch (UsartTaskId)
    {
    case 5:
    {
        TxData.tail[0] = 0x00;
        TxData.tail[1] = 0x00;
        TxData.tail[2] = 0x80;
        TxData.tail[3] = 0x7f;
        UsartTaskId = 10;
    }
    break;

    case 10:
    {
        if (UsartTaskTim >= 1) // 0.2ms
        {
            UsartTaskTim = 0;
            UsartTaskId = 20;
        }
    }
    break;

    case 20:
    {
        if (__HAL_DMA_GET_COUNTER(&hdma_usart1_tx) == 0)
        {
            //				TxData.fdata[0] = tMC.Sample.IuReal;     //打印U相电流值
            //				TxData.fdata[1] = tMC.Sample.IvReal;			//打印V相电流值
            //				TxData.fdata[2] = tMC.Sample.IwReal;			//打印W相电流值

            //				TxData.fdata[0] = tMC.Encoder.EncoderVal;    //打印编码器原始值
            //				TxData.fdata[1] = tMC.Encoder.ElectricalVal; //打印校准后电角度值

            TxData.fdata[0] = tMC.IqPid.Ref; // 打印Q轴目标电流值
            TxData.fdata[1] = tMC.IqPid.Fbk; // 打印Q轴实际电流值

            //				TxData.fdata[0] = tMC.SpdPid.Ref;   //打印目标速度值
            //				TxData.fdata[1] = tMC.SpdPid.Fbk;	 //打印实际速度值

            //				TxData.fdata[0] = tMC.PosPid.Ref;   //打印目标位置值
            //				TxData.fdata[1] = tMC.PosPid.Fbk;	 //打印实际位置值

            //				TxData.fdata[0] = tMC.HPLL.ETheta;  //打印高频注入得到的电角度

            //        TxData.fdata[0] = tMC.SPLL.ETheta;  //打印滑膜观测器得到的电角度

            __HAL_DMA_DISABLE(&hdma_usart1_tx);
            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&TxData, sizeof(TxData)); // 发送结构体(不用对float做处理了)
            __HAL_DMA_ENABLE(&hdma_usart1_tx);
        }
        UsartTaskId = 10;
    }
    break;

    default:
        break;
    }
}
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);

//			printf("%0.3f,%0.3f\n",(float)tMC.Sample.IuRaw,(float)tMC.Sample.IwRaw);                               //采样原始值

//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
//		  printf("%0.3f,%0.3f,%0.3f\n",tMC.Sample.IuReal,tMC.Sample.IvReal,tMC.Sample.IwReal);                    //三相电流值（正弦波）
//			printf("%0.3f,%0.3f\n",tMC.Foc.Ialpha,tMC.Foc.Ibeta);                                                  //α轴和β轴电流值（正弦波）
//		  printf("%0.3f,%0.3f,%0.3f\n",(float)tMC.Foc.Channel1,(float)tMC.Foc.Channel2,(float)tMC.Foc.Channel3);  //三相占空比（马鞍波）
//			printf("%0.3f,%0.3f\n",tMC.IdPid.Ref,tMC.IdPid.Fbk);                                                   //D轴电流目标值和反馈值
//			printf("%0.3f,%0.3f\n",tMC.IqPid.Ref,tMC.IqPid.Fbk);                                                   //Q轴电流目标值和反馈值
//	    printf("%0.3f\n",(float)tMC.Encoder.ElectricalVal);                                                   //电角度值
//			printf("%0.3f,%0.3f\n",tMC.TShapedAccDec.SpeedOut/7,tMC.Speed.MechanicalSpeed);                        //目标速度与实际速度（机械速度，单位RPM）
//      printf("%0.3f\n",tMC.SPLL.ETheta);                                                                    //滑膜观测器计算得到的电角度
//      printf("%0.3f\n",tMC.HPLL.ETheta);                                                                    //高频注入计算得到的电角度
