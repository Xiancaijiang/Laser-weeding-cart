#include "periph_autoaim.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx; 
extern DMA_HandleTypeDef hdma_usart1_tx;  



//UART1��tx��DMAģʽ��ʼ��
void usart1_tx_dma_init(void)
{
    //enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
      
      hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
		
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;
}
//UART1��rx��DMAģʽ��ʼ��
void usart1_rx_dma_init(void)
{
    //enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
		 __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);   //����1�����ж�ʹ�ܣ�������䲻�ܽ�����1���жϷ�����

	  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_HISR_TCIF5);
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(&ChariqotRecognition_data[0][0]);   //����������������ַ1   ˫������
		hdma_usart1_rx.Instance->M1AR = (uint32_t)(&ChariqotRecognition_data[1][0]);		//����������������ַ2
		
    hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;  //�������ݳ���
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);  //ʹ��DMA˫������
	  __HAL_DMA_ENABLE(&hdma_usart1_rx);		
}

void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart1_rx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}
