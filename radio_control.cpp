#include "pxt.h"

namespace pins {
    void digitalWritePin(int name, int value);
    void spiPins(int mosi, int miso, int sck);
    void spiFrequency(int frequency);
    void spiFormat(int bits, int mode);
    void spiTransfer(Buffer command, Buffer response);
    int  spiWrite(int value);
}
namespace pxt {
    void sleep_us(uint64_t us);
}


namespace luBit {

    //% shim=luBit::TestFunction_C
    void TestFunction_C() {
        // 发送10个1000Hz脉冲 (每个脉冲周期1ms)  
        for(int i = 0; i < 10; i++) {
            // 高电平阶段
            pins::digitalWritePin(MICROBIT_ID_IO_P1, 1);
            pxt::sleep_us(500); // 500微秒高电平
            
            // 低电平阶段
            pins::digitalWritePin(MICROBIT_ID_IO_P1, 0);
            pxt::sleep_us(500); // 500微秒低电平，完成1ms周期
        }
    }


 
    // nRF24L01寄存器地址宏定义
    #define NRF24L01_REG_CONFIG      0x00
    #define NRF24L01_REG_EN_AA       0x01
    #define NRF24L01_REG_EN_RXADDR   0x02
    #define NRF24L01_REG_SETUP_AW    0x03
    #define NRF24L01_REG_SETUP_RETR  0x04
    #define NRF24L01_REG_RF_CH       0x05
    #define NRF24L01_REG_RF_SETUP    0x06
    #define NRF24L01_REG_STATUS      0x07
    #define NRF24L01_REG_OBSERVE_TX  0x08
    #define NRF24L01_REG_RPD         0x09
    #define NRF24L01_REG_RX_ADDR_P0  0x0A
    #define NRF24L01_REG_RX_ADDR_P1  0x0B
    #define NRF24L01_REG_RX_ADDR_P2  0x0C
    #define NRF24L01_REG_RX_ADDR_P3  0x0D
    #define NRF24L01_REG_RX_ADDR_P4  0x0E
    #define NRF24L01_REG_RX_ADDR_P5  0x0F
    #define NRF24L01_REG_TX_ADDR     0x10
    #define NRF24L01_REG_RX_PW_P0    0x11
    #define NRF24L01_REG_RX_PW_P1    0x12
    #define NRF24L01_REG_RX_PW_P2    0x13
    #define NRF24L01_REG_RX_PW_P3    0x14
    #define NRF24L01_REG_RX_PW_P4    0x15
    #define NRF24L01_REG_RX_PW_P5    0x16
    #define NRF24L01_REG_FIFO_STATUS 0x17
    #define NRF24L01_REG_DYNPD       0x1C
    #define NRF24L01_REG_FEATURE     0x1D

    // NRF24L01寄存器操作命令（统一风格）
    #define NRF24L01_CMD_R_REGISTER      0x00  // 读配置寄存器,低5位为寄存器地址
    #define NRF24L01_CMD_W_REGISTER      0x20  // 写配置寄存器,低5位为寄存器地址
    #define NRF24L01_CMD_RD_RX_PLOAD     0x61  // 读RX有效数据,1~32字节
    #define NRF24L01_CMD_WR_TX_PLOAD     0xA0  // 写TX有效数据,1~32字节
    #define NRF24L01_CMD_FLUSH_TX        0xE1  // 清除TX FIFO寄存器.发射模式下用
    #define NRF24L01_CMD_FLUSH_RX        0xE2  // 清除RX FIFO寄存器.接收模式下用
    #define NRF24L01_CMD_REUSE_TX_PL     0xE3  // 重新使用上一包数据,CE为高,数据包被不断发送
    #define NRF24L01_CMD_NOP             0xFF  // 空操作,可以用来读状态寄存器

    // NRF24L01发送接收数据宽度定义
    #define TX_ADR_WIDTH    5    // 5字节的地址宽度
    #define RX_ADR_WIDTH    5    // 5字节的地址宽度
    #define TX_PLOAD_WIDTH  32   // 32字节的用户数据宽度
    #define RX_PLOAD_WIDTH  32   // 32字节的用户数据宽度

    // NRF24L01_REG_STATUS 
    #define STATUS_MAX_TX  	    0x10  //达到最大发送次数中断
    #define STATUS_TX_OK       	0x20  //TX发送完成中断
    #define STATUS_RX_OK   	    0x40  //接收到数据中断

     //P15（MOSI） P14（MISO） P13（SCK） P8(CE) P2(CSN)
    #define Clr_NRF24L01_CSN pins::digitalWritePin(MICROBIT_ID_IO_P2, 0) // 片选选中
    #define Set_NRF24L01_CSN pins::digitalWritePin(MICROBIT_ID_IO_P2, 1) // 片选取消
    #define Clr_NRF24L01_CE  pins::digitalWritePin(MICROBIT_ID_IO_P8, 0) // 使能nrf24L01
    #define Set_NRF24L01_CE  pins::digitalWritePin(MICROBIT_ID_IO_P8, 1) // 不使能nrf24L01

    #define AUTO_ON  2000 //定点模式打开
    #define AUTO_OFF 1000 //定点模式关闭


    //配对密码//各套件主控与遥控的匹配码相同才可以实现通信
    //只购买一套者默认地址5
    //手上有两套及以上着，实现一对一通信匹配码在这里更改，遥控与主控上会有地址序号来区分不同套，
    //地址5
    const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//本地地址
    const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//接收地址RX_ADDR_P0 == RX_ADDR
    ////地址4
    //const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE4};	//本地地址
    //const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE4};	//接收地址RX_ADDR_P0 == RX_ADDR
    ////地址3
    //const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE3};	//本地地址
    //const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE3};	//接收地址RX_ADDR_P0 == RX_ADDR
    ////地址2
    //const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE2};	//本地地址
    //const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE2};	//接收地址RX_ADDR_P0 == RX_ADDR
    ////地址1
    //const uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE1};	//本地地址
    //const uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE1};	//接收地址RX_ADDR_P0 == RX_ADDR

    typedef volatile struct 
    {
        uint16_t roll;
        uint16_t pitch;
        uint16_t thr;
        uint16_t yaw;	
        uint16_t AUX1;
        uint16_t AUX2;
        uint16_t AUX3;
        uint16_t AUX4;
    }_st_Remote;



    // 写nRF24L01寄存器
    uint8_t NRF24L01_Write_Reg(uint8_t regaddr, uint8_t data) {
        int status;
        Clr_NRF24L01_CSN;
        // 写命令: 0x20 | regaddr
        status = pins::spiWrite(NRF24L01_CMD_W_REGISTER | (regaddr & 0x1F));
        pins::spiWrite(data);
        Set_NRF24L01_CSN;
        return (uint8_t)status; // 返回状态寄存器
    }

    // 读nRF24L01寄存器
    uint8_t NRF24L01_Read_Reg(uint8_t regaddr) {
        int reg_val;
        Clr_NRF24L01_CSN;
        // 读命令: 0x00 | regaddr
        pins::spiWrite(NRF24L01_CMD_R_REGISTER | (regaddr & 0x1F));
        reg_val = pins::spiWrite(0xFF); // 发送空数据，接收寄存器内容
        Set_NRF24L01_CSN;
        return (uint8_t)reg_val;
    }


    //在指定位置读出指定长度的数据
    //     *pBuf:数据指针
    //返回值,此次读到的状态寄存器值 
    uint8_t NRF24L01_Read_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
    {
        uint8_t status;
        Buffer cmd = pxt::mkBuffer(NULL, datalen + 1); // 第一个字节是寄存器地址
        Buffer resp = pxt::mkBuffer(NULL, datalen + 1); // 响应数据

        cmd->data[0] = NRF24L01_CMD_R_REGISTER | (regaddr & 0x1F); // 读命令+寄存器地址
        for (uint8_t i = 1; i <= datalen; i++)
            cmd->data[i] = 0xFF; // dummy字节

        Clr_NRF24L01_CSN;
        pins::spiTransfer(cmd, resp);
        Set_NRF24L01_CSN;

        status = resp->data[0]; // 第一个字节是状态
        for (uint8_t i = 0; i < datalen; i++)
            pBuf[i] = resp->data[i + 1]; // 取出实际数据

        return status;
    }

    //在指定位置写指定长度的数据
    //    *pBuf:数据指针
    //返回值,此次读到的状态寄存器值
    uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
    {
        uint8_t status;
        Buffer cmd = pxt::mkBuffer(NULL, datalen + 1); // 第一个字节是写命令
        Buffer resp = pxt::mkBuffer(NULL, datalen + 1); // 响应数据

        cmd->data[0] = NRF24L01_CMD_W_REGISTER | (regaddr & 0x1F); // 写命令+寄存器地址
        for (uint8_t i = 0; i < datalen; i++)
            cmd->data[i + 1] = pBuf[i]; // 填充要写的数据

        Clr_NRF24L01_CSN;
        pins::spiTransfer(cmd, resp);
        Set_NRF24L01_CSN;

        status = resp->data[0]; // 第一个字节是状态
        return status;
    }

    
    //该函数初始化NRF24L01到TX模式
    //设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,
    //选择RF频道,波特率和LNA HCURR PWR_UP,CRC使能
    //当CE变高后,即进入RX模式,并可以接收数据了		   
    //CE为高大于10us,则启动发送.	 
    void TX_Mode(void)
    {														 
        Clr_NRF24L01_CE;	    
        //写TX节点地址 
        NRF24L01_Write_Buf(NRF24L01_REG_TX_ADDR,(uint8_t *)TX_ADDRESS,TX_ADR_WIDTH);    
        //设置TX节点地址,主要为了使能ACK	  
        NRF24L01_Write_Buf(NRF24L01_REG_RX_ADDR_P0,(uint8_t *)RX_ADDRESS,RX_ADR_WIDTH); 

        //使能通道0的自动应答    
        NRF24L01_Write_Reg(NRF24L01_REG_EN_AA,0x00);     
        //使能通道0的接收地址  
        NRF24L01_Write_Reg(NRF24L01_REG_EN_RXADDR,0x01); 
        //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
        NRF24L01_Write_Reg(NRF24L01_REG_SETUP_RETR,0x1a);
        //设置RF通道为40
        NRF24L01_Write_Reg(NRF24L01_REG_RF_CH,1);       
        //设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
        NRF24L01_Write_Reg(NRF24L01_REG_RF_SETUP,0x07);  //0x27  250K   0x07 1M
        //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX发送模式,开启所有中断
        NRF24L01_Write_Reg(NRF24L01_REG_CONFIG,0x0e);    
        // CE为高,10us后启动发送
        Set_NRF24L01_CE;                                  
    }	

    //启动NRF24L01发送一个遥控数据帧
    //txbuf:待发送数据首地址
    //返回值:发送完成状况
    uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
    {
        uint8_t status;
        Clr_NRF24L01_CE;
        NRF24L01_Write_Buf(NRF24L01_CMD_WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); // 写数据到TX BUF 32字节
        Set_NRF24L01_CE; // 启动发送
        // 等待发送完成，通过轮询 STATUS 寄存器判断
        do {
            status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
        } while (!(status & (STATUS_MAX_TX | STATUS_TX_OK)));

        // 清除 TX_DS 或 MAX_RT 中断标志
        NRF24L01_Write_Reg(NRF24L01_REG_STATUS, status);

        if (status & STATUS_MAX_TX) {
            NRF24L01_Write_Reg(NRF24L01_CMD_FLUSH_TX, 0xff); // 清除TX FIFO寄存器
            return STATUS_MAX_TX;
        }
        if (status & STATUS_TX_OK) {
            return STATUS_TX_OK;
        }
        return 0xff; // 其他原因发送失败
    }

    //发送遥控数据
    void NRF_SEND(_st_Remote * Remote)
    {
        volatile uint8_t tx_data[32] = {0xAA,0xAF,0x03,32-5};//匿名通信协议
        // 按顺序打包各通道数据（高位在前，低位在后）
        tx_data[4]  = (Remote->thr   >> 8) & 0xFF;
        tx_data[5]  = (Remote->thr)        & 0xFF;
        tx_data[6]  = (Remote->pitch >> 8) & 0xFF;
        tx_data[7]  = (Remote->pitch)      & 0xFF;
        tx_data[8]  = (Remote->roll  >> 8) & 0xFF;
        tx_data[9]  = (Remote->roll)       & 0xFF;
        tx_data[10] = (Remote->yaw   >> 8) & 0xFF;
        tx_data[11] = (Remote->yaw)        & 0xFF;
        tx_data[12] = (Remote->AUX1  >> 8) & 0xFF;
        tx_data[13] = (Remote->AUX1)       & 0xFF;
        tx_data[14] = (Remote->AUX2  >> 8) & 0xFF;
        tx_data[15] = (Remote->AUX2)       & 0xFF;
        tx_data[16] = (Remote->AUX3  >> 8) & 0xFF;
        tx_data[17] = (Remote->AUX3)       & 0xFF;
        tx_data[18] = (Remote->AUX4  >> 8) & 0xFF;
        tx_data[19] = (Remote->AUX4)       & 0xFF;

        // 校验位
        tx_data[31] = 0;
        for(uint8_t i=0; i<31; i++) {
            tx_data[31] += tx_data[i];
        }
        NRF24L01_TxPacket((uint8_t*)&tx_data); //调用NRF发射数据
    }
    
    //% shim=luBit::InitNrf24_C
    bool InitNrf24_C() {
        // 初始化SPI口 
        // 注意：这些引脚需要根据实际硬件连接进行调整

        uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
        uint8_t buf1[5];
        uint8_t i;

        pins::digitalWritePin(MICROBIT_ID_IO_P15, 0);
        pins::digitalWritePin(MICROBIT_ID_IO_P14, 0);
        pins::digitalWritePin(MICROBIT_ID_IO_P13, 0);
        pxt::sleep_us(100);
        pins::spiPins(MICROBIT_ID_IO_P15, MICROBIT_ID_IO_P14, MICROBIT_ID_IO_P13); //O,I,CLK
        pins::spiFormat(8,0);//设置SPI格式,8字节，SCK模式：平常低电平+上升沿读取数据
        pins::spiFrequency(1000000);//设置SPI频率
        Clr_NRF24L01_CE; 	                                //使能24L01
	    Set_NRF24L01_CSN;                                   //SPI片选取消

        // SPI初始化完成，接下来可以配置NRF24L01模块

        //上电检测NRF24L01是否在位
        //写5个数据然后再读回来进行比较，
        NRF24L01_Write_Buf(NRF24L01_REG_TX_ADDR,buf,TX_ADR_WIDTH);//写入5个字节的地址.
        NRF24L01_Read_Buf(NRF24L01_REG_TX_ADDR,buf1,TX_ADR_WIDTH);//读出写入的地址  	
        for(i=0;i<5;i++)
        {
            if(buf1[i]!=buf[i]) 
            {  
                return false; //NRF24L01驱动失败
            }	
        }  

        TX_Mode();//发送模式
        return true;    
    }


    //% shim=luBit::SendRC_C
    bool SendRC_C(int throttle, int pitch, int roll, int yaw) {
        pins::digitalWritePin(MICROBIT_ID_IO_P15, 0);
        return true;
    }
    
}
