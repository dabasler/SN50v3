/**
  ******************************************************************************
  * @file    
  * @author  Dragino
  * @version 
  * @date    
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_delay.h"
#include "log.h"
#include "I2C_sensor.h"
#include "I2C_A.h"
#include "bsp.h"

extern bool bh1750flags;
extern bool iic_noack;
	
uint8_t SHT31_CheckSum_CRC8(uint8_t* Result,uint8_t num) 
{
	uint8_t data[2];
	uint8_t index=0;
	index=num;
	
	data[0] = Result[index];
	data[1] = Result[index+1];

	uint32_t POLYNOMIAL = 0x131;
	uint8_t crc = 0xFF;
	uint8_t bit = 0;
	uint8_t byteCtr = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < 2; ++byteCtr)
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc == Result[index+2]) {
		return 1;
	}
	else {
		return 0;
	}
}

void SHT31_Read(sht31_t *sht31_data)
{
	uint8_t rxdatas[6];
	uint8_t data[2] = {0x2C, 0x06};	
  bool read_status=1;	
	uint8_t check_number=0;	

	I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(I2C_Write_Len(0x44,0x01,2,data)==1)
		{
			read_status=0;
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(60);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(I2C_Read_Len(0x44,0x01,6,rxdatas)==1)
			{
				read_status=0;
				delay_ms(20);
			}
			
			if( SHT31_CheckSum_CRC8(rxdatas,0)==0 && SHT31_CheckSum_CRC8(rxdatas,3)==0)
			{
				read_status=0;
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}

	if(read_status==1)
	{	
		sht31_data->temp_sht=((rxdatas[0]<<8)+rxdatas[1])*175.0/(65536-1)-45.0;
		sht31_data->hum_sht=((rxdatas[3]<<8)+rxdatas[4])*100.0/(65536-1);
		if(sht31_data->hum_sht>100)	
		{
			sht31_data->hum_sht=100;
		}
		else if(sht31_data->hum_sht<0)
		{		
		  sht31_data->hum_sht=0;
		}	
			
		if(sht31_data->temp_sht>125)
		{
			sht31_data->temp_sht=125;
		}
		else if(sht31_data->temp_sht<-40)
		{
			sht31_data->temp_sht=-40;
		}		
	}
	else
	{
		sht31_data->temp_sht=3276.7;
		sht31_data->hum_sht=6553.5;
	}	
}

uint8_t check_sht31_connect(void)
{
	uint8_t rxdata[3];
	uint8_t data[2] = {0xF3, 0x2D};		
	I2C_GPIO_MODE_Config();
	I2C_Write_Len(0x44,0x01,2,data);
	delay_ms(50);
	I2C_Read_Len(0x44,0x01,3,rxdata);
	if(SHT31_CheckSum_CRC8(rxdata,0)==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t SHT20_CheckSum_CRC8(uint8_t* Result) 
{
	uint8_t data[2];
	data[0] = Result[0];
	data[1] = Result[1];

	uint32_t POLYNOMIAL = 0x131;
	uint8_t crc = 0;
	uint8_t bit = 0;
	uint8_t byteCtr = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < 2; ++byteCtr)
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc == Result[2]) {
		return 1;
	}
	else {
		return 0;
	}
}

float SHT20_RH(void)
{
	uint8_t txdata[1]={0xf5};//Humidity measurement
  uint8_t rxdata[3];
	uint16_t AD_code;
	float hum;
	uint8_t check_number=0;	
  bool read_status=1;	

	I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(I2C_Write_Len(0x40,0x01,1,txdata)==1)
		{
			read_status=0;
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(30);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(I2C_Read_Len(0x40,0x01,3,rxdata)==1)
			{
				read_status=0;
				delay_ms(20);
			}
			
			if(SHT20_CheckSum_CRC8(rxdata)==0)
			{
				read_status=0;
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}

	if(read_status==1)
	{	
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;
		hum=(AD_code*125.0/65536.0)-6.0;
		
		if(hum>100)	
		{
			hum=100;
		}
		else if(hum<0)
		{		
		  hum=0;
		}		
	}
	else
	{
		hum=6553.5;
	}	
  return hum;	
}

float SHT20_RT(void)
{
	uint8_t txdata[1]={0xf3};//Humidity measurement
  uint8_t rxdata[3];
	uint16_t AD_code;
	float tem;
	uint8_t check_number=0;	
  bool read_status=1;	

	I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(I2C_Write_Len(0x40,0x01,1,txdata)==1)
		{
			read_status=0;
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(90);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(I2C_Read_Len(0x40,0x01,3,rxdata)==1)
			{
				read_status=0;
				delay_ms(20);
			}
			
			if(SHT20_CheckSum_CRC8(rxdata)==0)
			{
				read_status=0;
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}
	
	if(read_status==1)
	{	
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;
		tem=(AD_code*175.72/65536.0)-46.85;
			
		if(tem>125)
		{
			tem=125;
		}
		else if(tem<-40)
		{
			tem=-40;
		}		
	}
	else
	{
		tem=3276.7;
	}	
  return tem;
}

uint8_t check_sht20_connect(void)
{
	uint8_t rxdata[3];
	uint8_t data[1] = {0xF3};		
	I2C_GPIO_MODE_Config();
	I2C_Write_Len(0x40,0x01,1,data);
	delay_ms(90);
	I2C_Read_Len(0x40,0x01,3,rxdata);
	if(SHT20_CheckSum_CRC8(rxdata)==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint16_t bh1750_read(void)
{
	uint8_t rxdata[2];	
	uint16_t AD_code;
	uint16_t luminance;	
	
	I2C_GPIO_MODE_Config();
	
	delay_ms(10);//Required	
	I2C_Write_Byte(0x23,0x01); 
	I2C_Write_Byte(0x23,0x20);  
  delay_ms(200);	
  I2C_Read_Len(0x23,0x00,2,rxdata);
	I2C_Write_Byte(0x23,0x07);	

	if(iic_noack==1)
	{
		luminance=0;	
	}
	else
	{
		AD_code=(rxdata[0]<<8)+rxdata[1];
		luminance=AD_code/1.2;
  }
	
	iic_noack=0;
	
	return luminance;
}

void I2C_read_data(sensor_t *sensor_data,uint8_t flag_temp, uint8_t message)
{	
	I2C_GPIO_MODE_Config();
	if(flag_temp==0)
	{
		sensor_data->temp_sht=6553.5;
		sensor_data->hum_sht=6553.5;
	} 
	else if(flag_temp==1)
	{
		sensor_data->temp_sht=SHT20_RT();
		sensor_data->hum_sht=SHT20_RH(); 
		if(message==1)
		{
			LOG_PRINTF(LL_DEBUG,"SHT2x_temp:%.1f,SHT2x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			delay_ms(20);
		}
	}
	else if(flag_temp==2)
	{			       	
		sht31_t temphum_data;		
		SHT31_Read(&temphum_data);				
		sensor_data->temp_sht=temphum_data.temp_sht;
		sensor_data->hum_sht=temphum_data.hum_sht;	
		if(message==1)
		{
			LOG_PRINTF(LL_DEBUG,"SHT3x_temp:%.1f,SHT3x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			delay_ms(20);
		}
	}
	else if(flag_temp==3)
	{		
    bh1750flags=1;		
		sensor_data->illuminance=bh1750_read();		
		if(message==1)
		{			
			LOG_PRINTF(LL_DEBUG,"BH1750_lum:%d\r\n",sensor_data->illuminance);	
      delay_ms(20);			
		}				
	}
	else if(flag_temp == 4)
	{
		sht31_t temphum_data;
		SHT40_Read(&temphum_data);
		sensor_data->temp_sht = temphum_data.temp_sht;
		sensor_data->hum_sht  = temphum_data.hum_sht;

		if(message == 1)
		{
			LOG_PRINTF(LL_DEBUG,"SHT40_temp:%.1f,SHT40_hum:%.1f\r\n",sensor_data->temp_sht, sensor_data->hum_sht);
			delay_ms(20);
		}
	}		
  I2C_GPIO_MODE_ANALOG();	
}

void LidarLite_init(void)
{
   uint8_t sigCountMax[1]={0x80};
   uint8_t acqConfigReg[1]={0x08};
   uint8_t refCountMax[1]={0x05};
   uint8_t thresholdBypass[1]={0x00};	
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x02,1,sigCountMax);
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x04,1,acqConfigReg);
   waitbusy(1);
   I2C_Write_reg_Len(0x62,0x12,1,refCountMax);	 
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x1c,1,thresholdBypass);	 
   waitbusy(1);	
}

uint16_t LidarLite(void)
{
	uint8_t dataByte[1]={0x04};
  uint8_t rxdata1[1];
	uint8_t rxdata2[1];
	uint16_t distance;
  waitbusy(1);
  I2C_Write_reg_Len(0x62,0x00,1,dataByte);
  if(waitbusy(2)<999)
	{		
		I2C_Read_reg_Len(0x62,0x0f,1,rxdata1);
		I2C_Read_reg_Len(0x62,0x10,1,rxdata2);
		distance=(rxdata1[0]<<8)+rxdata2[0];
		if(distance>4000)
		{
			distance=0;			
			return distance;			
		}
		else
		{
			return distance*10;	
		}			
	}
	else
	{
	 distance=65535;
	 return distance;			
	}	
}

uint16_t waitbusy(uint8_t mode)
{
  uint16_t busyCounter = 0;
	uint8_t busy[1]={0x01};
  while (busy[0])      
  {
   if((mode==1)&&(busyCounter > 99))
   {
		return busyCounter;			 
   }
	 
   if((mode==2)&&(busyCounter > 999))
   {
		return busyCounter;			 
   }	 
	 I2C_Read_reg_Len(0x62,0x01,1,busy);
	 busy[0] &=0x01;
	 busyCounter++;
  }
	return busyCounter;	
}



uint8_t SHT40_CheckSum_CRC8(uint8_t* data)
{
    uint8_t crc = 0xFF;
    uint32_t POLYNOMIAL = 0x31;

    for (uint8_t i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ POLYNOMIAL;
            else
                crc <<= 1;
        }
    }

    return crc;
}



void SHT40_Read(sht31_t *sht40_data)
{
    uint8_t txdata[1] = {0xFD}; // High-precision measurement command
    uint8_t rxdata[6];
    bool read_status = 1;
    uint8_t check_number = 0;

    I2C_GPIO_MODE_Config();

    // Send measurement command
    do
    {
        read_status = 1;
        check_number++;
        if (I2C_Write_Len(0x44, 0x01, 1, txdata) == 1)
        {
            read_status = 0;
            delay_ms(20);
        }
    } while (read_status == 0 && check_number < 4);

    // Wait and read data
    if (read_status == 1)
    {
        delay_ms(10);
        check_number = 0;
        do
        {
            read_status = 1;
            check_number++;
            if (I2C_Read_Len(0x44, 0x01, 6, rxdata) == 1)
            {
                read_status = 0;
                delay_ms(20);
            }

            if (SHT40_CheckSum_CRC8(rxdata) != rxdata[2] || SHT40_CheckSum_CRC8(&rxdata[3]) != rxdata[5])
            {
                read_status = 0;
                delay_ms(20);
            }
        } while (read_status == 0 && check_number < 4);
    }

    // Convert if success
    if (read_status == 1)
    {
        uint16_t raw_temp = (rxdata[0] << 8) + rxdata[1];
        uint16_t raw_hum = (rxdata[3] << 8) + rxdata[4];

        sht40_data->temp_sht = -45.0 + 175.0 * ((float)raw_temp / 65535.0);
        sht40_data->hum_sht = 100.0 * ((float)raw_hum / 65535.0);

        // Clamp values
        if (sht40_data->hum_sht > 100)
            sht40_data->hum_sht = 100;
        else if (sht40_data->hum_sht < 0)
            sht40_data->hum_sht = 0;

        if (sht40_data->temp_sht > 125)
            sht40_data->temp_sht = 125;
        else if (sht40_data->temp_sht < -40)
            sht40_data->temp_sht = -40;
    }
    else
    {
        sht40_data->temp_sht = 3276.7;
        sht40_data->hum_sht = 6553.5;
    }
}


uint8_t check_sht40_connect(void)
{
    uint8_t txdata[1] = {0xFD}; // High-precision measurement command
    uint8_t rxdata[6];

    I2C_GPIO_MODE_Config();
    I2C_Write_Len(0x44, 0x01, 1, txdata);
    delay_ms(10);
    I2C_Read_Len(0x44, 0x01, 6, rxdata);

    if (SHT40_CheckSum_CRC8(rxdata) == rxdata[2] && SHT40_CheckSum_CRC8(&rxdata[3]) == rxdata[5])
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
