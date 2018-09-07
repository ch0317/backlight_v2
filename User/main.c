#include "stm32f10x.h"
#include "./uart/bsp_uart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "./pwm/bsp_pwm.h"
#include "./i2c/bsp_i2c_gpio.h"
#include "./i2c/bsp_i2c_ee.h"



#define CR '\r'
#define LF '\n'
#define CRLF "\r\n"

#define COMMAND_MAX_LENGTH 32


// messages
#define UNKNOWN_COMMAND "unknown command"
#define COMMAND_TOO_LONG "command too long"
#define WRONG_COMMAND "wrong command format, please check!"


//内部flash写入的起始地址与结束地址
#define WRITE_START_ADDR  ((uint32_t)0x08008000)
#define WRITE_END_ADDR    ((uint32_t)0x0800C000)

#define FLASH_PAGE_SIZE    ((uint16_t)0x400)	//1024
#define PAGE_ADDR(n)   (0x08000000 + n*1024)

//存在flash中的变量地址
#define BRIGHTNESS_ADDR     PAGE_ADDR(7)         //亮度
#define MODE_ADDR           (PAGE_ADDR(7) + 2)   //模式
#define CURRENT_ADDR        (PAGE_ADDR(7) + 4)   //电流

#define brightness_index 0
#define mode_index 1
#define current_index 2
uint16_t para_array[3];


// send one byte through USART
void usart_send(const char chr) {
    while(!(USART1->SR & USART_SR_TC));
    USART1->DR = chr;
}

// send a string through USART
void usart_send_string(const char *s) {
    int i=0;
    while (s[i]) {
        usart_send(s[i++]);
    }
}

void usart_send_newline(void) {
    usart_send_string(CRLF);
}

void usart_send_line(const char *s) {
    usart_send_string(s);
    usart_send_string(CRLF);
}


void get_flash_para()
{
	para_array[brightness_index] = *((uint8_t*)BRIGHTNESS_ADDR);
	para_array[mode_index] = *((uint8_t*)MODE_ADDR);
	para_array[current_index] = *((uint8_t*)CURRENT_ADDR);
	
}


void last_setting_get()
{
		//将flash中的值通过i2c写入8556
		uint16_t mode_in_flash = *(uint16_t*)(MODE_ADDR);
		uint8_t mode = (uint8_t)mode_in_flash;
		uint16_t brightness_in_flash = *(uint16_t*)(BRIGHTNESS_ADDR);
		uint8_t brightness = (uint8_t)brightness_in_flash;
		uint16_t current_in_flash = *(uint16_t*)(CURRENT_ADDR);
		uint8_t current = (uint8_t)current_in_flash;
	
		ee_WRITE_BYTES(BRIGHTNESS_CONTROL,&brightness,1);
		
	
		uint8_t mode_mask = 0xff;
		mode_mask = mode_mask & (mode << 1);
		
		uint8_t device_control = 0;
		ee_READ_BYTES(DEVICE_CONTROL,&device_control,1);
		
		printf("\r\ndevice_control_register:%d\r\n",device_control);
		device_control = device_control & mode_mask;   //set BRT_MODE
		
		ee_WRITE_BYTES(DEVICE_CONTROL,&device_control,1);
		ee_WRITE_BYTES(CFG0,&current,1);
		
}

/*
CMD TO HANDLE:
	COMMANDLINE_PRINTF("BL2DINIT <Brightness 0-255>\n");

	COMMANDLINE_PRINTF("BLSWITCH <Mode>\n");
	COMMANDLINE_PRINTF("BKL_SWITCH <Mode>\n");
	COMMANDLINE_PRINTF("BLSETTINGGET - turn on last saved backlight with saved or default settings\n");
	COMMANDLINE_PRINTF("BLSETMODE0\n");
	COMMANDLINE_PRINTF("BLSETMODE1\n");

*/

void handle_command(char *command) {

	char *tokens[5];
	int i = 0;
	int ret = 0;
	
	
	for (char *p = strtok(command," "); p != NULL; p = strtok(NULL, " "))
	{
		tokens[i] = p;
		
		printf("token[%d]:%s\t",i,tokens[i]);
		i++;
	}
	printf("\n");
	
	if(!strncmp(tokens[0],"BL2DINIT",strlen("BL2DINIT")))
	{
		if(i != 2)
		{
			usart_send_line(WRONG_COMMAND);
			usart_send_line("format:BL2DINIT <Brightness 0-255>, example: BL2DINIT 128");
			return;
		}
		
		int ibrightness = atoi(tokens[1]);
		uint8_t ibrightness_read = 0;
		if(ibrightness < 0 || ibrightness > 255)
		{
			usart_send_line("ibrightness wrong, range is 0~255");
			return;
		}
		
		uint8_t brightness = (uint8_t)ibrightness;
		
		printf("\r\nbrightness:%d\r\n",brightness);
		
		ret = ee_WRITE_BYTES(BRIGHTNESS_CONTROL,&brightness,1);

		ret = ee_READ_BYTES(BRIGHTNESS_CONTROL,&ibrightness_read,1);

		if(brightness == ibrightness_read)
		{
			usart_send_line("set success!");
		}
		printf("ibrightness_read=%d\n",ibrightness_read);
		
	}
	/*
	else if(!strncmp(tokens[0],"BL3DINIT",strlen("BL3DINIT")))
	{
		if(i != 3)
		{
			usart_send_line(WRONG_COMMAND);
			usart_send_line("format: BL3DINIT <Brightness 0-255> <Current 5-30mA>, example: BL3DINIT 128 5");
			return;
		}
		
		int ibrightness = atoi(tokens[1]);
		uint8_t brightness = (uint8_t)ibrightness;
		if(ibrightness < 0 || ibrightness > 255)
		{
			usart_send_line("ibrightness wrong, range is 0~255");
			return;
		}
		
		int icurrent = atoi(tokens[2]);
		if(icurrent < 5 || icurrent > 30)
		{
			usart_send_line("icurrent wrong, range is 5~30");
			return;
		}
		
		uint8_t current = (uint8_t)icurrent;
		printf("\r\nbrightness:%d,current:%d\r\n",brightness,current);
		 
		ret = ee_WRITE_BYTES(BRIGHTNESS_CONTROL_ADDRESS,&brightness,1);
		ret |= ee_WRITE_BYTES(CFG0,&current,1);
		printf("write ret=%d\n",ret);
		uint8_t current_read = 0;
		uint8_t brightness_read = 0;
		ret = ee_READ_BYTES(BRIGHTNESS_CONTROL_ADDRESS,&brightness_read,1);
		ret = ee_READ_BYTES(CFG0,&current_read,1);
		printf("brightness_read = %d\n",brightness_read);
		printf("current_read = %d\n",current_read);
		
	}
	*/
	else if(!strncmp(tokens[0],"BLSWITCH",strlen("BLSWITCH")))
	{
		
		if(i != 2)
		{
			usart_send_line(WRONG_COMMAND);
			usart_send_line("format: BLSWITCH <Mode>, example: BLSWITCH 3");
			return;
		}
		
		//Device Control Register, Address 01h
		//7 		|		6 		|		5 		|		4 		|		3 	|		2 	|		1 	|		0
		//FAST 																					BRT_MODE[1:0] 	 	BL_CTL
		int mode_to_switch = atoi(tokens[1]);
		
		if(mode_to_switch < 0 || mode_to_switch > 4)
		{
			usart_send_line("mode wrong, range is 1~4");
			return;
		}
		mode_to_switch = mode_to_switch & 0x3;
		uint8_t mode_mask = 0xff;
		mode_mask = mode_mask & (mode_to_switch << 1);
		
		uint8_t device_control = 0;
		ee_READ_BYTES( DEVICE_CONTROL,&device_control,1);
		
		printf("\r\ndevice_control:%d\r\n",device_control);
		device_control = device_control & mode_mask;   //set BRT_MODE to Device Control register
		
		ret = ee_WRITE_BYTES(DEVICE_CONTROL,&device_control,1);

		uint8_t read_device_control = 0;
		ret = ee_READ_BYTES(DEVICE_CONTROL,&read_device_control,1);
		printf("read read_mode=%d\n",read_device_control);
	}
	else if(!strncmp(tokens[0],"BLSETTINGGET",strlen("BLSETTINGGET")))
	{
		if(i != 1)
		{
			usart_send_line(WRONG_COMMAND);
			usart_send_line("format: BLSETTINGGET");
			return;
		}
		last_setting_get();
	}
	else if(!strncmp(tokens[0],"BLSETMODE0",strlen("BLSETMODE0")) ||
		!strncmp(tokens[0],"BLSETMODE1",strlen("BLSETMODE1")))
	{
		if(i != 1)
		{
			usart_send_line(WRONG_COMMAND);
			usart_send_line("format: BLSETMODEn");
			return;
		}
		
		printf("\r\n handle cmd: %s",tokens[0]);
		FLASH_Unlock();
		
		//擦除前获取保存的参数
		get_flash_para();
		FLASH_ErasePage(PAGE_ADDR(7));
		
		if(!strncmp(tokens[0],"BLSETMODE0",strlen("BLSETMODE0")))
		{
			para_array[mode_index] = 2;
		}
		else
		{
			para_array[mode_index] = 3;
		}
		
		uint32_t write_addr = PAGE_ADDR(7);
		uint16_t *wrtie_data_p = para_array;
		uint16_t *read_p ;
		int i = 0;
		
		for(i = 0;i < 3;i++)
		{
			//写入数据到flash中
			FLASH_ProgramHalfWord(write_addr,*wrtie_data_p);
			write_addr += 2;	
			wrtie_data_p++;		
		}
		
		read_p = (uint16_t *)PAGE_ADDR(7);
		for(i = 0;i < 3;i++)
		{
			printf("\r\nwrite[%d] = %d , read[%d] = %d",i,para_array[i],i,read_p[i]);
			if(para_array[i] != read_p[i])
			{
				printf("\r\n数据校验失败");
				break;
			}
		}
		if(i == 3)
		{
			printf("\r\nWrite Data to flash success,the mode has been set to 3 and saved.");
		}
			
		FLASH_Lock();
		
		//set the 8556 mode with i2c 
		int mode_to_switch = para_array[mode_index];
		mode_to_switch = mode_to_switch & 0x3; //取最后两bit
		uint8_t mode_mask = 0xff;
		mode_mask = mode_mask & (mode_to_switch << 1);
		
		uint8_t device_control = 0;
		ee_READ_BYTES( DEVICE_CONTROL,&device_control,1);
		
		printf("\r\nold_mode:%d\r\n",device_control);
		device_control = device_control & mode_mask;   //set BRT_MODE
		
		ee_WRITE_BYTES( DEVICE_CONTROL,&device_control,1);
		
	}
/*
	else if(!strncmp(tokens[0],"BLSET3DCURRENT",strlen("BLSET3DCURRENT")))
	{
		if(i != 1)
		{
			usart_send_line(WRONG_COMMAND);
			usart_send_line("format: BLSET3DCURRENT");
			return;
		}
		
		printf("\r\n handle cmd: %s",tokens[0]);
		
		//获取当前的电流值
		uint8_t current = 0;
		//I2C_EE_BufferRead(&current,CFG0,1);
		printf("\r\n The current:%d",current);
		
		FLASH_Unlock();
		
		//擦除前获取保存的参数
		get_flash_para();
		FLASH_ErasePage(PAGE_ADDR(7));
		
		para_array[current_index] = current;
		
		uint32_t write_addr = PAGE_ADDR(7);
		uint16_t *wrtie_data_p = para_array;
		uint16_t *read_p ;
		int i = 0;
		
		for(i = 0;i < 3;i++)
		{
			//写入数据到flash中
			FLASH_ProgramHalfWord(write_addr,*wrtie_data_p);
			write_addr += 2;	
			wrtie_data_p++;		
		}
		
		read_p = (uint16_t *)PAGE_ADDR(7);
		for(i = 0;i < 3;i++)
		{
			printf("\r\nwrite[%d] = %d , read[%d] = %d",i,para_array[i],i,read_p[i]);
			if(para_array[i] != read_p[i])
			{
				printf("\r\n数据校验不等");
				break;
			}
		}
		if(i == 3)
		{
			printf("\r\nWrite Data to flash success,the mode has been set to 3 and saved.");
		}
			
		FLASH_Lock();
	}
	*/
}

char usart_buf[COMMAND_MAX_LENGTH];
unsigned short usart_buf_length=0;

void USART1_IRQHandler() {
    unsigned char received;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        received = USART_ReceiveData(USART1);

        if (received == CR) {
            usart_buf[usart_buf_length] = 0;
            usart_send_newline();

            handle_command(usart_buf);
            usart_buf_length = 0;
        } else if (received == LF) {
            // ignore
        } else {
            if (usart_buf_length == COMMAND_MAX_LENGTH) {
                usart_send_newline();
                usart_send_line(COMMAND_TOO_LONG);
                usart_buf_length = 0;
                return;

            }

            usart_buf[usart_buf_length++] = received;

            // echo
            usart_send(received);
        }
    }
}


void delay(u16 num)
{
  u16 i,j;
  for(i=0;i<num;i++)
    for(j=0;j<0x800;j++);
}


int main(void)
{
	
	USART_Config();
	
	ADVANCE_TIM_Init();

	printf("init...\r\n");
	
	Usart_SendString(DEBUG_USARTx, "pwm is on, f=2500hz\r\n");

	i2c_GPIO_Config();
	
	delay(2000);
	
	if(ee_CHECK_DEVICE(WRITE_DIR_8556) == 0)
	{
		printf("\r\n8556 has been detected.\r\n");
	}
	else
	{
		printf("\r\n8556 has not been detected\r\n");
	}

	delay(2000);
	
	int ret = 0;

	//Setting Current
#if 0
	uint8_t cfg0_read = 0;
	uint8_t cfg0 = 0;
	ret = ee_READ_BYTES(CFG0,&cfg0_read,1);
	
	cfg0 = 0xff;
	ret = ee_WRITE_BYTES(CFG0,&cfg0,1);
	cfg0_read = 0;
	ret = ee_READ_BYTES(CFG0,&cfg0_read,1);
	printf("\r\ncfg0 now is 0x%x\r\n",cfg0_read);
	
	delay(100);
	uint8_t cfg1_read = 0;
	uint8_t cfg1 = 0;
	ret = ee_READ_BYTES(CFG1,&cfg1_read,1);
	cfg1 = 0x3f;
	ret = ee_WRITE_BYTES(CFG1,&cfg1,1);
	cfg1_read = 0;
	ret = ee_READ_BYTES(CFG1,&cfg1_read,1);
	printf("\r\ncfg1 now is 0x%x\r\n",cfg1_read);
	
	delay(100);
	uint8_t cfg4_read = 0;
	uint8_t cfg4 = 0;
	ret = ee_READ_BYTES(CFG4,&cfg4_read,1);
	cfg4 = 0x70;
	ret = ee_WRITE_BYTES(CFG4,&cfg4,1);
	cfg4_read = 0;
	ret = ee_READ_BYTES(CFG4,&cfg4_read,1);
	printf("\r\ncfg4 now is 0x%x\r\n",cfg4_read);
	
	uint8_t cfg5_read = 0;
	uint8_t cfg5 = 0;
	ret = ee_READ_BYTES(CFG5,&cfg5_read,1);
	cfg5 = 0x04;
	ret = ee_WRITE_BYTES(CFG5,&cfg5,1);
	cfg5_read = 0;
	ret = ee_READ_BYTES(CFG5,&cfg5_read,1);
	printf("\r\ncfg5 now is 0x%x\r\n",cfg5_read);

	delay(100);
	//EMI Reduction Settings
	uint8_t cfg7_read = 0;
	uint8_t cfg7 = 0;
	ret = ee_READ_BYTES(CFG7,&cfg7_read,1);
	cfg7 = 0x3f;
	ret = ee_WRITE_BYTES(CFG7,&cfg7,1);
	cfg7_read = 0;
	ret = ee_READ_BYTES(CFG7,&cfg7_read,1);
	printf("\r\ncfg7 now is 0x%x\r\n",cfg7_read);
	
	//设置MODE
	uint8_t device_control = 0;
	uint8_t device_control_read = 0;
	device_control |= (1 << 1);
	ret = ee_WRITE_BYTES(DEVICE_CONTROL_ADDRESS,&device_control,1);
	ret = ee_READ_BYTES(DEVICE_CONTROL_ADDRESS,&device_control_read,1);
	printf("\r\ndevice_control now is 0x%x\r\n",device_control_read);
	

#endif	
#if 0
	//设置MODE
	uint8_t device_control = 0;
	uint8_t device_control_read = 0;
	device_control |= (1 << 1);
	ret = ee_WRITE_BYTES(DEVICE_CONTROL,&device_control,1);
	ret = ee_READ_BYTES(DEVICE_CONTROL_ADDRESS,&device_control_read,1);
	printf("\r\ndevice_control now is 0x%x\r\n",device_control_read);

	delay(100);
	uint8_t cfg2_read = 0;
	uint8_t cfg2 = 0;
	ret = ee_READ_BYTES(CFG2,&cfg2_read,1);
	cfg2 = 0x08;
	ret = ee_WRITE_BYTES(CFG2,&cfg2,1);
	cfg2_read = 0;
	ret = ee_READ_BYTES(CFG2,&cfg2_read,1);
	printf("\r\ncfg2 now is 0x%x\r\n",cfg2_read);	


	delay(100);
	//VBOOST参数
	uint8_t cfg6_read = 0;
	uint8_t cfg6 = 0;
	ret = ee_READ_BYTES(CFG6,&cfg6_read,1);
	cfg6 = 0x8d;
	ret = ee_WRITE_BYTES(CFG6,&cfg6,1);
	ret = ee_READ_BYTES(CFG6,&cfg6_read,1);
	printf("\r\ncfg6 now is 0x%x\r\n",cfg6_read);
	

	delay(100);
	//VBOOST_MAX
	uint8_t cfg9_read = 0;
	uint8_t cfg9 = 0;
	ret = ee_READ_BYTES(CFG9,&cfg9_read,1);
	cfg9 = 0x90;
	ret = ee_WRITE_BYTES(CFG9,&cfg9,1);
	cfg9_read = 0;
	ret = ee_READ_BYTES(CFG9,&cfg9_read,1);
	printf("\r\ncfg9 now is 0x%x\r\n",cfg9_read);
	
	delay(100);
	//Setting Boost Voltage
	uint8_t cfga_read = 0;
	uint8_t cfga = 0;
	ret = ee_READ_BYTES(CFGA,&cfga_read,1);
	cfga = 0x38;
	ret = ee_WRITE_BYTES(CFGA,&cfga,1);
	ret = ee_READ_BYTES(CFGA,&cfga_read,1);
	printf("\r\ncfga now is 0x%x\r\n",cfga_read);
	
	uint8_t cfg9e_read = 0;
	uint8_t cfg9e = 0;
	
	//设置VBOOST_RANGE
	ret = ee_READ_BYTES(CFG9E,&cfg9e_read,1);
	//cfg9e = cfg9e_read | (1 << 5);
	cfg9e = 0x20;
	ret = ee_WRITE_BYTES(CFG9E,&cfg9e,1);
	ret = ee_READ_BYTES(CFG9E,&cfg9e_read,1);
	printf("\r\ncfg9e now is 0x%x\r\n",cfg9e_read);
	
/*
	delay(100);
	ret = ee_READ_BYTES(CFG2,&cfg2_read,1);
	cfg2 = 0x08;
	ret = ee_WRITE_BYTES(CFG2,&cfg2,1);
	cfg2_read = 0;
	ret = ee_READ_BYTES(CFG2,&cfg2_read,1);
	//printf("\r\ncfg2 now is 0x%x\r\n",cfg2_read);	
*/

	uint8_t status = 0;
	ret = ee_READ_BYTES(STATUS_ADDRESS,&status,1);
	printf("\r\nstatus now is 0x%x\r\n",status);
#endif	

#if 1

	static unsigned char i2c_init_table[][2] = {
    {0xa1, 0x7F},  //hight bit(8~11)(0~0X66e set backlight)
    {0xa0, 0x66},  //low bit(0~7)  20mA
    {0x16, 0x3F},  // 5channel LED enable 0x1F
    {0xa9, 0xc0},  //VBOOST_MAX 25V
    {0x9e, 0x22},  //VBOOST_RANGE
    {0xa2, 0x2b},  //BOOST_FSET_EN PWM_FSET_EN 
		{0xa6, 0x47},	 //VBOOST 
    {0x01, 0x05},  //0x03 pwm+I2c set brightness,0x5 I2c set brightness
    {0xff, 0xff},  //ending flag
};
#endif

#if 0
static unsigned char i2c_init_table[][2] = {
    {0xa1, 0x7f},  //hight bit(8~11)(0~0X66e set backlight)
    {0xa0, 0x66},  //low bit(0~7)  20mA
    {0x16, 0x3F},  // 5channel LED enable 0x1F
    {0xa9, 0xa0},  //VBOOST_MAX 25V
		{0xaa, 0x30},  //ADAPTIVE
    {0x9e, 0x12},  //VBOOST_RANGE
    {0xa2, 0x2b},  //BOOST_FSET_EN PWM_FSET_EN 
		{0xa6, 0x3f},	 //VBOOST 0.42*11
    {0x01, 0x03},  //0x03 pwm+I2c set brightness,0x5 I2c set brightness
    {0xff, 0xff},  //ending flag
};
#endif

#if 0
	static unsigned char i2c_init_table[][2] = {
    {0xa1, 0x7F}, //hight bit(8~11)(0~0X66e set backlight)
    {0xa0, 0x66},  //low bit(0~7)  20mA
    {0x16, 0x3F}, // 5channel LED enable 0x1F
    {0xa9, 0xc0}, //VBOOST_MAX 25V
    {0x9e, 0x22},  //VBOOST_RANGE
    {0xa2, 0x2b},  //BOOST_FSET_EN PWM_FSET_EN 
		{0xa6, 0x07},	//VBOOST 0.42*11
    {0x01, 0x05}, //0x03 pwm+I2c set brightness,0x5 I2c set brightness
    {0xff, 0xff},//ending flag
};
#endif
/*
static unsigned char i2c_init_table[][2] = {
    {0xa0, 0x66},  //low bit(0~7)  20mA
    {CFG9, 0xA0}, //VBOOST_MAX 25V
		{CFGA, 0x30},
    {CFG6, 0x80}, 
    {0xff, 0xff},//ending flag
};
*/
    uint8_t tData[3];
    int i=0, ending_flag=0;
		uint8_t read_data;
		while (ending_flag == 0) {
		if (i2c_init_table[i][0] == 0xff) {    //special mark
				if (i2c_init_table[i][1] == 0xff) { //ending flag
						ending_flag = 1;
				}
		}
		else {
				tData[0]=i2c_init_table[i][0];
				tData[1]=i2c_init_table[i][1];
				ret = ee_WRITE_BYTES(tData[0], &tData[1], 1);
				delay(1000);
				ret = ee_READ_BYTES(tData[0], &read_data, 1);
				printf("\r\naddress = 0x%x,read_data now is 0x%x\r\n",tData[0],read_data);
		}
		i++;
}


	uint8_t status = 0;
	ret = ee_READ_BYTES(STATUS,&status,1);
	printf("\r\nstatus now is 0x%x\r\n",status);

	
	while(1)
	{
		uint8_t status = 0;
		ret = ee_READ_BYTES(STATUS,&status,1);
		status = (status >> 4)& (0x1);
		printf("\r\nstatus now is 0x%x\r\n",status);
		delay(3000);
		#if 0
		delay(10000);
		i = 0;
		ending_flag = 0;
		uint8_t read_data;
		while (ending_flag == 0) {
		if (i2c_init_table[i][0] == 0xff) {    //special mark
				if (i2c_init_table[i][1] == 0xff) { //ending flag
						ending_flag = 1;
				}
		}
		else {
				tData[0]=i2c_init_table[i][0];
				tData[1]=i2c_init_table[i][1];
				ret = ee_READ_BYTES(tData[0], &read_data, 1);
				printf("\r\n---address = 0x%x,read_data now is 0x%x\r\n",tData[0],read_data);
		}
		i++;
}
		#endif
	};

}

