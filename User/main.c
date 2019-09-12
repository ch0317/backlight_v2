#include "stm32f10x.h"
#include "./uart/bsp_uart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "./pwm/bsp_pwm.h"
#include "./i2c/bsp_i2c_gpio.h"
#include "./i2c/bsp_i2c_ee.h"

#define bool u8
#define false 0
#define true 1

#define CR '\r'
#define LF '\n'
#define CRLF "\r\n"

#define __DEBUG__  
#ifdef __DEBUG__  
#define DEBUG(format,...) printf("[debug]"format"\n", ##__VA_ARGS__)
#else  
#define DEBUG(format,...)  
#endif  

#define COMMAND_MAX_LENGTH 128

enum CMD_ID{
    CMD_NULL,
    HELP = 1,
    INFO,
    DUMP,
    I2CREAD,
    I2CWRITE,
    BLSETBRIGHTNESS,
    BLSETRATIOS,
    BLSWITCH,
    BLSETTINGSGET,
    BLSET2DCURRENT,
    BLSET3DCURRENT,
    BLSETPWM,
    SET2DCTRLMODE,
    POKE32,
    PEEK32,
    POKE16,
    PEEK16,
    POKE8,
    PEEK8,
    GPIOREAD,
    GPIOREADPIN,
    GPIOSETPIN,
    GPIOCLRPIN,
    BLFACTORYRESET,
    setDisplayConfig,
    getDisplayConfig,
    UNKNOWCMD = 0xff,
};

enum DISPLAY_KEY{
    KEY_NULL,
    DisplayClass,
    DisplayID,
    DisplaySizeInMm,
    DotPitchInMm,
    PanelResolution,
    NumViews,
    InterlacingMatrix1,
    InterlacingMatrix2,
    InterlacingVector,
    Slant,
    AlignmentOffset,
    MirrorViewsX,
    MirrorViewsY,
    ReverseViews,
    ActCoefficientsX,
    ActCoefficientsY,
    Gamma,
    SystemDisparityPixels,
    ConvergenceDistance,
    CenterViewNumber,
    ViewBoxSize,
    ALL_CONFIG
};



// messages
#define UNKNOWN_COMMAND  "\"ERROR\": \"UnknownCommand\",\r\n"
#define COMMAND_TOO_LONG "{\"ERROR\": \"CommandTooLong\"}"
#define WRONG_FORMAT     "\"ERROR\": \"WrongFormat\",\r\n"
#define VALUE_ERROR      "\"ERROR\": \"ValueError\",\r\n"


//内部flash写入的起始地址与结束地址
#define WRITE_START_ADDR  ((uint32_t)0x08008000)
#define WRITE_END_ADDR    ((uint32_t)0x0800C000)

#define FLASH_PAGE_SIZE   ((uint16_t)0x400) //1024
#define PAGE_ADDR(n)      (0x08000000 + n*1024)

#define CONF_PAGE1 56
#define CONF_PAGE2 57
#define CONF_DEFAULT_PAGE1 58
#define CONF_DEFAULT_PAGE2 59

#define SHORT_VALUE_LEN 32
#define LONG_VALUE_LEN 128



#define DUTY_MAX_3D 2400   // 2400/4096 = 58.6% ,which is PWM 3D's LIMIT

//tokens for cmd analysis
char *g_tokens[5];
char board_info[128] = "v2.0    2019-8-9    Copyright(c)2019Leia,Inc";
char cmd_buffer[COMMAND_MAX_LENGTH];
char g_usart_buf[COMMAND_MAX_LENGTH];
int cmd_received = 0;
int cmd_too_long = 0;
unsigned short usart_buf_length=0;
char cmd_help_str[1500] =   "{\"HELP\" : \"Display all commands\",\r\n" \
                            "\"INFO\" : \"Display Board Information\",\r\n" \
                            "\"DUMP\" : \"Dump 8556 register\",\r\n" \
                            "\"I2CREAD  <uint8_addr>\"               : \"Read 8556 reg with i2c\",\r\n" \
                            "\"I2CWRITE <uint8_addr> <uint8_value>\" : \"Write 8556 reg with i2c\",\r\n" \
                            "\"BLSETBRIGHTNESS  <Brightness 0-255>\" : \"default 102\",\r\n" \
                            "\"BLSETRATIOS <Mode 2 or 3>  <RATIO_2D 0.00-1.00>  <RATIO_3D 0.00-3.00>\" : \"\",\r\n" \
                            "\"BLSWITCH    <Mode 2 or 3>\" : \"Switch mode\",\r\n" \
                            "\"SET2DCTRLMODE <Mode 0 or 1>\" : \"2D control mode, 0 for i2c, 1 for pwm\",\r\n" \
                            "\"BLSETPWM    <2 or 3> <Duty cycle 0-4096>\" : \"0~4096 map to 1% ~ 100%\",\r\n" \
                            "\"BLSET2DCURRENT  <0.00-25.00mA>\" : \"Set 2D current\",\r\n" \
                            "\"BLSET3DCURRENT  <0.00-30.00mA>\" : \"Set 3D current\",\r\n" \
                            "\"BLSETTINGSGET\" : \"Show settings\",\r\n" \
                            "\"POKE32 <Address> <Data>\" : \"\",\r\n" \
                            "\"POKE16 <Address> <Data>\" : \"\",\r\n" \
                            "\"POKE8  <Address> <Data>\" : \"\",\r\n" \
                            "\"PEEK32 <Address>\" : \"\",\r\n" \
                            "\"PEEK16 <Address>\" : \"\",\r\n" \
                            "\"PEEK8  <Address>\" : \"\",\r\n" \
                            "\"GPIOREAD    <port>\" : \"\",\r\n" \
                            "\"GPIOREADPIN <port> <pin>\" : \"\",\r\n" \
                            "\"GPIOSETPIN  <port> <pin>\" : \"\",\r\n" \
                            "\"GPIOCLRPIN  <port> <pin>\" : \"\",\r\n" \
                            "\"BLFACTORYRESET\" : \"Reset to Factory Defaults.\"}";

#define I2C_CTRL 0
#define PWM_CTRL 1

#define mode_index 0
#define brightness_index 1
#define mode2ratio2d_index 2
#define mode2ratio3d_index 3
#define mode3ratio2d_index 4
#define mode3ratio3d_index 5
#define ctrlmode2d_index 6
#define STORE_LEN 7

/*
  These values will be saved in stm32's flash PAGE_ADDR(54).
  [0]mode, 
  [1]brightness, 
  [2]mode2ratio2d(0.00~1.00,default:1.0,map to 1~100 in memory), 
  [3]mode2ratio3d(0.00~3.00,default:0.15,map to 1~300 in memory), 
  [4]mode3ratio2d(0.00~1.00,default:0,map to 1~100 in memory), 
  [5]mode3ratio3d(0.00~3.00,default:1.0,map to 1~300 in memory),
  [6]ctrlmode(0 for i2c, 1 for pwm)
*/
static uint32_t g_setting[STORE_LEN] = {2,102,100,15,0,100,I2C_CTRL}; 

int cmd_id = CMD_NULL;
int cmd_key_num = 0;
static unsigned char i2c1_init_table[][2] = {
    {0xa1, 0x5F},  //hight bit(8~11)
    {0xa0, 0xff},  //low bit(0~7)  25mA
    {0x16, 0x3F},  //5channel LED enable 0x1F
    {0xa9, 0x40},  //VBOOST_MAX = 010,17.9~23.1
    {0x9e, 0x22},  //VBOOST_RANGE = 1
    {0xa2, 0x2b},  //BOOST_FSET_EN PWM_FSET_EN 
    {0xa6, 0x05},  //VBOOST 
    {0x01, 0x05},  //0x01 pwm only set brightness,0x5 I2c set brightness
    {0xff, 0xff},  //ending flag
};


static unsigned char i2c_init_table[][2] = {
    {0xa1, 0x5F},  //hight bit(8~11)
    {0xa0, 0xff},  //low bit(0~7)  25mA
    {0x16, 0x3F},  //5channel LED enable 0x1F
    {0xa9, 0x60},  //VBOOST_MAX = 010,17.9~23.1
    {0x9e, 0x22},  //VBOOST_RANGE = 1
    {0xa2, 0x2b},  //BOOST_FSET_EN PWM_FSET_EN 
    {0xa6, 0x05},  //VBOOST 
    {0x01, 0x05},  //0x01 pwm only set brightness,0x5 I2c set brightness
    {0xff, 0xff},  //ending flag
};



char ShortValue[16][SHORT_VALUE_LEN] = {
    "A0",
    "Unknown",
    "[294,121]",
    "[0.0766,0.056]",
    "[3840,2160]",
    "[8,1]",
    "[0,0]",
    "1",
    "false",
    "false",
    "false",
    "2.2",
    "8",
    "800",
    "4",
    "[24.6,-1]"
};

char LongValue[5][LONG_VALUE_LEN] = {
    "[1,0,0,0,0,1,0.0013888889,0]",
    "[0,0,1,0,1440,270,0.375,0]",
    "[0,-0.0004629630,0,0]",
    "[0,0,0,0,0,0,0,0,0]",
    "[0.10000000149011612,0.0700000002902323,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"
};
    
typedef struct{
    u8 conf_key;
    bool isLong;
    u8 index;
    u32 address;
    char conf_str[25];
}ConfInfotbl;

static ConfInfotbl config_info_tab[] = {
    { DisplayClass, false, 0,    PAGE_ADDR(CONF_PAGE1), "DisplayClass"},
    { DisplayID,    false, 1,    PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN, "DisplayID"},
    { DisplaySizeInMm, false, 2, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 2, "DisplaySizeInMm"},
    { DotPitchInMm,    false, 3, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 3, "DotPitchInMm"},
    { PanelResolution, false, 4, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 4, "PanelResolution"},
    { NumViews,        false, 5, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 5, "NumViews"},
    
    { InterlacingMatrix1, true, 0, PAGE_ADDR(CONF_PAGE2),"InterlacingMatrix1"},
    { InterlacingMatrix2, true, 1, PAGE_ADDR(CONF_PAGE2) + LONG_VALUE_LEN,"InterlacingMatrix2"},
    { InterlacingVector, true, 2, PAGE_ADDR(CONF_PAGE2) + LONG_VALUE_LEN * 2, "InterlacingVector"},

    
    { Slant,            false, 6, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 6, "Slant"},
    { AlignmentOffset,  false, 7, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 7, "AlignmentOffset"},
    { MirrorViewsX,     false, 8, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 8, "MirrorViewsX"},
    { MirrorViewsY,     false, 9, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 9, "MirrorViewsY"},
    { ReverseViews,     false, 10, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 10, "ReverseViews"},

    
    { ActCoefficientsX, true,  3, PAGE_ADDR(CONF_PAGE2) + LONG_VALUE_LEN * 3,"ActCoefficientsX"},
    { ActCoefficientsY, true,  4, PAGE_ADDR(CONF_PAGE2) + LONG_VALUE_LEN * 4,"ActCoefficientsY"},

    { Gamma,            false, 11,      PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 11, "Gamma"},
    { SystemDisparityPixels, false, 12, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 12, "SystemDisparityPixels"},
    { ConvergenceDistance,   false, 13, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 13, "ConvergenceDistance"},
    { CenterViewNumber,      false, 14, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 14, "CenterViewNumber"},
    { ViewBoxSize,           false, 15, PAGE_ADDR(CONF_PAGE1) + SHORT_VALUE_LEN * 15, "ViewBoxSize"}
};

void get_flash_setting(void);
void save_setting_in_flash(void);
void set_flash_tag(void);
int set_sys_brightness(uint8_t brightness);
int is_flash_new(void);


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
#ifdef  __DEBUG__
    usart_send_string(s);
    usart_send_string(CRLF);
#endif
}

void USART1_IRQHandler() {
    unsigned char received;

    if ((USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)) {
        received = USART_ReceiveData(USART1);
        if(cmd_received)
            return;

        if (received == CR) {
            g_usart_buf[usart_buf_length] = 0;
            //usart_send_newline();
            //command received
            cmd_received = 1;
            usart_buf_length = 0;
        } else if (received == LF) {
            // ignore
        } else {
            if (usart_buf_length >= COMMAND_MAX_LENGTH) {
                usart_send_newline();
                //printf(COMMAND_TOO_LONG);
                cmd_received = 1;
                cmd_too_long = 1;
                usart_buf_length = 0;
                return;

            }

            g_usart_buf[usart_buf_length++] = received;

        }
    }
}

u32 get_key_address(u8 conf_key)
{
    int i = 0;
    for(i = 0; i < (sizeof(config_info_tab) / sizeof(ConfInfotbl)); i++)
    {
        if(config_info_tab[i].conf_key == conf_key)
        {
            return config_info_tab[i].address;
        }
    }

    return 0xff;
}

u32 get_key_index(u8 conf_key)
{
    int i = 0;
    for(i = 0; i < (sizeof(config_info_tab) / sizeof(ConfInfotbl)); i++)
    {
        if(config_info_tab[i].conf_key == conf_key)
        {
            return config_info_tab[i].index;
        }
    }

    return 0xff;
}


u8 get_conf_id()
{
    int i = 0;
    DEBUG("g_tokens[1]:%s\n",g_tokens[1]);

    if(g_tokens[1] == NULL && cmd_id == getDisplayConfig)
    {
        return ALL_CONFIG;
    }

    for(i = 0; i < (sizeof(config_info_tab) / sizeof(ConfInfotbl)); i++)
    {
        char *p = config_info_tab[i].conf_str;

        if(!strcmp(g_tokens[1], p))
        {
            DEBUG("return conf_key:%d, p:%s\n",config_info_tab[i].conf_key,config_info_tab[i].conf_str);
            return config_info_tab[i].conf_key;
        }
    }

    return 0xff;
}

u32 isLongValue(u8 conf_key)
{
    int i = 0;
    for(i = 0; i < (sizeof(config_info_tab) / sizeof(ConfInfotbl)); i++)
    {
        if(config_info_tab[i].conf_key == conf_key)
        {
            return config_info_tab[i].isLong;
        }
    }
    return false;
}

void FLASH_WriteByte(uint32_t addr , uint8_t *p , uint16_t Byte_Num)
{
    uint32_t HalfWord;
    Byte_Num = Byte_Num/2;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(addr);
    while(Byte_Num --)
    {
        HalfWord=*(p++);
        HalfWord|=*(p++)<<8;
        FLASH_ProgramHalfWord(addr, HalfWord);
        addr += 2;
    }
    FLASH_Lock();
}

void FLASH_ReadByte(uint32_t addr , uint8_t *p , uint16_t Byte_Num)
{
    while(Byte_Num--)
    {
       *(p++)=*((uint8_t*)addr++);
    }
}

void set_conf_to_flash()
{
    if(is_flash_new())
    {
        DEBUG("flash new, write flash.\n");
        FLASH_WriteByte(PAGE_ADDR(CONF_DEFAULT_PAGE1), (uint8_t *)ShortValue, 512);
        FLASH_WriteByte(PAGE_ADDR(CONF_DEFAULT_PAGE2), (uint8_t *)LongValue, 640);
        FLASH_WriteByte(PAGE_ADDR(CONF_PAGE1), (uint8_t *)ShortValue, 512);
        FLASH_WriteByte(PAGE_ADDR(CONF_PAGE2), (uint8_t *)LongValue, 640);
    }
}

void delay(u16 num)
{
  u16 i,j;
  for(i=0;i<num;i++)
    for(j=0;j<0x800;j++);
}

int init_chip_8556(uint8_t chip_id)
{
    uint8_t tData[3];
    int i=0, ending_flag=0;
    uint8_t read_data;
    DEBUG("chipid:%d\r\n",chip_id);
    while (ending_flag == 0) 
    {
        if(chip_id == 0)
        {
            if (i2c_init_table[i][0] == 0xff) {    //special mark
                    if (i2c_init_table[i][1] == 0xff) { //ending flag
                            ending_flag = 1;
                    }
            }
            else {
                tData[0]=i2c_init_table[i][0];
                tData[1]=i2c_init_table[i][1];
                ee_WRITE_BYTES(tData[0], &tData[1], 1);
                delay(400);
                ee_READ_BYTES(tData[0], &read_data, 1);

                if(tData[1] == read_data)
                {
                    DEBUG("address:0x%x, value:0x%x,set successfully\r\n",tData[0],read_data);
                }
                else
                {
                    DEBUG("address:0x%x, value:0x%x,set fail\r\n",tData[0],read_data);
                    return -1;
                }
            }
            i++;
        }
        else
        {
            if (i2c1_init_table[i][0] == 0xff) {    //special mark
                    if (i2c1_init_table[i][1] == 0xff) { //ending flag
                            ending_flag = 1;
                    }
            }
            else {
                tData[0]=i2c1_init_table[i][0];
                tData[1]=i2c1_init_table[i][1];
                ee_1_WRITE_BYTES(tData[0], &tData[1], 1);
                delay(400);
                ee_1_READ_BYTES(tData[0], &read_data, 1);

                if(tData[1] == read_data)
                {
                    DEBUG("address:0x%x, value:0x%x,set successfully\r\n",tData[0],read_data);
                }
                else
                {
                    DEBUG("address:0x%x, value:0x%x,set fail\r\n",tData[0],read_data);
                    return -1;
                }
            }
            i++;

        }
    }
    return 0;
}

int help_info()
{
    if(cmd_key_num == 1)
    {
        printf("\"Commands\" : \r\n%s,\r\n",cmd_help_str);
        return 0;
    }
    else
    {
        printf(WRONG_FORMAT);
        return -1;
    }
}

int dump_chip_8556(uint8_t chip_id)
{
    uint8_t tData[3];
    int i=0, ending_flag=0;
    uint8_t read_data = 0;
    printf("\"8556 chipid %d\" : \r\n[", chip_id);
    //printf("address          value\r\n");
    while (ending_flag == 0) 
    {
        if (i2c_init_table[i][0] == 0xff) {    //special mark
            if (i2c_init_table[i][1] == 0xff) { //ending flag
                ending_flag = 1;
                printf("{\"address\" : %d ,\"val\" : %d}\r\n",0xaf,0xff);
            }
        }
        else {
            tData[0]=i2c_init_table[i][0];
            tData[1]=i2c_init_table[i][1];
            if(chip_id == 0)
            {
                ee_READ_BYTES(tData[0], &read_data, 1);
            }
            else
            {
                ee_1_READ_BYTES(tData[0], &read_data, 1);
            }

            printf("{\"address\" : %d ,\"val\" : %d},\r\n",tData[0],read_data);
        }
        i++;
    }
    printf("], \r\n");

    return 0;
}

int i2c_read_8556(uint8_t chip_id)
{
    if(cmd_key_num != 2)
    {
        return -1;
    }
    char *str = NULL;
    
    uint8_t read_data = 0;
    int address_input = strtol(g_tokens[1],&str,16);
    
    if(address_input > 255 || address_input < 0 || (*str) != '\0')
    {
        usart_send_line("[ERROR]address can only be a 8 bit HEX num!");
        return -2;
    }
    
    uint8_t address = (uint8_t)address_input;
    if(chip_id == 0)
    {
        ee_READ_BYTES(address, &read_data, 1);
    }
    else
    {
        ee_1_READ_BYTES(address, &read_data, 1);
    }
    printf("\"8556 chipid %d\" : {", chip_id);
    //printf("address          value\r\n");
    printf("\"0x%x\" : \"0x%x\"",address,read_data);
    printf("}, \r\n");
    
    return 0;
}


int i2c_write_8556(uint8_t chip_id)
{
    if(cmd_key_num != 3)
    {
        usart_send_line(WRONG_FORMAT);
        return -1;
    }
    char *str = NULL;
    
    int address_input = strtol(g_tokens[1],&str,16);

    if(address_input > 255 || address_input < 0 || (*str) != '\0')
    {
        usart_send_line("[ERROR]address can only be a uint8 HEX num!");
        return -2;
    }
    int value_input = strtol(g_tokens[2],&str,16);
    if(value_input > 255 || value_input < 0 || (*str) != '\0')
    {
        usart_send_line("[ERROR]value can only be a uint8 HEX num!");
        return -2;
    }

    uint8_t read_data = 0;
    uint8_t address = (uint8_t)address_input;
    uint8_t value = (uint8_t)value_input;

    if(chip_id == 0)
    {
        ee_WRITE_BYTES(address, &value, 1);
        ee_READ_BYTES(address, &read_data, 1);
    }
    else
    {
        ee_1_WRITE_BYTES(address, &value, 1);
        ee_1_READ_BYTES(address, &read_data, 1);
    }

    if(value == read_data)
    {
        printf("\"8556 chipid %d\" : {", chip_id);
        //printf("address          value\r\n");
        printf("\"0x%x\" : \"0x%x\"",address,read_data);
        printf("}, \r\n");
    }
    else
    {
        //printf("The result is not correct!");
        return -3;
    }
    
    return 0;
}

int is_flash_new()
{
    //在page55中存放标志位，根据标志位判断是否设置默认值  
    uint32_t *read_p = (uint32_t *)PAGE_ADDR(55);
    if(*read_p == 0x5a5b5c)
        return 0;
    else
        return 1;
}

void set_flash_tag()
{
    FLASH_Unlock();
    
    FLASH_ErasePage(PAGE_ADDR(55));
    
    uint32_t write_addr = PAGE_ADDR(55);
    uint32_t wrtie_data = 0x5a5b5c;
    
    FLASH_ProgramWord(write_addr,wrtie_data);

    //write board info into flash
    write_addr += 4;
    uint32_t *wrtie_data_p = (uint32_t *)board_info;
    for(int i = 0;i < 32;i++)
    {
        //write data into flash
        FLASH_ProgramWord(write_addr,*wrtie_data_p);
        write_addr += 4;
        wrtie_data_p++;
    }
    
    FLASH_Lock();

}

void clear_flash_tag()
{
    FLASH_Unlock();
    
    FLASH_ErasePage(PAGE_ADDR(55));
    
    FLASH_Lock();
}

void brightness_init()
{
    if(is_flash_new())
    {
        //printf("flash is new, set default values.\r\n");
        set_flash_tag();

        FLASH_Unlock();
        FLASH_ErasePage(PAGE_ADDR(54));

        uint32_t write_addr = PAGE_ADDR(54);

        //set to default
        g_setting[mode_index] = 2;
        g_setting[brightness_index] = 102;    
        g_setting[mode2ratio2d_index] = 100;  //1.00
        g_setting[mode2ratio3d_index] = 15;   //0.15
        g_setting[mode3ratio2d_index] = 0;    //0
        g_setting[mode3ratio3d_index] = 100;  //1.00
        g_setting[ctrlmode2d_index] = I2C_CTRL;

        uint32_t *wrtie_data_p = g_setting;
        for(int i = 0;i < STORE_LEN;i++)
        {
            //write data into flash
            FLASH_ProgramWord(write_addr,*wrtie_data_p);
            write_addr += 4;
            wrtie_data_p++;

        }
        
        set_sys_brightness(g_setting[brightness_index]);
    }
    else
    {
        get_flash_setting();
        if(g_setting[ctrlmode2d_index] == PWM_CTRL)
        {
            //pwm only mode
            uint8_t value = 1;  //0x01 pwm only set brightness,0x5 I2c set brightness
            uint8_t address = 0x01;
            uint8_t read_data = 0;
            uint8_t read_data1 = 0;
            ee_WRITE_BYTES(address, &value, 1);
            ee_READ_BYTES(address, &read_data, 1);
            if(value == read_data)
            {
                DEBUG("use pwm to control 8556 chip 0.\r\n");
                
            }
            else
            {
                DEBUG("set pwm to control 8556 chip 0 fail.\r\n");
            }   
            
            ee_1_WRITE_BYTES(address, &value, 1);
            ee_1_READ_BYTES(address, &read_data1, 1);
            if(value == read_data1)
            {
                DEBUG("use pwm to control 8556 chip 1.\r\n");
                
            }
            else
            {
                DEBUG("set pwm to control 8556 chip 1 fail.\r\n");
            }
        }
        set_sys_brightness(g_setting[brightness_index]);
    }

}

void init()
{
    int ret = 0;
    delay(500);
    USART_Config();
    ADVANCE_TIM_GPIO_Config();
    //ADVANCE_2D_PWM_Config(2048);

    DEBUG("init...\r\n");
    DEBUG ("pwm configuration is finished.\r\n");
    i2c_GPIO_Config();
    set_conf_to_flash();
    delay(1000);

    if(ee_CHECK_DEVICE(WRITE_DIR_8556) == 0)
    {
        DEBUG("8556 0 has been detected.\r\n");
    }
    else
    {
        DEBUG("8556 0 has not been detected\r\n");
    }
    
    if(ee_1_CHECK_DEVICE(WRITE_DIR_8556) == 0)
    {
        DEBUG("8556 1 has been detected.\r\n");
    }
    else
    {
        DEBUG("8556 1 has not been detected\r\n");
    }

#if 0   
step1:
    ret = init_chip_8556(0);
    if(ret != 0)
    {
        delay(100);
        goto step1;
    }
    
step2:
    ret = init_chip_8556(1);
    if(ret != 0)
    {
        delay(100);
        goto step2;
    }
#endif  
    delay(500);

    brightness_init();

}



int set_sys_brightness(uint8_t brightness)
{
    //set 2d brightness,use i2c to set brightness register
    uint8_t blu_brightness_2d = 0;
    if(g_setting[mode_index] == 2)
    {
        blu_brightness_2d = (double)brightness * ((double)g_setting[mode2ratio2d_index] / 100.0);
    }
    else
    {
        blu_brightness_2d = (double)brightness * ((double)g_setting[mode3ratio2d_index] / 100.0);
    }

    //set 3d brightness, use pwm duty to set brightness
    int blu_brightness_3d = 0;
    if(g_setting[mode_index] == 2)
    {
        blu_brightness_3d = (double)brightness * ((double)g_setting[mode2ratio3d_index] / 100.0);
    }
    else
    {
        blu_brightness_3d = (double)brightness * ((double)g_setting[mode3ratio3d_index] / 100.0);
    }

    if(blu_brightness_3d > 255)
    {
        blu_brightness_3d = 255;
    }
    
    printf("\"system brightness\" : %d,\r\n",brightness);

    printf("\"brightness_2D\" : %d, \r\n \"brightness_3D\" : %d,\r\n",blu_brightness_2d,blu_brightness_3d);

    int duty_3d = (blu_brightness_3d / 255.0) * DUTY_MAX_3D;
    
    uint8_t ibrightness_read = 0;
    uint8_t ibrightness_read1 = 0;

    if(g_setting[ctrlmode2d_index] == I2C_CTRL)
    {
        ADVANCE_3D_PWM_Config(duty_3d);
        printf("\"duty_3d\" : \"%d/4096\",\r\n",duty_3d);
        ee_WRITE_BYTES(BRIGHTNESS_CONTROL,&blu_brightness_2d,1);
        
        ee_READ_BYTES(BRIGHTNESS_CONTROL,&ibrightness_read,1);

        delay(50);
        
        ee_1_WRITE_BYTES(BRIGHTNESS_CONTROL,&blu_brightness_2d,1);
        
        ee_1_READ_BYTES(BRIGHTNESS_CONTROL,&ibrightness_read1,1);
        if(blu_brightness_2d == ibrightness_read && blu_brightness_2d == ibrightness_read1)
        {
            usart_send_line("2d i2c set brightness successfully!");
            return 0;
        }
        else
        { 
            usart_send_line("2d set i2c set brightness failed!");
            return -1;
        }
        //ADVANCE_2D_PWM_Config(2048);

    }
    else
    {
        int duty_2d = (blu_brightness_2d / 255.0) * 4096;
        ADVANCE_2D_PWM_Config(duty_2d);
        ADVANCE_3D_PWM_Config(duty_3d);
        printf("\"duty_2d\" : \"%d/4096\",\r\n \"duty_3d\" : \"%d/4096\",\r\n",duty_2d,duty_3d);
        return 0;
    }
    
}

int mode_switch()
{
    int ret = -1;
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        printf(WRONG_FORMAT);
        return -1;
    }

    int mode = atoi(g_tokens[1]);
    if(mode != 2 && mode != 3)
    {
        usart_send_line("[ERROR]mode must be 2 or 3!");
        printf(VALUE_ERROR);
        return -1;
    }

    g_setting[mode_index] = mode;
    save_setting_in_flash();
    ret = set_sys_brightness(g_setting[brightness_index]);
    return ret;
}

void get_flash_setting()
{
    uint32_t *read_p = (uint32_t *)PAGE_ADDR(55);
    //printf("tag:%d\r\n",*read_p);

    read_p = (uint32_t *)PAGE_ADDR(54);
    for(int i = 0; i < STORE_LEN; i++)
    {
        g_setting[i] = read_p[i];
        //printf("g_setting[%d]:%d\r\n",i,g_setting[i]);
    }
}

void save_setting_in_flash()
{
    FLASH_Unlock();

    FLASH_ErasePage(PAGE_ADDR(54));

    uint32_t write_addr = PAGE_ADDR(54);
    uint32_t *wrtie_data_p = g_setting;
    uint32_t *read_p;
    int i = 0;

    for(i = 0;i < STORE_LEN;i++)
    {
        //write data into flash
        FLASH_ProgramWord(write_addr,*wrtie_data_p);
        write_addr += 4;
        wrtie_data_p++;
    }

    read_p = (uint32_t *)PAGE_ADDR(54);
    for(i = 0; i < STORE_LEN; i++)
    {
        //printf("\r\nwrite[%d] = %d, read[%d] = %d",i,g_setting[i],i,read_p[i]);
        if(g_setting[i] != read_p[i])
        {
            break;
        }
    }
    
    if(i == STORE_LEN)
    {
        usart_send_line("Write Data to flash success.\r\n");
    }
    
    FLASH_Lock();

}

int mode_set_ratio()
{
    if(cmd_key_num != 4)
    {
        printf(WRONG_FORMAT);
        return -1;
    }

    int mode = atoi(g_tokens[1]);
    char *str1,*str2 = NULL;
    double ratio_2d = strtod(g_tokens[2],&str1);
    double ratio_3d = strtod(g_tokens[3],&str2);

    if((mode != 2 && mode != 3) || (*str1 != '\0') || (*str2 != '\0'))
    {
        usart_send_line("mode or ratio illegal");
        printf(VALUE_ERROR);
        return -1;
    }

    if(strncmp(g_tokens[2],"0",strlen("0")) && ratio_2d == 0)
    {
        printf(VALUE_ERROR);
        return -1;
    }

    if(strncmp(g_tokens[3],"0",strlen("0")) && ratio_3d == 0)
    {
        printf(VALUE_ERROR);
        return -1;
    }

    if(ratio_2d > 1.00 || ratio_2d < 0.00 || ratio_3d > 3.00 || ratio_3d < 0.00)
    {
        usart_send_line("ratio error,ratio_2d:0.00~1.00 ,ratio_3d:0.00~3.00");
        printf(VALUE_ERROR);
        return -1;
    }

    if(mode == 2)
    {
        g_setting[mode2ratio2d_index] = (int)(ratio_2d * 100);
        g_setting[mode2ratio3d_index] = (int)(ratio_3d * 100);
    }
    else
    {
        g_setting[mode3ratio2d_index] = (int)(ratio_2d * 100);
        g_setting[mode3ratio3d_index] = (int)(ratio_3d * 100);
    }

    //save in stm32's flash
    save_setting_in_flash();
    return set_sys_brightness(g_setting[brightness_index]);
    
}

int display_boardinfo()
{
    if(cmd_key_num != 1)
    {
        printf(WRONG_FORMAT);
        return -1;
    }
    
    uint32_t read_addr = PAGE_ADDR(55) + 4;
    char *p = (char *)read_addr;
    char info[128] = "";
    for(int i = 0; i < 128; i++)
    {
        info[i] = *p;
        p++;
    }
    
    printf("\"BOARD INFO\" : \"%s\" , \r\n",info);
    return 0;
}

int setting_get()
{
    if(cmd_key_num != 1)
    {
        printf(WRONG_FORMAT);
        return -1;
    }

    get_flash_setting();
    int brightness = g_setting[brightness_index];
    int mode = g_setting[mode_index];
    double mode2_ratio_2d = g_setting[mode2ratio2d_index] / 100.0;
    double mode2_ratio_3d = g_setting[mode2ratio3d_index] / 100.0;
    double mode3_ratio_2d = g_setting[mode3ratio2d_index] / 100.0;
    double mode3_ratio_3d = g_setting[mode3ratio3d_index] / 100.0;
    
    double current_2d = (mode == 2) ? ((double)brightness * mode2_ratio_2d /255.0 * 25.00) : ((double)brightness * mode3_ratio_2d /255.0 * 25.00);
    double current_3d = (mode == 2) ? ((double)brightness * mode2_ratio_3d /255.0 * 30.00) : ((double)brightness * mode3_ratio_3d /255.0 * 30.00);
    usart_send_line("Display current system configurations:\r\n");
    if(g_setting[ctrlmode2d_index] == I2C_CTRL)
    {
        printf("\"brightness\" : %d , \r\n\"2d control mode\" : \"i2c\",\r\n",brightness);
    }
    else
    {
        printf("\"brightness\" : %d , \r\n\"2d control mode\" : \"pwm\",\r\n",brightness);
    }

    //usart_send_line("+------+----------+----------+------------+------------+\r\n");
    //usart_send_line("| mode | 2D_ratio | 3D_ratio | 2D_current | 3D_current |\r\n");
    if(mode == 2)
    {
        printf("\"mode\" : %d , \r\n\"2D_ratio\" : %.2f, \r\n\"3D_ratio\" : %.2f, \r\n\"2D_current\": %.2f, \r\n\"3D_current\" : %.2f,\r\n",2,mode2_ratio_2d,mode2_ratio_3d,current_2d,current_3d);
        //usart_send_line("+------+----------+----------+------------+------------+\r\n");
        //usart_send_line("|%-6d|%-10.2f|%-10.2f|%-12.2f|%-12.2f|\r\n",2,mode2_ratio_2d,mode2_ratio_3d,current_2d,current_3d);
    }
    else
    {
        printf("\"mode\" : %d , \r\n\"2D_ratio\" : %.2f, \r\n\"3D_ratio\" : %.2f, \r\n\"2D_current\": %.2f, \r\n\"3D_current\" : %.2f,\r\n",3,mode3_ratio_2d,mode3_ratio_3d,current_2d,current_3d);
        //usart_send_line("+------+----------+----------+------------+------------+\r\n");
        //usart_send_line("|%-6d|%-10.2f|%-10.2f|%-12.2f|%-12.2f|\r\n",3,mode3_ratio_2d,mode3_ratio_3d,current_2d,current_3d);
    }

    //usart_send_line("+------+----------+----------+------------+------------+\r\n\r\n");
    return 0;
}

int set_2d_current()
{
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        printf(WRONG_FORMAT);
        return -1;
    }  

    char *str = NULL;
    double current = strtod(g_tokens[1],&str);

    if((*str != '\0'))
    {
        usart_send_line("current illegal");
        printf(VALUE_ERROR);
        return -1;
    }

    if(current > 25.00 || current < 0.00)
    {
        usart_send_line("current error,current:0.00~25.00");
        printf(VALUE_ERROR);
        return -1;
    }
    
    int brightness = current / 25.00 * 255;
    uint8_t ibrightness_read = 0;
    uint8_t ibrightness_read1 = 0;

    if(g_setting[ctrlmode2d_index] == I2C_CTRL)
    {
        uint8_t brightness_set = (uint8_t)brightness;
        ee_WRITE_BYTES(BRIGHTNESS_CONTROL,&brightness_set,1);
    
        ee_READ_BYTES(BRIGHTNESS_CONTROL,&ibrightness_read,1);

        ee_1_WRITE_BYTES(BRIGHTNESS_CONTROL,&brightness_set,1);
    
        ee_1_READ_BYTES(BRIGHTNESS_CONTROL,&ibrightness_read1,1);
    
        if(brightness == ibrightness_read && brightness == ibrightness_read1)
        {
            //printf("2d set successfully! This setting is temporary. Now temp brightness is %d\r\n",ibrightness_read);
            return 0;
        }
        else
        {
            usart_send_line("2d set failed!");
            return -1;
        }
    }
    else
    {
        int duty_2d = (current / 25.00) * 4096; 
        ADVANCE_2D_PWM_Config(duty_2d); 
        //printf("duty_2d :%d/4096\r\n",duty_2d);
        return 0;
    }
    
}

int set_3d_current()
{
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        printf(WRONG_FORMAT);
        return -1;
    }  

    char *str = NULL;
    double current = strtod(g_tokens[1],&str);

    if((*str != '\0'))
    {
        usart_send_line("current illegal\r\n");
        printf(VALUE_ERROR);
        return -1;
    }

    if(current > 30.00 || current < 0.00)
    {
        usart_send_line("current error,current:0.00~30.00\r\n");
        printf(VALUE_ERROR);
        return -1;
    }

    int duty_3d = (current / 30.00) * DUTY_MAX_3D;
    ADVANCE_3D_PWM_Config(duty_3d); 
    //printf("duty_3d :%d/4096\r\n",duty_3d);
    return 0;

}

int set_2dctrmode()
{
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        printf(WRONG_FORMAT);
        return -1;
    }

    int mode = atoi(g_tokens[1]);

    if(strncmp(g_tokens[1],"0",strlen("0")) && mode == 0)
    {
        printf(VALUE_ERROR);
        return -1;
    }
    
    if(mode != 0 && mode != 1)
    {
        usart_send_line("[ERROR]mode should be 0(i2c) or 1(pwm).");
        printf(VALUE_ERROR);
        return -1;
    }

    uint8_t read_data = 0;
    uint8_t read_data1 = 0;
    uint8_t address = 0x01;
    uint8_t value = 0;

    g_setting[ctrlmode2d_index] = mode;
    save_setting_in_flash();
    if(mode == I2C_CTRL)
    {
        //i2c only mode
        value = 5;
        ee_WRITE_BYTES(address, &value, 1);
        ee_READ_BYTES(address, &read_data, 1);

        ee_1_WRITE_BYTES(address, &value, 1);
        ee_1_READ_BYTES(address, &read_data1, 1);
        if(value == read_data && value == read_data1)
        {
            usart_send_line("use i2c to control 8556.\r\n");
            return 0;
        }
        else
        {
            usart_send_line("set i2c to control 8556 fail.\r\n");
            return -1;
        }
    }
    else
    {
        //pwm only mode
        value = 1;
        ee_WRITE_BYTES(address, &value, 1);
        ee_READ_BYTES(address, &read_data, 1);

        ee_1_WRITE_BYTES(address, &value, 1);
        ee_1_READ_BYTES(address, &read_data1, 1);
        if(value == read_data && value == read_data1)
        {
            int blu_brightness_2d = 0;
            if(g_setting[mode_index] == 2)
            {
                blu_brightness_2d = (double)g_setting[brightness_index] * ((double)g_setting[mode2ratio2d_index] / 100.0);
            }
            else
            {
                blu_brightness_2d = (double)g_setting[brightness_index] * ((double)g_setting[mode3ratio2d_index] / 100.0);
            }
            usart_send_line("use pwm to control 8556.\r\n");
            int duty_2d = (blu_brightness_2d / 255.0) * 4096;
            ADVANCE_2D_PWM_Config(duty_2d);
            return 0;
        }
        else
        {
            usart_send_line("set pwm to control 8556 fail.\r\n");
            return -1;
        }
    }
    
}

int set_pwm()
{
    if(cmd_key_num != 3)
    {
        usart_send_line(WRONG_FORMAT);
        printf(WRONG_FORMAT);
        return -1;
    }  

    int mode = atoi(g_tokens[1]);
    int duty = atoi(g_tokens[2]);

    if((mode != 2 && mode != 3))
    {
        usart_send_line("mode illegal");
        printf(VALUE_ERROR);
        return -1;
    }

    if(strncmp(g_tokens[2],"0",strlen("0") && duty == 0))
    {
        printf(VALUE_ERROR);
        return -1;
    }

    if(duty < 0 || duty > 4096)
    {
        usart_send_line("duty error,duty can only be 0~4096");
        printf(VALUE_ERROR);
        return -1;
    }

    if(mode == 2)
    {
        ADVANCE_2D_PWM_Config(duty);  
    }
    else
    {
        ADVANCE_3D_PWM_Config(duty);  
    }
    return 0;
}

void gpio_read()
{
    //TODO
}

void gpio_readpin()
{
    //TODO
}

void gpio_setpin()
{
    //TODO
}

void gpio_clrpin()
{
    //TODO
}

int display_config()
{
    //FLASH_ReadByte(PAGE_ADDR(CONF_PAGE1), (uint8_t *)ShortValue, 512);
    int conf_id = 0;
    u32 address = 0;
    char short_data[SHORT_VALUE_LEN] = {0};
    char long_data[LONG_VALUE_LEN] = {0};

    conf_id = get_conf_id();

	if(cmd_key_num != 2 && cmd_key_num != 1)
    {
        usart_send_line(WRONG_FORMAT);
        printf(WRONG_FORMAT);
        return -1;
    }  
	
    if(conf_id == 0xff)
    {
        return -1;
    }

    if(conf_id == ALL_CONFIG)
    {
        int i = 0;
        for(i = 0; i < (sizeof(config_info_tab) / sizeof(ConfInfotbl)); i++)
        {
            if(isLongValue(config_info_tab[i].conf_key))
            {
                
                address = config_info_tab[i].address;
                FLASH_ReadByte(address, (uint8_t *)long_data, LONG_VALUE_LEN);
                printf("\"%s\": \"%s\",\n",config_info_tab[i].conf_str, long_data);
            }
            else
            {
                address = config_info_tab[i].address;
                FLASH_ReadByte(address, (uint8_t *)short_data, SHORT_VALUE_LEN);
                printf("\"%s\": \"%s\",\n",config_info_tab[i].conf_str, short_data);
            }
        }
        
        return 0;
    }

    if(isLongValue(conf_id))
    {
        address = get_key_address(conf_id);
        DEBUG("conf_id:%x, address:%x\n", conf_id, address);
        FLASH_ReadByte(address, (uint8_t *)long_data, LONG_VALUE_LEN);
        printf("\"%s\": \"%s\",\n",g_tokens[1], long_data);
        
        return 0;
    }
    else
    {
        address = get_key_address(conf_id);
        DEBUG("conf_id:%x, address:%x\n", conf_id, address);
        FLASH_ReadByte(address, (uint8_t *)short_data, SHORT_VALUE_LEN);
        printf("\"%s\": \"%s\",\n",g_tokens[1], short_data);
        
        return 0;
    }

    return 0;
    //printf("%s",ShortValue[0]);
}

int set_config()
{
	if(cmd_key_num != 3)
    {
        usart_send_line(WRONG_FORMAT);
        printf(WRONG_FORMAT);
        return -1;
    }  

    u8 conf_id = get_conf_id();
    
    if(conf_id == 0xff)
    {
        return -1;
    }

    u8 conf_index = get_key_index(conf_id);

    if(strlen(g_tokens[2]) > LONG_VALUE_LEN)
    {
        return -1;
    }

    if(isLongValue(conf_id))
    {
        char *p = (char *)LongValue[conf_index];
        FLASH_ReadByte(PAGE_ADDR(CONF_PAGE2), (uint8_t *)LongValue, 640);
        memset(&LongValue[conf_index], 0, LONG_VALUE_LEN);
        strncpy(p, g_tokens[2], LONG_VALUE_LEN);
        FLASH_WriteByte(PAGE_ADDR(CONF_PAGE2), (uint8_t *)LongValue, 640);
		DEBUG("write conf_id:%d to CONF_PAGE2", conf_id);

    }
    else
    {
        char *p = (char *)ShortValue[conf_index];
        FLASH_ReadByte(PAGE_ADDR(CONF_PAGE1), (uint8_t *)ShortValue, 512);  
        memset(&ShortValue[conf_index], 0, SHORT_VALUE_LEN);
        strncpy(p, g_tokens[2], SHORT_VALUE_LEN);
        FLASH_WriteByte(PAGE_ADDR(CONF_PAGE1), (uint8_t *)ShortValue, 512);
		DEBUG("write conf_id:%d to CONF_PAGE1", conf_id);
    }
    
    return 0;
}

void factoryreset()
{

    FLASH_ReadByte(PAGE_ADDR(CONF_DEFAULT_PAGE2), (uint8_t *)LongValue, 640);
    FLASH_WriteByte(PAGE_ADDR(CONF_PAGE2), (uint8_t *)LongValue, 640);
	DEBUG("write CONF_DEFAULT_PAGE2 to CONF_PAGE2");

    FLASH_ReadByte(PAGE_ADDR(CONF_DEFAULT_PAGE1), (uint8_t *)ShortValue, 512);
    FLASH_WriteByte(PAGE_ADDR(CONF_PAGE1), (uint8_t *)ShortValue, 512);
	DEBUG("write CONF_DEFAULT_PAGE1 to CONF_PAGE1");
	
    clear_flash_tag();
    init();
    //printf("Reset to Factory Defaults.\r\n");
}

void clear_cmd_info(void)
{
    cmd_id = CMD_NULL;
    cmd_key_num = 0;
    memset(g_usart_buf,0,COMMAND_MAX_LENGTH);
}

void result_suffix(int ret)
{
    if(ret == 0)
    {
        printf("\"result\" : \"success\" }\r\n");
    }
    else
    {
        printf("\"result\" : \"fail\" }\r\n");
    }
}

void command_parse() {

    int i = 0;
    printf("\r\n{ \"CMD\" : \"%s\" , \r\n",g_usart_buf);
    memset(g_tokens,0,5 * sizeof(char *));
    
    for (char *p = strtok(g_usart_buf," "); p != NULL; p = strtok(NULL, " "))
    {
        g_tokens[i] = p;
        DEBUG("CMD KEY:%s\t",g_tokens[i]);
        i++;
        cmd_key_num = i;
    }
    //printf("\n");
    if(!strcmp(g_tokens[0],"HELP"))
    {
        cmd_id = HELP;
    }
    else if(!strcmp(g_tokens[0],"INFO"))
    {
        cmd_id = INFO;
    }
    else if(!strcmp(g_tokens[0],"DUMP"))
    {
        cmd_id = DUMP;
    }
    else if(!strcmp(g_tokens[0],"I2CREAD"))
    {
        cmd_id = I2CREAD;
    }
    else if(!strcmp(g_tokens[0],"I2CWRITE"))
    {
        cmd_id = I2CWRITE;
    }
    else if(!strcmp(g_tokens[0],"BLSETBRIGHTNESS"))
    {
        cmd_id = BLSETBRIGHTNESS;
    }
    else if(!strcmp(g_tokens[0],"BLSETRATIOS"))
    {
        cmd_id = BLSETRATIOS;
    }
    else if(!strcmp(g_tokens[0],"BLSWITCH"))
    {
        cmd_id = BLSWITCH;
    }
    else if(!strcmp(g_tokens[0],"SET2DCTRLMODE"))
    {
        cmd_id = SET2DCTRLMODE;
    }
    else if(!strcmp(g_tokens[0],"BLSETPWM"))
    {
        cmd_id = BLSETPWM;
    }
    else if(!strcmp(g_tokens[0],"BLSETTINGSGET"))
    {
        cmd_id = BLSETTINGSGET;
    }
    else if(!strcmp(g_tokens[0],"BLSET2DCURRENT"))
    {
        cmd_id = BLSET2DCURRENT;
    }
    else if(!strcmp(g_tokens[0],"BLSET3DCURRENT"))
    {
        cmd_id = BLSET3DCURRENT;
    }
    else if(!strcmp(g_tokens[0],"POKE32"))
    {
        cmd_id = POKE32;
    }
    else if(!strcmp(g_tokens[0],"POKE16"))
    {
        cmd_id = POKE16;
    }
    else if(!strcmp(g_tokens[0],"POKE8"))
    {
        cmd_id = POKE8;
    }
    else if(!strcmp(g_tokens[0],"PEEK32"))
    {
        cmd_id = PEEK32;
    }
    else if(!strcmp(g_tokens[0],"PEEK16"))
    {
        cmd_id = PEEK16;
    }
    else if(!strcmp(g_tokens[0],"PEEK8"))
    {
        cmd_id = PEEK8;
    }
    else if(!strcmp(g_tokens[0],"GPIOREAD"))
    {
        cmd_id = GPIOREAD;
    }
    else if(!strcmp(g_tokens[0],"GPIOREADPIN"))
    {
        cmd_id = GPIOREADPIN;
    }
    else if(!strcmp(g_tokens[0],"GPIOSETPIN"))
    {
        cmd_id = GPIOSETPIN;
    }
    else if(!strcmp(g_tokens[0],"GPIOCLRPIN"))
    {
        cmd_id = GPIOCLRPIN;
    }
    else if(!strcmp(g_tokens[0],"BLFACTORYRESET"))
    {
        cmd_id = BLFACTORYRESET;
    }
    else if(!strcmp(g_tokens[0],"setDisplayConfig"))
    {
        cmd_id = setDisplayConfig;
    }
    else if(!strcmp(g_tokens[0],"getDisplayConfig"))
    {
        cmd_id = getDisplayConfig;
    }
    else
    {
        cmd_id = UNKNOWCMD;
    }
}


void handle_command()
{
    int ret = -1;
    switch(cmd_id){
        case HELP:
        {
            ret = help_info();
            clear_cmd_info();
            result_suffix(ret);
            break;
        }      
        case DUMP: 
        {                
            if(cmd_key_num != 1)
            {
                printf(WRONG_FORMAT);
                ret = -1;
                clear_cmd_info();
                result_suffix(ret);
                break;
            }
            dump_chip_8556(0);
            dump_chip_8556(1);
            clear_cmd_info();
            result_suffix(0);
            break;
        }
        case INFO:
        {                
            ret = display_boardinfo();
            clear_cmd_info();
            result_suffix(ret);
            break;
        }
        case I2CREAD:
        {
            int ret = i2c_read_8556(0);
            ret |= i2c_read_8556(1);
            if (ret == -1)
            {
                printf(WRONG_FORMAT);
            }
            else if(ret == -2)
            {
                printf(VALUE_ERROR);
            }

            clear_cmd_info();
            result_suffix(ret);
            break;
        }
        case I2CWRITE:
        {
            int ret = i2c_write_8556(0);
            ret |= i2c_write_8556(1);
            if (ret == -1)
            {
                printf(WRONG_FORMAT);
            }
            else if(ret == -2)
            {
                printf(VALUE_ERROR);
            }
            
            clear_cmd_info();
            result_suffix(ret);
            break;
        }
        case BLSWITCH:
        {
            ret = mode_switch();
            clear_cmd_info();
            result_suffix(ret);
            break;
        }
        case BLSETBRIGHTNESS:
        {
            int brightness = atoi(g_tokens[1]);

            if(cmd_key_num != 2)
            {
                printf(WRONG_FORMAT);
                ret = -1;
                result_suffix(ret);
                clear_cmd_info();
                break;
            }
            
            if(strncmp(g_tokens[1],"0",strlen(g_tokens[1])) && brightness == 0)
            {
                printf(VALUE_ERROR);
                ret = -1;
                result_suffix(ret);
                clear_cmd_info();
                break;
            }
            
            if( (brightness > 255 || brightness < 0))
            {
                usart_send_line("[ERROR]brightness value should be 0~255.\r\n");
                printf(VALUE_ERROR);
                ret = -1;
                result_suffix(ret);
                clear_cmd_info();
                break;
            }
            ret = set_sys_brightness(brightness);
            g_setting[brightness_index] = brightness;
            save_setting_in_flash();

            clear_cmd_info();
            result_suffix(ret);
            break;
        }
        case BLSETRATIOS:
        {
            ret = mode_set_ratio();
            clear_cmd_info();
            result_suffix(ret);
            break;
        }
        case BLSETTINGSGET:
        {
            ret = setting_get();
            clear_cmd_info(); 
            result_suffix(ret);
            break;
        }
        case BLSET2DCURRENT:
        {
            ret = set_2d_current();
            clear_cmd_info();  
            result_suffix(ret);
            break;  
        }
        case BLSET3DCURRENT:
        {
            ret = set_3d_current();
            clear_cmd_info();  
            result_suffix(ret);
            break;  
        }
        case BLSETPWM:
        {
            ret = set_pwm();
            clear_cmd_info();
            result_suffix(ret);
            break; 
        }
        case SET2DCTRLMODE:
        {
            ret = set_2dctrmode();
            clear_cmd_info(); 
            result_suffix(ret);
            break; 
        }
        case POKE32:
        {
            char *str1 = NULL;
            char *str2 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            int data_input = strtol(g_tokens[2],&str2,16);

            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 32 bit HEX num!");
                clear_cmd_info();
                break;
            }

            if((*str2) != '\0')
            {
                usart_send_line("[ERROR]data can only be a 32 bit HEX num!");
                clear_cmd_info();
                break;
            }

            uint32_t *p = (uint32_t *)address_input;
            *p = data_input;
            clear_cmd_info();
            result_suffix(0);
            break; 
        }
        case POKE16:
        {
            char *str1 = NULL;
            char *str2 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            int data_input = strtol(g_tokens[2],&str2,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 16 bit HEX num!");
                clear_cmd_info();
                break;
            }

            if((*str2) != '\0' || data_input < 0 || data_input > 65535)
            {
                usart_send_line("[ERROR]data can only be a 16 bit HEX num!");
                clear_cmd_info();
                break;
            }

            uint16_t *p = (uint16_t *)address_input;
            *p = data_input;

            clear_cmd_info();
            result_suffix(0);
            break; 
        }
        case POKE8:
        {
            char *str1 = NULL;
            char *str2 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            int data_input = strtol(g_tokens[2],&str2,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 8 bit HEX num!");
                break;
            }

            if((*str2) != '\0' || data_input < 0 || data_input > 255)
            {
                usart_send_line("[ERROR]data can only be a 8 bit HEX num!");
                break;
            }

            uint8_t *p = (uint8_t *)address_input;
            *p = data_input;

            clear_cmd_info(); 
            result_suffix(0);
            break; 
        }
        case PEEK32:
        {
            char *str1 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 32 bit HEX num!");
                break;
            }

            uint32_t *p = (uint32_t *)address_input;
            printf("address:0x%x, data:0x%x\r\n",address_input,*p);

            clear_cmd_info();
            result_suffix(0);
            break; 
        }
        case PEEK16:
        {
            char *str1 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 32 bit HEX num!");
                break;
            }

            uint16_t *p = (uint16_t *)address_input;
            printf("address:0x%x, data:0x%x\r\n",address_input,*p);

            clear_cmd_info(); 
            result_suffix(0);
            break; 
        }
        case PEEK8:
        {
            char *str1 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 32 bit HEX num!");
                break;
            }

            uint8_t *p = (uint8_t *)address_input;
            printf("address:0x%x, data:0x%x\r\n",address_input,*p);

            clear_cmd_info();
            result_suffix(0);
            break; 
        }
        case GPIOREAD:
        {
            gpio_read();
            clear_cmd_info();
            result_suffix(0);
            break;
        }
        case GPIOREADPIN:
        {
            gpio_readpin();
            clear_cmd_info();
            result_suffix(0);
            break;
        }
        case GPIOSETPIN:
        {
            gpio_setpin();
            clear_cmd_info();
            result_suffix(0);
            break;
        }
        case GPIOCLRPIN:
        {
            gpio_clrpin();
            clear_cmd_info();
            result_suffix(0);
            break; 
        }
        case setDisplayConfig:
        {
            ret = set_config();
            clear_cmd_info();
            result_suffix(ret);
            break;
        }
        case getDisplayConfig:
        {
            ret = display_config();
            clear_cmd_info();
            result_suffix(ret);
            break;
        }
        case BLFACTORYRESET:
        {
            if(cmd_key_num != 1)
            {
                printf(WRONG_FORMAT);
                result_suffix(-1);
                clear_cmd_info();
                break;
            }
            factoryreset();
            result_suffix(0);
            clear_cmd_info();

            break; 
        }
        case CMD_NULL:
            break;
        default:
            printf(UNKNOWN_COMMAND);
            result_suffix(1);
            break;
    }
}   

int main(void)
{
    init();

    while(1)
    {
        if(cmd_received)
        {
            if(cmd_too_long)
            {
                printf(COMMAND_TOO_LONG);
                delay(100);
                cmd_too_long = 0;
            }
            else
            {
                command_parse();
                handle_command();
            }
            cmd_received = 0;
        }
    }
}



