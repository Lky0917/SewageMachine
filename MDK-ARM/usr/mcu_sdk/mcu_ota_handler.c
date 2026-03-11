/****************************************Copyright (c)*************************
**                               Copyright (C) 2014-2020, Tuya Inc., All Rights Reserved
**
**                                 http://www.tuya.com
**
**--------------Revision record---------------------------------------------------
** version: v1.0
** date : may 3, 2017 
description: Initial version
**

**version::v2.0
** date: March 23, 2020
** description: 
1. Added module unbinding interface support, command code 0x09.
2.Add rf RF test interface support, command code 0x0e.
3.Add record-based data reporting interface support,command code 0xe0.
4. Added access to real-time time API support,command code 0xe1.
5. Added support for modifying sleep mode state bluetooth broadcast interval,command code 0xe2.
6. Added support for turning off system clock,command code 0xe4.
7. Increase low power consumption to enable support,commadn code 0xe5.
8. Add dynamic password authentication interface support,command code 0xe6.
9. Added support for disconnecting Bluetooth connection,command code 0xe7.
10. Added support for querying MCU version number,command code 0xe8.
11. Added support for MCU to actively send version Numbers,command code 0xe9.
12. Add OTA upgrade request support,command code 0xea.
13. Add OTA update file information support,command 0xeb.
14. Add OTA upgrade file migration request support,command code 0xec.
15. Add OTA upgrade data support,command code 0xed.
16. Add OTA upgrade end support,command code 0xee.
17. Added support for MCU to acquire module version information,commadn code 0xa0.
18. Added support for resuming factory Settings notifications,command code 0xa1.
19. Add MCU OTA demo code.
20. Optimized bt_uart_service.
**
**-----------------------------------------------------------------------------
******************************************************************************/


/******************************************************************************
						  Pay special attention!
The way of MCU OTA is strongly related to the chip. The MCU OTA program demo is not necessarily suitable for all chip platforms, but it is more or less the same. 
Users can modify the demo according to the upgrade mode of their chip platform or refer to the demo to complete the MCU OTA function.

******************************************************************************/
#include "bluetooth.h"
#include "mcu_ota_handler.h"
#include <string.h>

#ifdef SUPPORT_MCU_FIRM_UPDATE

/*****************************************************************************
Function name: mcu_flash_init
Function description: flash initialization function
Input parameters: none
Return parameter: none
Instructions for use: users need to improve the flash initialization function here. If there is a flash initialization operation elsewhere, the function can not be called.
*****************************************************************************/
unsigned char mcu_flash_init(void)
{
	#error "Please improve this function by yourself, and delete the line when you are finished."
}
/*****************************************************************************
Function name: mcu_flash_erase
Function description: flash erasure function
Input parameter: 
Return parameter: 
Instructions for use: users improve themselves
*****************************************************************************/
static unsigned char mcu_flash_erase(unsigned int addr,unsigned int size)
{
	#error "Please improve this function by yourself, and delete the line when you are finished."
}
/*****************************************************************************
Function name: mcu_flash_write
Function description: flash write function
Input parameter: 
	addr: address 
	size:size
	p_data: data address to be written

Return parameter: 
Instructions for use: users improve themselves
*****************************************************************************/

static unsigned char mcu_flash_write(unsigned int addr, const unsigned char *p_data, unsigned int size)
{
	#error "Please improve this function by yourself, and delete the line when you are finished."
}

/*****************************************************************************
Function name: mcu_flash_read
Function description: flash read function
Input parameter: 
	addr: address 
	size:size
	p_data: 

Return parameter: 
Instructions for use: users improve themselves
*****************************************************************************/

static unsigned char mcu_flash_read(unsigned int addr, unsigned char *p_data, unsigned int size)
{
	#error "Please improve this function by yourself, and delete the line when you are finished."
}
/*****************************************************************************
Function name: mcu_device_delay_restart
Function description: delay restart function. It is recommended to delay 500ms restart in order to wait for mcu to complete some necessary operations.
Input parameter: 

Return parameter: 
Instructions for use: users improve themselves
*****************************************************************************/

static void mcu_device_delay_restart(void)
{
	error "Please improve this function by yourself, and delete the line when you are finished."
}







static dfu_settings_t s_dfu_settings;

static volatile mcu_ota_status_t tuya_ota_status;
static void mcu_ota_status_set(mcu_ota_status_t status)
{
	tuya_ota_status = status;
}
static mcu_ota_status_t mcu_ota_status_get(void)
{
	return tuya_ota_status;
}


#define MAX_DFU_DATA_LEN  200
#define CODE_PAGE_SIZE	4096
#define MAX_DFU_BUFFERS   ((CODE_PAGE_SIZE / MAX_DFU_DATA_LEN) + 1)


static unsigned int m_firmware_start_addr;          /**< Start address of the current firmware image. */
static unsigned int m_firmware_size_req;


static unsigned short current_package = 0;
static unsigned short last_package = 0;

static unsigned int crc32_compute(unsigned char const * p_data, unsigned int size, unsigned int const * p_crc)
{
	unsigned int crc;
    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (unsigned int i = 0; i < size; i++)
    {
        crc = crc ^ p_data[i];
        for (unsigned int j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}
static unsigned short crc16_compute(unsigned char * p_data, unsigned short size, unsigned short * p_crc) 
{

    unsigned short poly[2] = {0, 0xa001};            //0x8005 <==> 0xa001
    unsigned short crc = 0xffff;
//    unsigned char ds;
    int i, j;

    for(j = size; j > 0; j--) {
        unsigned char ds = *p_data++;

        for(i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ poly[(crc ^ ds) & 1];
            ds = ds >> 1;
        }
    }

    return crc;
}

static void mcu_ota_start_req(unsigned char*recv_data,unsigned int recv_len)
{
    unsigned char p_buf[12];
    unsigned char payload_len = 0;
    unsigned int current_version = MCU_OTA_VERSION;
	unsigned short length = 0;

    if(mcu_ota_status_get()!=MCU_OTA_STATUS_NONE)
    {
        //IOS 杀掉APP，设置里还是保持连接的,所以去掉限制(android是正常的)
        TUYA_OTA_LOG("current ota status is not MCU_OTA_STATUS_NONE  and is : %d !",mcu_ota_status_get());
        // return;
    }

    p_buf[0] = MCU_OTA_TYPE;
    p_buf[1] = (current_version>>16)&0xff;
    p_buf[2] = (current_version>>8)&0xff;
    p_buf[3] = current_version&0xff;

    p_buf[4] = MAX_DFU_DATA_LEN>>8;
    p_buf[5] = MAX_DFU_DATA_LEN;
    
    mcu_ota_status_set(MCU_OTA_STATUS_START);
    payload_len = 6;

	length = set_bt_uart_buffer(length,(unsigned char *)p_buf,payload_len);
	bt_uart_write_frame(TUYA_BCI_UART_COMMON_MCU_OTA_REQUEST,length);
}


static unsigned char file_crc_check_in_flash(unsigned int len,unsigned int *crc)
{

    static unsigned char buf[257]={0};
    if(len == 0)
    {
        return 1;
    }
    unsigned int crc_temp = 0;
    unsigned int read_addr = APP_NEW_FW_START_ADR;
    unsigned int cnt = len/256;
    unsigned int remainder = len % 256;
    for(unsigned int i = 0; i<cnt; i++)
    {
        TUYA_OTA_LOG("%d",i);
        mcu_flash_read(read_addr,buf,256);
        crc_temp = crc32_compute(buf, 256, &crc_temp);
        read_addr += 256;
    }

    if(remainder>0&&remainder<256)
    {
        TUYA_OTA_LOG("%d",remainder);
        mcu_flash_read(read_addr,buf,remainder);
        crc_temp = crc32_compute(buf, remainder, &crc_temp);
        read_addr += remainder;
    }
    
    *crc = crc_temp;
    
    return 0;
}




static void mcu_ota_file_info_req(unsigned char*recv_data,unsigned int recv_len)
{
    unsigned char p_buf[30];
    unsigned char payload_len = 0;
    unsigned int file_version;
    unsigned int file_length;
    unsigned int file_crc;
    unsigned char file_md5;
    // unsigned char file_md5[16];
    unsigned short length = 0;
    unsigned char state;
	
    if(mcu_ota_status_get()!=MCU_OTA_STATUS_START)
    {
        TUYA_OTA_LOG("current ota status is not MCU_OTA_STATUS_START  and is : %d !",mcu_ota_status_get());
        return;
    }

    file_version = recv_data[0+8]<<16;
    file_version += recv_data[1+8]<<8;
    file_version += recv_data[2+8];

    if(memcmp(s_dfu_settings.progress.firmware_file_md5,&recv_data[3+8],16)==0)
    {
        file_md5 = TRUE;
    }
    else
    {
        file_md5 = FALSE;
    }

    file_length = recv_data[27]<<24;
    file_length += recv_data[28]<<16;
    file_length += recv_data[29]<<8;
    file_length += recv_data[30];

    file_crc = recv_data[31]<<24;
    file_crc += recv_data[32]<<16;
    file_crc += recv_data[33]<<8;
    file_crc += recv_data[34];



    if (memcmp(&recv_data[0], PRODUCT_KEY, 8) == 0)
    {
        if((file_version > MCU_OTA_VERSION)&&(file_length <= APP_NEW_FW_MAX_SIZE))
        {

            if(file_md5&&(s_dfu_settings.progress.firmware_file_version==file_version)&&(s_dfu_settings.progress.firmware_file_length==file_length)
                    &&(s_dfu_settings.progress.firmware_file_crc==file_crc))
            {
                state = 0;
            }
            else
            {
                memset(&s_dfu_settings.progress, 0, sizeof(dfu_progress_t));
                s_dfu_settings.progress.firmware_image_crc_last = 0;
                s_dfu_settings.progress.firmware_file_version = file_version;
                s_dfu_settings.progress.firmware_file_length = file_length;
                s_dfu_settings.progress.firmware_file_crc = file_crc;
                memcpy(s_dfu_settings.progress.firmware_file_md5,&recv_data[3+8],16);
                s_dfu_settings.write_offset = s_dfu_settings.progress.firmware_image_offset_last;
                state = 0;
                mcu_flash_write(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings));
            }

            m_firmware_start_addr = APP_NEW_FW_START_ADR;
            m_firmware_size_req = s_dfu_settings.progress.firmware_file_length;

        }
        else
        {
            if(file_version <= MCU_OTA_VERSION)
            {
                TUYA_OTA_LOG("ota file version error !");
                state = 2;
            }
            else
            {
                TUYA_OTA_LOG("ota file length is bigger than rev space !");
                state = 3;
            }
        }

    }
    else
    {
        TUYA_OTA_LOG("ota pid error !");
        state = 1;
    }

    memset(p_buf,0,sizeof(p_buf));
    p_buf[0] = state;
    if(state==0)
    {
        unsigned int crc_temp  = 0;
        if(file_crc_check_in_flash(s_dfu_settings.progress.firmware_image_offset_last,&crc_temp)==0)
        {
            if(crc_temp != s_dfu_settings.progress.firmware_image_crc_last)
            {
                s_dfu_settings.progress.firmware_image_offset_last = 0;
                s_dfu_settings.progress.firmware_image_crc_last = 0;
                s_dfu_settings.write_offset = s_dfu_settings.progress.firmware_image_offset_last;
                mcu_flash_write(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings));
            }
        }

        p_buf[1] = s_dfu_settings.progress.firmware_image_offset_last>>24;
        p_buf[2] = s_dfu_settings.progress.firmware_image_offset_last>>16;
        p_buf[3] = s_dfu_settings.progress.firmware_image_offset_last>>8;
        p_buf[4] = (unsigned char)s_dfu_settings.progress.firmware_image_offset_last;
        
        p_buf[5] = s_dfu_settings.progress.firmware_image_crc_last>>24;
        p_buf[6] = s_dfu_settings.progress.firmware_image_crc_last>>16;
        p_buf[7] = s_dfu_settings.progress.firmware_image_crc_last>>8;
        p_buf[8] = (unsigned char)s_dfu_settings.progress.firmware_image_crc_last;
        mcu_ota_status_set(MCU_OTA_STATUS_FILE_INFO);
        current_package = 0;
        last_package = 0;

        TUYA_OTA_LOG("ota file length  : 0x%04x",s_dfu_settings.progress.firmware_file_length);
        TUYA_OTA_LOG("ota file  crc    : 0x%04x",s_dfu_settings.progress.firmware_file_crc);
        TUYA_OTA_LOG("ota file version : 0x%04x",s_dfu_settings.progress.firmware_file_version);
        TUYA_OTA_LOG("ota firmware_image_offset_last : 0x%04x",s_dfu_settings.progress.firmware_image_offset_last);
        TUYA_OTA_LOG("ota firmware_image_crc_last    : 0x%04x",s_dfu_settings.progress.firmware_image_crc_last);
        TUYA_OTA_LOG("ota firmware   write offset    : 0x%04x",s_dfu_settings.write_offset);

    }
    payload_len = 25;

	length = set_bt_uart_buffer(length,(unsigned char *)p_buf,payload_len);
	bt_uart_write_frame(TUYA_BCI_UART_COMMON_MCU_OTA_FILE_INFO,length);


}


static void mcu_ota_offset_req(unsigned char*recv_data,unsigned int recv_len)
{
    unsigned char p_buf[5];
    unsigned char payload_len = 0;
    unsigned int offset;
	unsigned short length = 0;
	
    if(mcu_ota_status_get()!=MCU_OTA_STATUS_FILE_INFO)
    {
        TUYA_OTA_LOG("current ota status is not MCU_OTA_STATUS_FILE_INFO  and is : %d !",mcu_ota_status_get());
        return;
    }


    offset  = recv_data[0]<<24;
    offset += recv_data[1]<<16;
    offset += recv_data[2]<<8;
    offset += recv_data[3];

    if((offset==0)&&(s_dfu_settings.progress.firmware_image_offset_last!=0))
    {
        s_dfu_settings.progress.firmware_image_crc_last = 0;
        s_dfu_settings.progress.firmware_image_offset_last = 0;
        s_dfu_settings.write_offset = s_dfu_settings.progress.firmware_image_offset_last;
        mcu_flash_write(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings));
    }

    p_buf[0] = s_dfu_settings.progress.firmware_image_offset_last>>24;
    p_buf[1] = s_dfu_settings.progress.firmware_image_offset_last>>16;
    p_buf[2] = s_dfu_settings.progress.firmware_image_offset_last>>8;
    p_buf[3] = (unsigned char)s_dfu_settings.progress.firmware_image_offset_last;

    mcu_ota_status_set(MCU_OTA_STATUS_FILE_OFFSET);

    payload_len = 4;

	length = set_bt_uart_buffer(length,(unsigned char *)p_buf,payload_len);
	bt_uart_write_frame(TUYA_BCI_UART_COMMON_MCU_OTA_FILE_OFFSET,length);


}



static void mcu_ota_data_req(unsigned char*recv_data,unsigned int recv_len)
{
	TUYA_OTA_LOG("%s",__func__);
    unsigned char p_buf[2];
    unsigned char payload_len = 0;
    unsigned char state = 0;
    unsigned short len;
    unsigned char p_alloc_buf[256] ={0};
    unsigned short length = 0;
    

    if((mcu_ota_status_get()!=MCU_OTA_STATUS_FILE_OFFSET)&&(mcu_ota_status_get()!=MCU_OTA_STATUS_FILE_DATA))
    {
        TUYA_OTA_LOG("current ota status is not MCU_OTA_STATUS_FILE_OFFSET  or MCU_OTA_STATUS_FILE_DATA and is : %d !",mcu_ota_status_get());
        return;
    }

    state = 0;


    current_package = (recv_data[0]<<8)|recv_data[1];
    len = (recv_data[2]<<8)|recv_data[3];

    if((current_package!=(last_package+1))&&(current_package!=0))
    {
        TUYA_OTA_LOG("ota received package number error.received package number : %d",current_package);
        state = 1;
    }
    else  if(len>MAX_DFU_DATA_LEN)
    {
        TUYA_OTA_LOG("ota received package data length error : %d",len);
        state = 5;
    }
    else
    {
        unsigned int const write_addr = APP_NEW_FW_START_ADR +  s_dfu_settings.write_offset;
        if(write_addr>=APP_NEW_FW_END_ADR)
        {
            TUYA_OTA_LOG("ota write addr error.");
            state = 1;
        }

        if(write_addr%CODE_PAGE_SIZE==0)
        {
            if (mcu_flash_erase(write_addr,4096)!= 0)
            {
                TUYA_OTA_LOG("ota Erase page operation failed");
                state = 4;
            }
        }

        if(state==0)
        {

            len = (recv_data[2]<<8)|recv_data[3];

            memcpy(p_alloc_buf, &recv_data[6], len);

            unsigned char ret = mcu_flash_write(write_addr, p_alloc_buf, len);
			TUYA_OTA_LOG("ota save len :%d",len);
			
            if (ret != 0)
            {
                state = 4;
            }
            else
            {
                s_dfu_settings.progress.firmware_image_crc_last = crc32_compute(p_alloc_buf, len, &s_dfu_settings.progress.firmware_image_crc_last);
                s_dfu_settings.write_offset    += len;
                s_dfu_settings.progress.firmware_image_offset_last += len;

                if((current_package+1)%32==0)
                {
                    mcu_flash_write(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings));
                }


            }
			
        }

    }

    p_buf[0] = state;

    mcu_ota_status_set(MCU_OTA_STATUS_FILE_DATA);

    payload_len = 1;

	length = set_bt_uart_buffer(length,(unsigned char *)p_buf,payload_len);
	bt_uart_write_frame(TUYA_BCI_UART_COMMON_MCU_OTA_DATA,length);



    if(state!=0)
    {
        TUYA_OTA_LOG("ota error so free!");
        mcu_ota_status_set(MCU_OTA_STATUS_NONE);
        mcu_ota_init_disconnect();
        memset(&s_dfu_settings, 0, sizeof(dfu_settings_t));
        mcu_flash_write(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings));
    }
    else
    {
        last_package = current_package;
    }


}


static void reset_after_flash_write(void * p_context)
{
    TUYA_OTA_LOG("start reset~~~.");
    mcu_device_delay_restart();
}


static void on_dfu_complete(void)
{

    TUYA_OTA_LOG("All flash operations have completed. DFU completed.");

    reset_after_flash_write(NULL);
}



static void on_data_write_request_sched(void * data)
{
    unsigned char          ret;
    unsigned char p_buf[1];
    unsigned char payload_len = 0;
    unsigned char state;
	unsigned short length = 0;

    if (s_dfu_settings.progress.firmware_image_offset_last == m_firmware_size_req)
    {
        TUYA_OTA_LOG("Whole firmware image received. Postvalidating.");
        unsigned int crc_temp  = 0;
        if(file_crc_check_in_flash(s_dfu_settings.progress.firmware_image_offset_last,&crc_temp)==0)
        {
            if(s_dfu_settings.progress.firmware_image_crc_last != crc_temp)
            {
                TUYA_OTA_LOG("file crc check in flash diff from crc_last. firmware_image_offset_last = 0x%04x,crc_temp = 0x%04x,crc_last = 0x%04x",s_dfu_settings.progress.firmware_image_offset_last,crc_temp,s_dfu_settings.progress.firmware_image_crc_last);
                s_dfu_settings.progress.firmware_image_crc_last = crc_temp;
            }
            
        }
        TUYA_OTA_LOG("file crc check past.");
        
		TUYA_OTA_LOG("file crc check in flash diff from crc_last. firmware_image_offset_last = 0x%04x,crc_temp = 0x%04x,crc_last = 0x%04x",s_dfu_settings.progress.firmware_image_offset_last,crc_temp,s_dfu_settings.progress.firmware_image_crc_last);
        if(s_dfu_settings.progress.firmware_image_crc_last!=s_dfu_settings.progress.firmware_file_crc)
        {
            TUYA_OTA_LOG("ota file crc check error,last_crc = 0x%04x ,file_crc = 0x%04x",s_dfu_settings.progress.firmware_image_crc_last,s_dfu_settings.progress.firmware_file_crc);
            state = 2;
        }
        else
        {
            s_dfu_settings.bank_1.image_crc = s_dfu_settings.progress.firmware_image_crc_last;
            s_dfu_settings.bank_1.image_size = m_firmware_size_req;
            s_dfu_settings.bank_1.bank_code = NRF_DFU_BANK_VALID_APP;

            memset(&s_dfu_settings.progress, 0, sizeof(dfu_progress_t));

            s_dfu_settings.write_offset                  = 0;
            s_dfu_settings.progress.update_start_address = APP_NEW_FW_START_ADR;

            state = 0;


        }


    }
    else
    {

        state = 1;
    }

    p_buf[0] = state;
    mcu_ota_status_set(MCU_OTA_STATUS_NONE);
    payload_len = 1;

	length = set_bt_uart_buffer(length,(unsigned char *)p_buf,payload_len);
	bt_uart_write_frame(TUYA_BCI_UART_COMMON_MCU_OTA_END,length);

    
    if(state==0)
    {
    	TUYA_OTA_LOG("ota will success!");
    	mcu_flash_write(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings)); 
        on_dfu_complete();
    }
    else
    {
        TUYA_OTA_LOG("ota crc error!");
        mcu_ota_status_set(MCU_OTA_STATUS_NONE);
        mcu_ota_init_disconnect();
        memset(&s_dfu_settings, 0, sizeof(dfu_settings_t));
        mcu_flash_write(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings));
    }

}



static void mcu_ota_end_req(unsigned char*recv_data,unsigned int recv_len)
{
    if(mcu_ota_status_get()==MCU_OTA_STATUS_NONE)
    {
        TUYA_OTA_LOG("current ota status is MCU_OTA_STATUS_NONE!");
        return;
    }
    on_data_write_request_sched(NULL);

}


void mcu_ota_proc(unsigned short cmd,unsigned char*recv_data,unsigned int recv_len)
{
    switch(cmd)
    {
    case TUYA_BCI_UART_COMMON_MCU_OTA_REQUEST:
        mcu_ota_start_req(recv_data,recv_len);
        break;
    case TUYA_BCI_UART_COMMON_MCU_OTA_FILE_INFO:
        mcu_ota_file_info_req(recv_data,recv_len);
        break;
    case TUYA_BCI_UART_COMMON_MCU_OTA_FILE_OFFSET:
        mcu_ota_offset_req(recv_data,recv_len);
        break;
    case TUYA_BCI_UART_COMMON_MCU_OTA_DATA:
        mcu_ota_data_req(recv_data,recv_len);
        break;
    case TUYA_BCI_UART_COMMON_MCU_OTA_END:
        mcu_ota_end_req(recv_data,recv_len);
        break;
    default:
    	TUYA_OTA_LOG("tuya_ota_proc cmd err.");
        break;
    }

}


unsigned char mcu_ota_init_disconnect(void)
{
    if(mcu_ota_status_get() != MCU_OTA_STATUS_NONE)
    {
        mcu_flash_write(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings));
        mcu_ota_status_set(MCU_OTA_STATUS_NONE);
    }
    current_package = 0;
    last_package = 0;
	return 0;
}

unsigned int mcu_ota_init(void)
{
    mcu_ota_status_set(MCU_OTA_STATUS_NONE);

    current_package = 0;
    last_package = 0;
	mcu_flash_read(DFU_SETTING_SAVE_ADDR,(unsigned char*)&s_dfu_settings,sizeof(s_dfu_settings));


    return 0;
}
#endif

