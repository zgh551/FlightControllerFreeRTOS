/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "main.h"

/* Definitions of physical drive number for each drive */
#define DEV_TF		0	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		1	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	switch (pdrv) {
	case DEV_TF :   
        stat = SD_GetStatus();
		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	switch (pdrv) {
	case DEV_TF :
		stat = SD_Init();
		return stat;
	}
	return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;  
	switch (pdrv) {
	case DEV_TF :   
      
//      res = SD_ReadDisk((uint8_t *)buff, sector, count);
    if((DWORD)buff%4!=0)
    {      
      /* Read block of many bytes from address 0 */
      res = (DRESULT)SD_ReadMultiBlocks((uint8_t *)buff,( sector << 9 ) + sector, 512, count);  
    }
    else
    {
      res = (DRESULT)SD_ReadMultiBlocks((uint8_t *)buff, ( sector << 9 ), 512, count);  
    }
    /* Check if the Transfer is finished */
    res = (DRESULT)SD_WaitReadOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);
    return res;
	}
	return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	switch (pdrv) {
	case DEV_TF : 
//      res = SD_WriteDisk((uint8_t*)buff, sector, count);
      /* Write multiple block of many bytes on address 0 */
      if((DWORD)buff%4!=0)
      {
        res = (DRESULT)SD_WriteMultiBlocks((uint8_t*)buff, ( sector << 9 ) + sector, 512, count);
      }
      else
      {
        res = (DRESULT)SD_WriteMultiBlocks((uint8_t*)buff, ( sector << 9 ), 512, count);
      }
      /* Check if the Transfer is finished */
      res = (DRESULT)SD_WaitWriteOperation();
      while(SD_GetStatus() != SD_TRANSFER_OK);  
      return res;
	}
	return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
    SD_CardInfo SDCard;
    if ( SD_GetCardInfo ( &SDCard ) ) return RES_NOTRDY;
	switch (pdrv) {
	case DEV_TF :
      switch(cmd)
      {
          case CTRL_SYNC:
              res = RES_OK; 
              break;	 
          case GET_SECTOR_SIZE:
              *(DWORD*)buff = 512; 
              res = RES_OK;
              break;	 
          case GET_BLOCK_SIZE:
              *(DWORD *)buff = SDCard.CardBlockSize;
              res = RES_OK;
              break;	 
          case GET_SECTOR_COUNT:
              *(DWORD*)buff = SDCard.CardCapacity/512;
              res = RES_OK;
              break;
          default:
              res = RES_PARERR;
              break;
      }
      return res;
	}
	return RES_PARERR;
}

//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{				 
	return 0;
}			 

void *ff_memalloc (UINT size)			
{
	return (void*)mymalloc(SRAMIN,size);
}

void ff_memfree (void* mf)		 
{
	myfree(SRAMIN,mf);
}