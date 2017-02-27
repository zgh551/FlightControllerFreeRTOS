#include "malloc.h"	
#include <stdbool.h>
#include <stdint.h>
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEKս��STM32������
//�ڴ���� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//�ڴ��(4�ֽڶ���)__align(4) __align(4)__no_init__no_init  
#pragma pack(push) //�������״̬ 
#pragma pack(4)//�趨Ϊ4�ֽڶ���  
uint8_t mem1base[MEM1_MAX_SIZE];									//�ڲ�SRAM�ڴ��
//uint8_t mem2base[MEM2_MAX_SIZE] @(0X68000000+MEM2_MAX_SIZE);		//�ⲿSRAM�ڴ��
#pragma pack(pop)//�ָ�����״̬
//�ڴ������
uint16_t mem1mapbase[MEM1_ALLOC_TABLE_SIZE];													//�ڲ�SRAM�ڴ��MAP
//uint16_t mem2mapbase[MEM2_ALLOC_TABLE_SIZE] @(0X68000000+MEM2_MAX_SIZE);	//�ⲿSRAM�ڴ��MAP
//�ڴ��������	   
const uint32_t memtblsize[2]={MEM1_ALLOC_TABLE_SIZE,MEM2_ALLOC_TABLE_SIZE};		//�ڴ����С
const uint32_t memblksize[2]={MEM1_BLOCK_SIZE,MEM2_BLOCK_SIZE};					//�ڴ�ֿ��С
const uint32_t memsize[2]={MEM1_MAX_SIZE,MEM2_MAX_SIZE};							//�ڴ��ܴ�С


//�ڴ����������mem2base mem2mapbase
struct _m_mallco_dev mallco_dev=
{
	mem_init,				//�ڴ��ʼ��
	mem_perused,			//�ڴ�ʹ����
	mem1base,		//�ڴ��
	mem1mapbase,//�ڴ����״̬��
	0,  					//�ڴ����δ����
};

//�����ڴ�
//*des:Ŀ�ĵ�ַ
//*src:Դ��ַ
//n:��Ҫ���Ƶ��ڴ泤��(�ֽ�Ϊ��λ)
void mymemcpy(void *des,void *src,uint32_t n)  
{  
    uint8_t *xdes=des;
	uint8_t *xsrc=src; 
    while(n--)*xdes++=*xsrc++;  
}  
//�����ڴ�
//*s:�ڴ��׵�ַ
//c :Ҫ���õ�ֵ
//count:��Ҫ���õ��ڴ��С(�ֽ�Ϊ��λ)
void mymemset(void *s,uint8_t c,uint32_t count)  
{  
    uint8_t *xs = s;  
    while(count--)*xs++=c;  
}	   
//�ڴ������ʼ��  
//memx:�����ڴ��
void mem_init(uint8_t memx)  
{  
    mymemset(mallco_dev.memmap[memx], 0,memtblsize[memx]*2);//�ڴ�״̬����������  
	mymemset(mallco_dev.membase[memx], 0,memsize[memx]);	//�ڴ��������������  
	mallco_dev.memrdy[memx]=1;								//�ڴ������ʼ��OK  
}  
//��ȡ�ڴ�ʹ����
//memx:�����ڴ��
//����ֵ:ʹ����(0~100)
uint8_t mem_perused(uint8_t memx)  
{  
    uint32_t used=0;  
    uint32_t i;  
    for(i=0;i<memtblsize[memx];i++)  
    {  
        if(mallco_dev.memmap[memx][i])used++; 
    } 
    return (used*100)/(memtblsize[memx]);  
}  
//�ڴ����(�ڲ�����)
//memx:�����ڴ��
//size:Ҫ������ڴ��С(�ֽ�)
//����ֵ:0XFFFFFFFF,��������;����,�ڴ�ƫ�Ƶ�ַ 
uint32_t mem_malloc(uint8_t memx,uint32_t size)  
{  
    signed long offset=0;  
    uint16_t nmemb;	//��Ҫ���ڴ����  
	uint16_t cmemb=0;//�������ڴ����
    uint32_t i;  
    if(!mallco_dev.memrdy[memx])mallco_dev.init(memx);//δ��ʼ��,��ִ�г�ʼ�� 
    if(size==0)return 0XFFFFFFFF;//����Ҫ����
    nmemb=size/memblksize[memx];  	//��ȡ��Ҫ����������ڴ����
    if(size%memblksize[memx])nmemb++;  
    for(offset=memtblsize[memx]-1;offset>=0;offset--)//���������ڴ������  
    {     
		if(!mallco_dev.memmap[memx][offset])cmemb++;//�������ڴ��������
		else cmemb=0;								//�����ڴ������
		if(cmemb==nmemb)							//�ҵ�������nmemb�����ڴ��
		{
            for(i=0;i<nmemb;i++)  					//��ע�ڴ��ǿ� 
            {  
                mallco_dev.memmap[memx][offset+i]=nmemb;  
            }  
            return (offset*memblksize[memx]);//����ƫ�Ƶ�ַ  
		}
    }  
    return 0XFFFFFFFF;//δ�ҵ����Ϸ����������ڴ��  
}  
//�ͷ��ڴ�(�ڲ�����) 
//memx:�����ڴ��
//offset:�ڴ��ַƫ��
//����ֵ:0,�ͷųɹ�;1,�ͷ�ʧ��;  
uint8_t mem_free(uint8_t memx,uint32_t offset)  
{  
    int i;  
    if(!mallco_dev.memrdy[memx])//δ��ʼ��,��ִ�г�ʼ��
	{
		mallco_dev.init(memx);    
        return 1;//δ��ʼ��  
    }  
    if(offset<memsize[memx])//ƫ�����ڴ����. 
    {  
        int index=offset/memblksize[memx];			//ƫ�������ڴ�����  
        int nmemb=mallco_dev.memmap[memx][index];	//�ڴ������
        for(i=0;i<nmemb;i++)  						//�ڴ������
        {  
            mallco_dev.memmap[memx][index+i]=0;  
        }  
        return 0;  
    }else return 2;//ƫ�Ƴ�����.  
}  
//�ͷ��ڴ�(�ⲿ����) 
//memx:�����ڴ��
//ptr:�ڴ��׵�ַ 
void myfree(uint8_t memx,void *ptr)  
{  
	u32 offset;  
    if(ptr==NULL)return;//��ַΪ0.  
 	offset=(u32)ptr-(u32)mallco_dev.membase[memx];  
    mem_free(memx,offset);//�ͷ��ڴ�     
}  
//�����ڴ�(�ⲿ����)
//memx:�����ڴ��
//size:�ڴ��С(�ֽ�)
//����ֵ:���䵽���ڴ��׵�ַ.
void *mymalloc(uint8_t memx,uint32_t size)  
{  
    uint32_t offset;  									      
	offset=mem_malloc(memx,size);  	   				   
    if(offset==0XFFFFFFFF)return NULL;  
    else return (void*)((u32)mallco_dev.membase[memx]+offset);  
}  
//���·����ڴ�(�ⲿ����)
//memx:�����ڴ��
//*ptr:���ڴ��׵�ַ
//size:Ҫ������ڴ��С(�ֽ�)
//����ֵ:�·��䵽���ڴ��׵�ַ.
void *myrealloc(uint8_t memx,void *ptr,uint32_t size)  
{  
    uint32_t offset;  
    offset=mem_malloc(memx,size);  
    if(offset==0XFFFFFFFF)return NULL;     
    else  
    {  									   
	    mymemcpy((void*)((uint32_t)mallco_dev.membase[memx]+offset),ptr,size);	//�������ڴ����ݵ����ڴ�   
        myfree(memx,ptr);  											  		//�ͷž��ڴ�
        return (void*)((uint32_t)mallco_dev.membase[memx]+offset);  				//�������ڴ��׵�ַ
    }  
}











