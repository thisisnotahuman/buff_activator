#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

void* g_hHandle = NULL;
bool  g_bConnect = false;
char  g_strSerialNumber[64] = {0};

// �ȴ��û�����enter��������ȡ�����������
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
	int c;
	while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("%s\n" , "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
		// ��ӡ��ǰ���ip���û��Զ�������
		// print current ip and user defined name
        printf("%s %x\n" , "nCurrentIp:" , pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                   //��ǰIP
        printf("%s %s\n\n" , "chUserDefinedName:" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);     //�û�������
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

void __stdcall cbException(unsigned int nMsgType, void* pUser)
{
	printf("Device disconnect!\n");
	g_bConnect = false;
}

static void* ReconnectProcess(void* pUser)
{
	int nRet = MV_OK;
	MV_CC_DEVICE_INFO_LIST stDeviceList = {0};

	while(1)
	{
		if (true == g_bConnect)
		{
			sleep(1);
			continue;
		}
		
		nRet = MV_CC_CloseDevice(g_hHandle);
		nRet = MV_CC_DestroyHandle(g_hHandle);
		g_hHandle = NULL;

		printf("connecting...\n");
		// ö���豸
		// enum device
		nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
		if (MV_OK != nRet)
		{
			printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
			continue;
		}
		
		// �������к�ѡ�����
		unsigned int nIndex = -1;
		if (stDeviceList.nDeviceNum > 0)
		{
			for (int i = 0; i < stDeviceList.nDeviceNum; i++)
			{
				MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
				if (NULL == pDeviceInfo)
				{
					continue;
				} 

				
				if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) 
				{
					if (!strcmp((char*)(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber), g_strSerialNumber))
					{
						nIndex = i;
						break;
					}
				}
				else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
				{
					if (!strcmp((char*)(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), g_strSerialNumber))
					{
						nIndex = i;
						break;
					}
				}

			}  
		} 
		else
		{
			continue;
		}
		
		if (-1 == nIndex)
		{
			continue;
		}

		// ѡ���豸���������
		// select device and create handle
		nRet = MV_CC_CreateHandle(&g_hHandle, stDeviceList.pDeviceInfo[nIndex]);
		if (MV_OK != nRet)
		{
			printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
			continue;
		}

		// ���豸
		// open device
		nRet = MV_CC_OpenDevice(g_hHandle);
		if (MV_OK != nRet)
		{
			printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
			continue;
		}
		
		g_bConnect = true;
		
		// ע���쳣�ص�
		// register exception callback
		nRet = MV_CC_RegisterExceptionCallBack(g_hHandle, cbException, NULL);
		if (MV_OK != nRet)
		{
			printf("MV_CC_RegisterExceptionCallBack fail! nRet [%x]\n", nRet);
			continue;
		}
		printf("connect succeed\n");
	}
	return 0;
}

int main()
{
    int nRet = MV_OK;
	MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
	unsigned int nSelectNum = 0;
	// ö���豸
	// enum device
	nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
	if (MV_OK != nRet)
	{
		printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
		return -1;
	}
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            } 
            PrintDeviceInfo(pDeviceInfo);            
        }  
    } 
    else
    {
        printf("Find No Devices!\n");
        return -1;
    }


    scanf("%d", &nSelectNum);

	if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_GIGE_DEVICE) 
	{
		memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stGigEInfo.chSerialNumber, 
			sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stGigEInfo.chSerialNumber));
	}
	else if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_USB_DEVICE)
	{
		memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stUsb3VInfo.chSerialNumber, 
			sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stUsb3VInfo.chSerialNumber));
	}
	

	pthread_t nThreadID;
	nRet = pthread_create(&nThreadID, NULL, ReconnectProcess, NULL);
	if (nRet != 0)
	{
		printf("thread create failed nRet = %d\n",nRet);
		return -1;
	}
	
	PressEnterToExit();
	
    // �ر��豸
	// close device
    nRet = MV_CC_CloseDevice(g_hHandle);
    // ���پ��
	// destroy handle
    nRet = MV_CC_DestroyHandle(g_hHandle);
	
	printf("exit\n");
    return 0;
}
