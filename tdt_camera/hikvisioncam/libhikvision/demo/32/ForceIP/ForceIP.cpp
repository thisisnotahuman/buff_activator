#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
	int c;
	while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

bool ConvertToHexIp(unsigned int *nHexIP, unsigned int *nDecIP, char c)
{
	if ( nDecIP[0] < 0 || nDecIP[0] > 255
	  || nDecIP[1] < 0 || nDecIP[1] > 255
	  || nDecIP[2] < 0 || nDecIP[2] > 255
	  || nDecIP[3] < 0 || nDecIP[3] > 255
	  || c != '\n')
	{
		return false;
	}
	*nHexIP = (nDecIP[0] << 24) + (nDecIP[1] << 16) + (nDecIP[2] << 8) + nDecIP[3];

	return true;
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
		// 打印当前相机ip和用户自定义名字
		// print current ip and user defined name
        printf("%s %x\n" , "nCurrentIp:" , pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp); 
        printf("%s %s\n\n" , "chUserDefinedName:" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); 
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

int main()
{
    int nRet = MV_OK;
    void* handle = NULL;
	unsigned int nIP[4] = {0};
	char c = '\0';
	unsigned int nIpAddr = 0, nNetWorkMask = 0, nDefaultGateway = 0;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
	// enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return -1;
    }
    unsigned int nIndex = 0;
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

    scanf("%d", &nIndex);

    // 选择设备并创建句柄
	// select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return -1;
    }
	
	// 输入IP 子网掩码 默认网关
	// input ip, subnet mask and defaultway
	printf("Please input ip, example: 192.168.1.100\n");
	int ch;
	if ( 5 != scanf("%d.%d.%d.%d%c", &nIP[0], &nIP[1], &nIP[2], &nIP[3], &c) )
	{
		printf("input count error\n");
		MV_CC_DestroyHandle(handle);
		return -1;
	}
	if (!ConvertToHexIp(&nIpAddr, nIP, c))
	{
		printf("input IpAddr format is not correct\n");
		MV_CC_DestroyHandle(handle);
		return -1;
	}
	
	printf("Please input NetMask, example: 255.255.255.0\n");
	if ( 5 != scanf("%d.%d.%d.%d%c", &nIP[0], &nIP[1], &nIP[2], &nIP[3], &c) )
	{
		printf("input count error\n");
		MV_CC_DestroyHandle(handle);
		return -1;
	}
	if (!ConvertToHexIp(&nNetWorkMask, nIP, c))
	{
		printf("input NetMask format is not correct\n");
		MV_CC_DestroyHandle(handle);
		return -1;
	}
	
	printf("Please input DefaultWay, example: 192.168.1.1\n");
	if ( 5 != scanf("%d.%d.%d.%d%c", &nIP[0], &nIP[1], &nIP[2], &nIP[3], &c) )
	{
		printf("input count error\n");
		MV_CC_DestroyHandle(handle);
		return -1;
	}
	if (!ConvertToHexIp(&nDefaultGateway, nIP, c))
	{
		printf("input DefaultWay format is not correct\n");
		MV_CC_DestroyHandle(handle);
		return -1;
	}
	
	// 设置ForceIP
	// set forceip
    nRet = MV_GIGE_ForceIpEx(handle, nIpAddr, nNetWorkMask, nDefaultGateway);
	if (MV_OK != nRet)
    {
        printf("MV_GIGE_ForceIpEx fail! nRet [%x]\n", nRet);
        return -1;
    }
	printf("set IP succeed\n");

	PressEnterToExit();

    // 销毁句柄
	// destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        return -1;
    }

	printf("exit\n");
    return 0;
}
