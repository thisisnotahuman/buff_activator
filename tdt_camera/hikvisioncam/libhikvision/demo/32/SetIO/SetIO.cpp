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
        printf("%s %x\n" , "nCurrentIp:" , pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                   //当前IP
        printf("%s %s\n\n" , "chUserDefinedName:" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);     //用户定义名
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
	
    // 打开设备
	// open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        return -1;
    }
	
	// 获取LineSelector
	// get LineSelector
    MVCC_ENUMVALUE stLineSelector = {0};
    nRet = MV_CC_GetEnumValue(handle, "LineSelector", &stLineSelector);
    if (MV_OK == nRet)
    {
        printf("stLineSelector current value:%d\n", stLineSelector.nCurValue);

        printf("supported stLineSelector number:%d\n", stLineSelector.nSupportedNum);

        for (unsigned int i = 0; i < stLineSelector.nSupportedNum; ++i)
        {
            printf("supported stLineSelector [%d]:%d\n", i, stLineSelector.nSupportValue[i]);
        }
		printf("\n");
    }
    else
    {
        printf("get stLineSelector failed! nRet [%x]\n\n", nRet);
    }
	
    // 设置LineSelector
	// set LineSelector
    unsigned int nLineSelector = 0;
    printf("please input the LineSelector to set:");
    scanf("%d", &nLineSelector);

    nRet = MV_CC_SetEnumValue(handle, "LineSelector", nLineSelector);
    if (MV_OK == nRet)
    {
        printf("set LineSelector OK!\n\n");
    }
    else
    {
        printf("set LineSelector failed! nRet [%x]\n\n", nRet);
    }
	
	// 获取LineMode
	// get LineMode
    MVCC_ENUMVALUE stLineMode = {0};
    nRet = MV_CC_GetEnumValue(handle, "LineMode", &stLineMode);
    if (MV_OK == nRet)
    {
        printf("stLineMode current value:%d\n", stLineMode.nCurValue);

        printf("supported stLineSelector number:%d\n", stLineMode.nSupportedNum);

        for (unsigned int i = 0; i < stLineMode.nSupportedNum; ++i)
        {
            printf("supported stLineSelector [%d]:%d\n", i, stLineMode.nSupportValue[i]);
        }
		printf("\n");
    }
    else
    {
        printf("get stLineMode failed! nRet [%x]\n\n", nRet);
    }
	
    // 设置LineMode
	// set LineMode
    unsigned int nLineMode = 0;
    printf("please input the LineMode to set:");
    scanf("%d", &nLineMode);

    nRet = MV_CC_SetEnumValue(handle, "LineMode", nLineMode);
    if (MV_OK == nRet)
    {
        printf("set LineMode OK!\n\n");
    }
    else
    {
        printf("set LineMode failed! nRet [%x]\n\n", nRet);
    }

	// 关闭设备
	// close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        return -1;
    }
	
    // 销毁句柄
	// destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        return -1;
    }

	PressEnterToExit();
	printf("exit\n");
    return 0;
}
