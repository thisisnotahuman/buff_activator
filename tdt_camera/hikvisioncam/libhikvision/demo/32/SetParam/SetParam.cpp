#include <stdio.h>
#include <string.h>
#include "MvCameraControl.h"

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


int main()
{
    int nRet = MV_OK;

    void* handle = NULL;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // ö���豸
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

    // ѡ���豸���������
	// select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return -1;
    }

    // ���豸
	// open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        return -1;
    }

    // ����int�ͱ���
	// set IInteger variable
    unsigned int nHeightValue = 0;
    printf("please input the height to set:");
    scanf("%d", &nHeightValue);
   
	// �������ʱ�迼�ǲ���(16)�������ÿ����16�ı���
	// Step (16) should be considered when setting width and height, that is the width and height should be a multiple of 16
    nRet = MV_CC_SetIntValue(handle, "Height", nHeightValue);    
    if (MV_OK == nRet)
    {
        printf("set height OK!\n\n");
    }
    else
    {
        printf("set height failed! nRet [%x]\n\n", nRet);
    }
	
	// ��ȡint�ͱ���
	// get IInteger variable
    MVCC_INTVALUE stHeight = {0};
    nRet = MV_CC_GetIntValue(handle, "Height", &stHeight);
    if (MV_OK == nRet)
    {
        printf("height current value:%d\n", stHeight.nCurValue);
        printf("height max value:%d\n", stHeight.nMax);
        printf("height min value:%d\n", stHeight.nMin);
        printf("height increment value:%d\n\n", stHeight.nInc);
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }

    // ����float�ͱ���
	// set IFloat variable
    float fExposureTime = 0.0f;
    printf("please input the exposure time to set:");
    scanf("%f", &fExposureTime);

    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", fExposureTime);
    if (MV_OK == nRet)
    {
        printf("set exposure time OK!\n\n");
    }
    else
    {
        printf("set exposure time failed! nRet [%x]\n\n", nRet);
    }
	
	// ��ȡfloat�ͱ���
	// get IFloat variable
    MVCC_FLOATVALUE stExposureTime = {0};
    nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
    if (MV_OK == nRet)
    {
        printf("exposure time current value:%f\n", stExposureTime.fCurValue);
        printf("exposure time max value:%f\n", stExposureTime.fMax);
        printf("exposure time min value:%f\n\n", stExposureTime.fMin);
    }
    else
    {
        printf("get exposure time failed! nRet [%x]\n\n", nRet);
    }

    // ����enum�ͱ���
	// set IEnumeration variable
    unsigned int nTriggerMode = 0;
    printf("please input the TriggerMode to set:");
    scanf("%d", &nTriggerMode);

    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", nTriggerMode);
    if (MV_OK == nRet)
    {
        printf("set TriggerMode OK!\n\n");
    }
    else
    {
        printf("set TriggerMode failed! nRet [%x]\n\n", nRet);
    }
	
	// ��ȡenum�ͱ���
	// get IEnumeration variable
    MVCC_ENUMVALUE stTriggerMode = {0};
    nRet = MV_CC_GetEnumValue(handle, "TriggerMode", &stTriggerMode);
    if (MV_OK == nRet)
    {
        printf("TriggerMode current value:%d\n", stTriggerMode.nCurValue);

        printf("supported TriggerMode number:%d\n", stTriggerMode.nSupportedNum);

        for (unsigned int i = 0; i < stTriggerMode.nSupportedNum; ++i)
        {
            printf("supported TriggerMode [%d]:%d\n", i, stTriggerMode.nSupportValue[i]);
        }
		printf("\n");
    }
    else
    {
        printf("get TriggerMode failed! nRet [%x]\n\n", nRet);
    }

    // ����bool�ͱ���
	// set IBoolean variable
    int nSetBoolValue;
    bool bSetBoolValue;
    printf("please input the ReverseX to set(bool):");
    scanf("%d", &nSetBoolValue);
    if (0 != nSetBoolValue)
    {
        bSetBoolValue = true;
    }
    else
    {
        bSetBoolValue = false;
    }
    nRet = MV_CC_SetBoolValue(handle, "ReverseX", bSetBoolValue);
    if (MV_OK == nRet)
    {
        printf("Set ReverseX OK!\n\n");
    }
    else
    {
        printf("Set ReverseX Failed! nRet = [%x]\n\n", nRet);
    }
	
	// ��ȡbool�ͱ���
	// get IBoolean variable
    bool bGetBoolValue = false;
    nRet = MV_CC_GetBoolValue(handle, "ReverseX", &bGetBoolValue);
    if (MV_OK == nRet)
    {
		if (0 != bGetBoolValue)
		{
			printf("ReverseX current is true\n\n");
		}
		else
		{
			printf("ReverseX current is false\n\n");
		}
    }

    // ����string�ͱ���
	// set IString variable
    unsigned char strValue[256];
    printf("please input the DeviceUserID to set(string):");
    scanf("%s", strValue);
    nRet = MV_CC_SetStringValue(handle, "DeviceUserID", (char*)strValue);
    if (MV_OK == nRet)
    {
        printf("Set DeviceUserID OK!\n\n");
    }
    else
    {
        printf("Set DeviceUserID Failed! nRet = [%x]\n\n", nRet);
    }
	
	// ��ȡstring�ͱ���
	// get IString variable
	MVCC_STRINGVALUE stStringValue = {0};
    nRet = MV_CC_GetStringValue(handle, "DeviceUserID", &stStringValue);
    if (MV_OK == nRet)
    {
        printf("Get DeviceUserID [%s]\n\n", stStringValue.chCurValue);
    }
    else
    {
        printf("Get DeviceUserID Failed! nRet = [%x]\n\n", nRet);
    }
	
    // �ر��豸
	// close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        return -1;
    }

    // ���پ��
	// destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        return -1;
    }
	
	PressEnterToExit();

    return 0;

}
