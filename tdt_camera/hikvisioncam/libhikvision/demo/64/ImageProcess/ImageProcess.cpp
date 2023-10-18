#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

#define MAX_IMAGE_DATA_SIZE   (40*1024*1024)

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

	nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return -1;
    }
	
    // ��ʼȡ��
	// start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }
	
	MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
    unsigned int nDataSize = MAX_IMAGE_DATA_SIZE;
	unsigned char *pDataForRGB = NULL;
	unsigned char *pDataForSaveImage = NULL;
	
	nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
	if (nRet == MV_OK)
	{
		printf("Now you GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n", 
			stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);

	
		// ����ͼ��
		// image processing
		printf("input 0 to do nothing, 1 to convert RGB, 2 to save as BMP\n");
		int nInput = 0;
		scanf("%d", &nInput);
		switch (nInput)
		{
			// �����κ��£�����������
			// do nothing, and go on next
			case 0: 
			{
				break;
			}
			// ת��ͼ��ΪRGB��ʽ���û��ɸ�����������ת��������ʽ
			// convert image format to RGB, user can convert to other format by their requirement
			case 1: 
			{
				pDataForRGB = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
				if (NULL == pDataForRGB)
				{
					break;
				}
				// ���ظ�ʽת��
				// convert pixel format 
				MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
				// ���ϵ��������ǣ�ͼ���ͼ��ߣ��������ݻ��棬�������ݴ�С��Դ���ظ�ʽ��
				// Ŀ�����ظ�ʽ��������ݻ��棬�ṩ�������������С
				// Top to bottom are��image width, image height, input data buffer, input data size, source pixel format, 
				// destination pixel format, output data buffer, provided output buffer size
				stConvertParam.nWidth = stImageInfo.nWidth;
				stConvertParam.nHeight = stImageInfo.nHeight;
				stConvertParam.pSrcData = pData;
				stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
				stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
				stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
				stConvertParam.pDstBuffer = pDataForRGB;
				stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight *  4 + 2048;
				nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
			    if (MV_OK != nRet)
				{
					printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
					return -1;
				}

				FILE* fp = fopen("AfterConvert_RGB.raw", "wb");
				if (NULL == fp)
				{
					printf("fopen failed\n");
					break;
				}
				fwrite(pDataForRGB, 1, stConvertParam.nDstLen, fp);
				fclose(fp);
				printf("convert succeed\n");
				break;
			}
			case 2:
			{
				pDataForSaveImage = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
				if (NULL == pDataForSaveImage)
				{
					break;
				}
				// ����ͼ����
				// fill in the parameters of save image
				MV_SAVE_IMAGE_PARAM_EX stSaveParam;
				memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
				// ���ϵ��������ǣ����ͼƬ��ʽ���������ݵ����ظ�ʽ���ṩ�������������С��ͼ���
				// ͼ��ߣ��������ݻ��棬���ͼƬ���棬JPG��������
				// Top to bottom are��
				stSaveParam.enImageType = MV_Image_Bmp; 
				stSaveParam.enPixelType = stImageInfo.enPixelType; 
				stSaveParam.nBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
				stSaveParam.nWidth      = stImageInfo.nWidth; 
				stSaveParam.nHeight     = stImageInfo.nHeight; 
				stSaveParam.pData       = pData;
				stSaveParam.nDataLen    = stImageInfo.nFrameLen;
				stSaveParam.pImageBuffer = pDataForSaveImage;
				stSaveParam.nJpgQuality = 80;
				 
				nRet = MV_CC_SaveImageEx(&stSaveParam);
				if(MV_OK != nRet)
				{
					printf("failed in MV_CC_SaveImage,nRet[%x]\n", nRet);
					break;
				}

				FILE* fp = fopen("image.bmp", "wb");
				if (NULL == fp)
				{
					printf("fopen failed\n");
					break;
				}
				fwrite(pDataForSaveImage, 1, stSaveParam.nImageLen, fp);
				fclose(fp);
				printf("save image succeed\n");
				break;
			}
			default:
				break;
		}
	}
	
	// ֹͣȡ��
	// end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
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
	
	if (pData)
	{
		free(pData);	
		pData = NULL;
	}
	if (pDataForRGB)
	{
		free(pDataForRGB);
		pDataForRGB = NULL;
	}
	if (pDataForSaveImage)
	{
		free(pDataForSaveImage);
		pDataForSaveImage = NULL;
	}

	PressEnterToExit();
	printf("exit\n");
    return 0;
}
