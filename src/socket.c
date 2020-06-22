#include <winsock.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
#include <STDIO.H>
#include "cJSON.h"

static SOCKET sclient;
struct sockaddr_in serAddr;
static int msgcnts = 0;
//CJSON在内存中的存储方式是用链表进行存储的，所以在进行操作的时候，我们可见的部分全部是用指针进行操作的。
#include <stdio.h>
#include "cJSON.h"
cJSON *pJsonRoot = NULL;

cJSON *pJsonArry;
int arryLen = 0;

char *makeJson(void);

int socket_vinit(void)
{
    WORD sockVersion = MAKEWORD(2, 2);
    WSADATA data;
    if (WSAStartup(sockVersion, &data) != 0)
    {
        return 0;
    }

    sclient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sclient == INVALID_SOCKET)
    {
        printf("invalid socket !");
        return 0;
    }

    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(8888);
    serAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

    //定义要连接的服务器地址
    if (connect(sclient, (SOCKADDR *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
    {
        printf("客户端建立连接失败！\n");
        closesocket(sclient);
        WSACleanup();
        return 0;
    }
    else
        printf("客户端建立连接成功，准备发送数据！\n");
}

void socket_vSend(void)
{
    /*     if (pJsonArry == NULL)
    {
        pJsonArry = cJSON_CreateArray();
    }
    cJSON_AddItemToArray(pJsonArry, pJsonRoot);
    pJsonRoot = NULL; */
    arryLen++;

    // if (arryLen >= 20)
    {
        char *sendData = makeJson();
        // sendto(sclient, sendData, strlen(sendData), 0, (struct sockaddr *)&serAddr, sizeof(serAddr));
        send(sclient, sendData, strlen(sendData), 0);
        free(sendData);
        arryLen = 0;
        // cJSON_Delete(pJsonArry);
        msgcnts++;
        printf("%d\n", msgcnts);
    }
    /*int ret = recv(sclient, recData, 255, 0);
		if (ret > 0)
		{
	
			recData[ret] = 0x00;
			//printf(recData);
			printf("%d%s",i,recData);
		}
		*/
}

int socket_vClose()
{
    closesocket(sclient);
    WSACleanup();
    return 0;
}

void dbglog(char *name, float value)
{
    if (pJsonRoot == NULL)
    {
        pJsonRoot = cJSON_CreateObject(); //新建JSON主项目：pJsonRoot
    }
    cJSON_AddNumberToObject(pJsonRoot, name, value); //在主目录下添加一级目录number并添加数字10010
}

char *makeJson(void)
{
    char *p = cJSON_PrintUnformatted(pJsonRoot); //将项目压缩后(去除\t\n)转换成字符串输出到指针p上
    if (NULL == p)
    {
        //convert json list to string faild, exit
        //because sub json pSubJson han been add to pJsonRoot, so just delete pJsonRoot, if you also delete pSubJson, it will coredump, and error is : double free
        cJSON_Delete(pJsonRoot);
        return NULL;
    }
    cJSON_Delete(pJsonRoot); //删除项目
    pJsonRoot = NULL;
    return p;
}
