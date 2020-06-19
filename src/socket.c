#include <winsock2.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
#include <STDIO.H>

static SOCKET sclient;

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

    struct sockaddr_in serAddr;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(8888);
    serAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
    if (connect(sclient, (struct sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
    {
        printf("connect error !");
        closesocket(sclient);
        return 0;
    }
}
void socket_vSend(int pind, float value)
{
    char *sendData = "csiljpojp\n";
    send(sclient, sendData, strlen(sendData), 0);
    char recData[255];
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
