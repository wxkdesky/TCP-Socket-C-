// testDll.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "clkAndsync.h"
#include "time.h"
#include <windows.h>  
#include <process.h> 
#include <iostream>
using namespace std;
#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"..\\Debug\\clkAndsync.lib")
extern "C"{
	_declspec(dllimport) class CS_Kalman;
}

#define Server_Port 10000

//�����׽���  
SOCKET serverSocket;

//��ȡ����IP  
in_addr getHostName(void)
{
	char host_name[255];
	//��ȡ������������  
	if (gethostname(host_name, sizeof(host_name)) == SOCKET_ERROR) {
		cout << "Error %d when getting local host name." << WSAGetLastError();
		Sleep(3000);
		exit(-1);
	}

	//�����������ݿ��еõ���Ӧ�ġ�IP��   
	struct hostent *phe = gethostbyname(host_name);
	if (phe == 0) {
		cout << "Yow! Bad host lookup.";
		Sleep(3000);
		exit(-1);
	}

	struct in_addr addr;
	memcpy(&addr, phe->h_addr_list[0], sizeof(struct in_addr));
	return addr;
}

//���������߳�  
void receive(PVOID param)
{
	SOCKET* Socket = (SOCKET*)param;
	char buf[8];
	int bytes;
	while (1)
	{
		//��������  
		if ((bytes = recv(*Socket, buf, sizeof(buf), 0)) == SOCKET_ERROR){
			cout << "��������ʧ��!׼����һ�ν���\n";
			//_endthread();//��ֹ��ǰ�߳�  
			continue;
		}
		buf[bytes] = '\0';
		cout << "���յ����ݣ�" << buf << endl;		
	}
}


//����������  
SOCKET StartServer(void)
{
	if ((serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET){
		cout << "�����׽���ʧ�ܣ�";
		Sleep(3000);
		exit(-1);
	}
	struct sockaddr_in serverAddress;
	//��ʼ��ָ�����ڴ�����  
	memset(&serverAddress, 0, sizeof(sockaddr_in));
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(Server_Port);

	//��  
	if (bind(serverSocket, (sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR){
		cout << "�׽��ְ󶨵��˿�ʧ�ܣ��˿�:" << Server_Port;
		Sleep(3000);
		exit(-1);
	}

	//��������״̬  
	if (listen(serverSocket, SOMAXCONN) == SOCKET_ERROR){
		cout << "����ʧ�ܣ�";
		Sleep(3000);
		exit(-1);
	}

	//��ȡ������IP  
	struct in_addr addr = getHostName();
	cout << "Server " << inet_ntoa(addr) << " : " << Server_Port << " is listening......" << endl;
	return serverSocket;
}




int _tmain(int argc, _TCHAR* argv[])
{
	WSADATA wsa;//WSADATA�ṹ���������溯��WSAStartup���ص�Windows Sockets��ʼ����Ϣ  
	//MAKEWORD(a,b)�ǽ�����byte�ͺϲ���һ��word�ͣ�һ���ڸ�8λ(b)��һ���ڵ�8λ(a)   
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0){
		cout << "�׽��ֳ�ʼ��ʧ��!";
		Sleep(3000);
		exit(-1);
	}
	SOCKET serverSocket = StartServer();//���������� 
	while (1)
	{
		SOCKET clientSocket;//�����Ϳͻ���ͨ�ŵ��׽���  
		struct sockaddr_in clientAddress;//�����Ϳͻ���ͨ�ŵ��׽��ֵ�ַ  
		memset(&clientAddress, 0, sizeof(clientAddress));//��ʼ����ſͻ�����Ϣ���ڴ�  
		int addrlen = sizeof(clientAddress);

		//��������  
		if ((clientSocket = accept(serverSocket, (sockaddr*)&clientAddress, &addrlen)) == INVALID_SOCKET){
			cout << "���ܿͻ�������ʧ�ܣ������ȴ�����";
			continue;
		}
		cout << "Accept connection from " << inet_ntoa(clientAddress.sin_addr) << endl;
		_beginthread(receive, 0, &clientSocket);
	}
	//HMODULE HD11 = LoadLibrary(_T("..\\Debug\\clkAndsync.dll"));
	//if (HD11 != NULL)
	//{
	//	testF tt = (testF)GetProcAddress(HD11, "CS_Kalman");
	//	if (tt != NULL)
	//		tt(100);
	//	FreeLibrary(HD11);
	//}
	CS_Kalman cs;
	cs.test(10000);
	int a;
	cin >>a ;
	return 0;
}

