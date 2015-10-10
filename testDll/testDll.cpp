// testDll.cpp : 定义控制台应用程序的入口点。
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

//创建套接字  
SOCKET serverSocket;

//获取本机IP  
in_addr getHostName(void)
{
	char host_name[255];
	//获取本地主机名称  
	if (gethostname(host_name, sizeof(host_name)) == SOCKET_ERROR) {
		cout << "Error %d when getting local host name." << WSAGetLastError();
		Sleep(3000);
		exit(-1);
	}

	//从主机名数据库中得到对应的“IP”   
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

//接收数据线程  
void receive(PVOID param)
{
	SOCKET* Socket = (SOCKET*)param;
	char buf[8];
	int bytes;
	while (1)
	{
		//接收数据  
		if ((bytes = recv(*Socket, buf, sizeof(buf), 0)) == SOCKET_ERROR){
			cout << "接收数据失败!准备下一次接收\n";
			//_endthread();//终止当前线程  
			continue;
		}
		buf[bytes] = '\0';
		cout << "接收到数据：" << buf << endl;		
	}
}


//启动服务器  
SOCKET StartServer(void)
{
	if ((serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET){
		cout << "创建套接字失败！";
		Sleep(3000);
		exit(-1);
	}
	struct sockaddr_in serverAddress;
	//初始化指定的内存区域  
	memset(&serverAddress, 0, sizeof(sockaddr_in));
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(Server_Port);

	//绑定  
	if (bind(serverSocket, (sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR){
		cout << "套接字绑定到端口失败！端口:" << Server_Port;
		Sleep(3000);
		exit(-1);
	}

	//进入侦听状态  
	if (listen(serverSocket, SOMAXCONN) == SOCKET_ERROR){
		cout << "侦听失败！";
		Sleep(3000);
		exit(-1);
	}

	//获取服务器IP  
	struct in_addr addr = getHostName();
	cout << "Server " << inet_ntoa(addr) << " : " << Server_Port << " is listening......" << endl;
	return serverSocket;
}




int _tmain(int argc, _TCHAR* argv[])
{
	WSADATA wsa;//WSADATA结构被用来保存函数WSAStartup返回的Windows Sockets初始化信息  
	//MAKEWORD(a,b)是将两个byte型合并成一个word型，一个在高8位(b)，一个在低8位(a)   
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0){
		cout << "套接字初始化失败!";
		Sleep(3000);
		exit(-1);
	}
	SOCKET serverSocket = StartServer();//启动服务器 
	while (1)
	{
		SOCKET clientSocket;//用来和客户端通信的套接字  
		struct sockaddr_in clientAddress;//用来和客户端通信的套接字地址  
		memset(&clientAddress, 0, sizeof(clientAddress));//初始化存放客户端信息的内存  
		int addrlen = sizeof(clientAddress);

		//接受连接  
		if ((clientSocket = accept(serverSocket, (sockaddr*)&clientAddress, &addrlen)) == INVALID_SOCKET){
			cout << "接受客户端连接失败！继续等待……";
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

