// quadcopter_flight_software.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "logger.h"
#include "gyro.h"
#include "motorVectorCtrl.h"
#include "mavlink_udp.h"
#include "quadcopter_flight_software.h"
#include <winsock2.h>
#include "common.h"

#include "motor_control.h"
#include "pidControl.h"

#define MAX_LOADSTRING 100

// Global Variables:
static HINSTANCE hInst;								// current instance
static HWND hWnd;
static TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
static TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
static ATOM				MyRegisterClass(HINSTANCE hInstance);
static BOOL				initInstance(HINSTANCE, int);
static void				cleanupInstance();
static LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
//static INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);
static void readIni( PIDcoefficients* PIDvals, char* port1, char* port2, char* target_ip );


int APIENTRY _tWinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPTSTR    lpCmdLine,
	int       nCmdShow)
{
	char GYRO_PORT[128] = "COM8";
	char MAESTRO_PORT[128] = "COM6";
	char target_ip[128] = "127.0.0.1";

	QLogger log;

	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_QUADCOPTER_FLIGHT_SOFTWARE, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if( ! initInstance(hInstance, nCmdShow) )
		return FALSE;

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_QUADCOPTER_FLIGHT_SOFTWARE));

	PIDcoefficients pidVals = { 0, 0, 0 };
	readIni( &pidVals, GYRO_PORT, MAESTRO_PORT, target_ip );

	//CreateThread mavlink
	MavThreadParam mavParam = { GetCurrentThreadId() };
	strcpy_s( mavParam.target_ip, target_ip );
	HANDLE h = CreateThread( NULL, 0, mav_thread, &mavParam, 0, NULL );
	if( h )
		CloseHandle( h );

	//gyroscope thread
	GyroThreadParam gyroParam = { GetCurrentThreadId(), "" };
	strcpy_s( gyroParam.portname, GYRO_PORT );
	h = CreateThread( NULL, 0, commReader, &gyroParam, 0, NULL );
	if( h )
		CloseHandle( h );


	//init things below: PID, motor, etc
	
	PidControl pidX( pidVals.Kp, pidVals.Ki, pidVals.Kd, &log );
	PidControl pidY( pidVals.Kp, pidVals.Ki, pidVals.Kd, NULL );

	MotorControl motorCtrl;
	motorCtrl.openPort( MAESTRO_PORT );

	motorVectorCtrl motVec;

	const unsigned ID_WATCHDOG = 11;
	SetTimer( hWnd, ID_WATCHDOG, 1000, NULL );
	
	bool watchdog_gyro = false;
	bool watchdog_mav = false;

	//max angle for the angle
	const int maxAngle = 40;
	const double maxRotPower = 0.1;

	// Main message loop:
	MSG msg;
	MavManualMsg attitude = { 0, 0, 0, 0 };

	GyroVals latestGVal = { 0, 0, 0 };
	GyroVals offset = { 0, 0, 0 };

	double lift = 0, rotate = 0, tiltX = 0, tiltY = 0;
	
	bool first = true;
	bool landed = true;

	while( GetMessage( &msg, NULL, 0, 0 ) )
	{
		if( msg.message == GYRO_MSG || msg.message == MAV_MSG_CTRL )
		{
			if( msg.message == GYRO_MSG )
			{
				watchdog_gyro = true;
				GyroVals* tmpGyro = ( GyroVals* )msg.lParam;
				latestGVal = *tmpGyro;
				delete tmpGyro;
				if( first )
				{
					offset = latestGVal;
					first = false;
				}
				latestGVal.x -= offset.x;
				latestGVal.y -= offset.y;
				
				
				char logbuf[512];
				sprintf_s( logbuf, "New gyro packet (TXYZ): %d, %lf, %lf, %lf", latestGVal.timestamp, latestGVal.x, latestGVal.y, latestGVal.z );
				log.log( logbuf );
				
			}
			else if( msg.message == MAV_MSG_CTRL )
			{
				watchdog_mav = true;

				MavManualMsg* newMavMsg = ( MavManualMsg * )msg.lParam;
				attitude = *newMavMsg;
				delete newMavMsg;

				//ideally, attitude would contain side / forward / upwards speeds
				//since we don't have a point of reference for hover, 
				//thrust is converted from from {-1, 1} to {0, 1}
				attitude.thrust += 1.0;
				attitude.thrust /= 2.0;
				
				char logbuf[512];
				sprintf_s( logbuf, "New mav packet (PRYT): %lf, %lf, %lf, %lf", attitude.pitch, attitude.roll, attitude.yaw, attitude.thrust );
				log.log( logbuf );
				
			}

			int curTime = (int) GetTickCount();
			double lift = ( attitude.thrust );
			lift *= motVec.adjustLiftPowForTilt( latestGVal.x , latestGVal.y );
			double rotate = attitude.yaw * maxRotPower;
			double tiltX = pidX.pidMotorValue( attitude.roll * maxAngle, latestGVal.x, curTime );
			double tiltY = pidY.pidMotorValue( attitude.pitch * maxAngle, latestGVal.y, curTime );

			motVec.setAllCoefficients( lift, rotate, tiltX, tiltY );
			motVec.computeVect( );

			
			char logbuf[512];
			sprintf_s( logbuf, "Setting motors (0123): %lf, %lf, %lf, %lf", motVec.motorPower[0], motVec.motorPower[1], motVec.motorPower[2], motVec.motorPower[3] );
			log.log( logbuf );
			

			for( int i = 0; i < 4; i++ )
			{
				motorCtrl.setMotor( i, motVec.motorPower[i] );
			}
		}
		else if( msg.message == MAV_MSG_ACTION )
		{
			//Start/stop, etc etc
		}
		else if( msg.message == WM_TIMER && msg.wParam == ID_WATCHDOG )
		{
			if( ! watchdog_gyro )
			{
				//motorCtrl.zeroMotors();
			}
			else if( !watchdog_mav )
			{
				//land nicely
				
				//set angles to 0
				/*
				
				attitude.pitch = 0;
				attitude.roll = 0;
				attitude.yaw = 0;
				attitude.thrust *= 0.9;
				*/
			}
			watchdog_gyro = false;
			watchdog_mav = false;
		}
		else
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	cleanupInstance();
	return (int) msg.wParam;
}

static void clearEOL( char* s )
{
	for( int i = 0; s[i]; i++ )
		if( s[i] == '\n' || s[i] == '\r' )
			s[i] = 0;
}

static void readIni( PIDcoefficients* PIDvals, char* port1, char* port2, char* target_ip )
{
	//read copter.ini
	/*sample:
	COM5
	COM6
	3.5343325
	0.0112344
	1.341
	*/

	char portName[128] = "COM5";
	char portName2[128] = "COM4";
	char target_ip_loc[128] = "127.0.0.1";

	FILE* ini;
	errno_t err = fopen_s( &ini, "copter.ini", "rb" );
	if( ini )
	{
		/*
		char maestroPath[MAX_PATH + 32];
		fgets( maestroPath, sizeof( maestroPath ), ini );
		clearEOL( maestroPath );
		/*char* p = strchr( maestroPath, 0 );
		*p++ = ' ';*/

		fgets( portName, sizeof( portName ), ini );
		clearEOL( portName );
		strcpy_s( port1, 128, portName );

		fgets( portName2, sizeof( portName2 ), ini );
		clearEOL( portName2 );
		strcpy_s( port2, 128, portName2 );

		fgets( target_ip_loc, sizeof( target_ip_loc ), ini );
		clearEOL( target_ip_loc );
		strcpy_s( target_ip, 128, target_ip_loc );

		
		char buff[256];
		fgets( buff, sizeof(buff), ini );
		PIDvals->Kp = strtod( buff, 0 );
		fgets( buff, sizeof(buff), ini );
		PIDvals->Ki = strtod( buff, 0 );
		fgets( buff, sizeof(buff), ini );
		PIDvals->Kd = strtod( buff, 0 );

		fclose( ini );
	}
}

static ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= 0; // CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_QUADCOPTER_FLIGHT_SOFTWARE));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_QUADCOPTER_FLIGHT_SOFTWARE);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

static BOOL initInstance(HINSTANCE hInstance, int nCmdShow)
{
	WORD wVersionRequested = MAKEWORD(2, 2);
	WSADATA wsaData = {0};
	int err = WSAStartup( wVersionRequested, &wsaData );
	if( err )
		return FALSE;

	hInst = hInstance; // Store instance handle in our global variable

	hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

	if (!hWnd)
		return FALSE;

	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	return TRUE;
}

static void cleanupInstance()
{
	WSACleanup();
}

static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	try
	{
		switch (message)
		{
		case WM_COMMAND:
			{
				int wmId    = LOWORD(wParam);
				int wmEvent = HIWORD(wParam);
				// Parse the menu selections:
				switch (wmId)
				{
	//			case IDM_ABOUT:
	//				DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
	//				break;
				case IDM_EXIT:
					DestroyWindow(hWnd);
					break;
				default:
					return DefWindowProc(hWnd, message, wParam, lParam);
				}
			}
			break;
		case WM_PAINT:
			{
				PAINTSTRUCT ps;
				HDC hdc;
				hdc = BeginPaint(hWnd, &ps);
				// TODO: Add any drawing code here...
				EndPaint(hWnd, &ps);
			}
			break;
		case WM_DESTROY:
			PostQuitMessage(0);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
	}
	catch( ... )
	{
	}

	return 0;
}
/*
// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}
*/