#include "main.h"
#include "ESP8266.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define WAIT_RESPOND_TIME               500
#define WAIT_REPETE                     20

ESP8266::esp8266()
{
        LastError = NO_ERROR;
        pHandle = 0;
        memset(&Queue, 0, sizeof(QUEUE));
}

ESP8266::~esp8266()
{

}
void ESP8266::SetHandle(UART_HandleTypeDef *pUart)
{
        pHandle = pUart;
}

void ESP8266::SetSSID(uint8_t *pSSID)
{
        strcpy((char *)SSID, (char *)pSSID);
}

void ESP8266::SetPassword(uint8_t *pPassword)
{
        strcpy((char *)Password, (char *)pPassword);
}

bool ESP8266::SetIp(PIPSTRUCT pIp)
{
        uint8_t Buffer[128];

        memcpy(&Ip, pIp, sizeof(IPSTRUCT));
        sprintf((char *)Buffer, "AT+CIPSTA_CUR=\"%d.%d.%d.%d\",\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"",
                        Ip.ip.val[0], Ip.ip.val[1], Ip.ip.val[2], Ip.ip.val[3],
                        Ip.gateway.val[0], Ip.gateway.val[1], Ip.gateway.val[2], Ip.gateway.val[3],
                        Ip.subnet.val[0], Ip.subnet.val[1], Ip.subnet.val[2], Ip.subnet.val[3]);
        SendString(Buffer);
        return WaitForOK();
}

UART_HandleTypeDef *ESP8266::GetHandle(void)
{
        return pHandle;
}

uint8_t *ESP8266::GetSSID(void)
{
        return SSID;
}

uint8_t *ESP8266::GetPassword(void)
{
        return Password;
}

PIPSTRUCT ESP8266::GetIp(void)
{
        return &Ip;
}

uint8_t ESP8266::GetLastError(void)
{
        return LastError;
}

uint8_t ESP8266::GetData(void)
{
        if(Queue.Data == 0) {
                LastError = QUEUE_EMPTY;
                return FALSE;
        }
        uint8_t Ret = Queue.Queue[Queue.Tail++];
        Queue.Tail %= QUEUE_SIZE;
        Queue.Data--;
        LastError = NO_ERROR;
        return Ret;
}

bool ESP8266::PutData(uint8_t data)
{
        if(Queue.Data == QUEUE_SIZE) {
                LastError = QUEUE_FULL;
                return FALSE;
        }
        Queue.Queue[Queue.Head++] = data;
        Queue.Head %= QUEUE_SIZE;
        Queue.Data++;
        LastError = NO_ERROR;
        return TRUE;
}

uint16_t ESP8266::GetDataSoo(void)
{
        return Queue.Data;
}

bool ESP8266::CopyQueue(uint8_t *pDest)
{
        if(Queue.Data == 0) {
                LastError = QUEUE_EMPTY;
                return FAIL;
        }
        for(uint16_t i = 0;i < Queue.Data;i++) {
                *(pDest + i ) = Queue.Queue[(Queue.Tail + i) % QUEUE_SIZE];
        }
        LastError = NO_ERROR;
        return TRUE;
}

bool ESP8266::FindStringInqueue(uint8_t *pStr)
{
        uint8_t Buffer[QUEUE_SIZE] = {0,};
        uint8_t *p;

        CopyQueue(Buffer);
        if(LastError == QUEUE_EMPTY) return FALSE;
        p = (uint8_t *)strstr((char *)Buffer, (char *)pStr);
        if(p == 0) {
                LastError = NOT_FOUND;
                return FALSE;
        }
        LastError = NO_ERROR;
        return TRUE;
}

bool ESP8266::FindSkipStringInqueue(uint8_t *pConn, uint8_t *pStr)
{
        uint8_t Buffer[QUEUE_SIZE] = {0,};
        uint8_t *p;
        uint16_t Length;

        *pConn = 0;
        CopyQueue(Buffer);
        if(LastError == QUEUE_EMPTY) return FALSE;
        p = (uint8_t *)strstr((char *)Buffer, (char *)pStr);
        if(p == NULL) {
                LastError = NOT_FOUND;
                return FALSE;
        }
        *pConn = *(p - 2);
        Length = (int16_t)((int32_t)p - (int32_t)Buffer + strlen((char *)pStr));
        if(Queue.Data < Length) {
                LastError = DATA_NOTENOUGH;
                return FALSE;
        }
        Queue.Data -= Length;
        Queue.Tail += Length;
        Queue.Tail %= QUEUE_SIZE;
        LastError = NO_ERROR;
        return TRUE;
}

void ESP8266::SendString(uint8_t *pStr)
{
        if(pHandle == NULL) {
                LastError = HANDLE_NOTSET;
                return;
        }
        HAL_UART_Transmit(pHandle, pStr, strlen((char *)pStr), 10);
        HAL_UART_Transmit(pHandle, (uint8_t *)"\x0D\x0A", 2, 10);
        LastError = NO_ERROR;
}

bool ESP8266::WaitForOK(void)
{
        uint8_t r, j;
        uint16_t i;

        for(i = 0; i < WAIT_REPETE;i++) {
                HAL_Delay(WAIT_RESPOND_TIME);
                r = FindSkipStringInqueue(&j, (uint8_t *)"OK\x0D\x0A");
                if(r == TRUE) {
                        LastError = NO_ERROR;
                        return TRUE;
                }
        }
        LastError = NOT_FOUND;
        return FALSE;
}

bool ESP8266::SetModeStation(void)
{
        SendString((uint8_t *)"AT+CWMODE_CUR=1");
        return WaitForOK();
}

bool ESP8266::Connect(uint8_t *ssid, uint8_t *passwd)
{
        SetSSID(ssid);
        SetPassword(passwd);
        return Connect();
}

bool ESP8266::Connect(void)
{
        uint8_t Buffer[64];

        sprintf((char *)Buffer, "AT+CWJAP_CUR=\"%s\",\"%s\"", SSID, Password);
        SendString(Buffer);
        return WaitForOK();
}

bool ESP8266::SetMUX(uint8_t value)
{
        uint8_t Buffer[64];

        sprintf((char *)Buffer, "AT+CIPMUX=%d", value);
        SendString(Buffer);
        return WaitForOK();
}

bool ESP8266::SetServer(uint8_t set, uint16_t port)
{
        uint8_t Buffer[64];

        sprintf((char *)Buffer, "AT+CIPSERVER=%d,%d", set, port);
        SendString(Buffer);
        return WaitForOK();
}

bool ESP8266::CheckConnect(uint8_t *pConn)              // 0,CONNECT\x0D\x0A return connect number '0'
{
        return FindSkipStringInqueue(pConn, (uint8_t *)"CONNECT\x0D\x0A");
}

bool ESP8266::CheckClosed(uint8_t *pConn)               // 0,CLOSED\x0D\x0A return connect number '0'
{
        return FindSkipStringInqueue(pConn, (uint8_t *)"CLOSED\x0D\x0A");
}

bool ESP8266::Send(uint8_t Conn, uint16_t Length, uint8_t *pSend)
{
        uint8_t Buffer[64];
        uint16_t i;

        sprintf((char *)Buffer, "AT+CIPSEND=%c,%d", Conn, Length);
        SendString(Buffer);
        HAL_Delay(10);
        for(i = 0;i < Length;i++)
                HAL_UART_Transmit(pHandle, pSend + i, 1, 10);
        return WaitForOK();
}

bool ESP8266::Receive(uint8_t *pConn, uint16_t *pLength, uint8_t *pReceive)
{
        uint8_t *p;
        uint16_t i;                                     // +IPD,0,n:xxxx

        *pConn = *pLength = 0;
        if(FindSkipStringInqueue(pConn, (uint8_t *)"+IPD,") == FALSE)
                return FALSE;
        HAL_Delay(500);
        *pConn = GetData();
        GetData();              // ','
        p = pReceive;
        while(1) {
                *p = GetData();
                if(*p == ':') break;
                p++;
        }
        *p = 0;
        *pLength = atoi((char *)pReceive);
        if(Queue.Data < *pLength) {
                LastError = DATA_NOTENOUGH;
                return FALSE;
        }
        for(i = 0;i < *pLength;i++)
                *(pReceive + i) = GetData();
        LastError = NO_ERROR;
        return TRUE;
}

bool ESP8266::Close(uint8_t Conn)
{
        uint8_t Buffer[64];

        sprintf((char *)Buffer, "AT+CIPCLOSE=%d", Conn);
        SendString(Buffer);
        return WaitForOK();
}
