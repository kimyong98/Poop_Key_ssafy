#ifndef INC_ESP8266_H_
#define INC_ESP8266_H_

#define QUEUE_SIZE              1024
#define SSID_PW_LENGTH  16

#define NO_ERROR                0
#define NOT_FOUND               0xFF
#define HANDLE_NOTSET           1
#define QUEUE_EMPTY             10
#define QUEUE_FULL              11
#define DATA_NOTENOUGH  12



typedef union {
        uint8_t val[4];
        uint32_t longvalue;
} IPUNION, *PIPUNION;

typedef struct {
        IPUNION ip;
        IPUNION subnet;
        IPUNION gateway;
} IPSTRUCT, *PIPSTRUCT;

typedef struct {
        uint16_t Head, Tail, Data;
        uint8_t Queue[QUEUE_SIZE];
} QUEUE, *PQUEUE;

typedef class esp8266 {
protected:
        QUEUE Queue;
        uint8_t LastError;
        UART_HandleTypeDef *pHandle;
        uint8_t SSID[SSID_PW_LENGTH];
        uint8_t Password[SSID_PW_LENGTH];
        IPSTRUCT Ip;
public:
        esp8266();
        ~esp8266();
        void SetHandle(UART_HandleTypeDef *pUart);
        void SetSSID(uint8_t *pSSID);
        void SetPassword(uint8_t *pPassword);
        bool SetIp(PIPSTRUCT pIp);
        UART_HandleTypeDef *GetHandle(void);
        uint8_t *GetSSID(void);
        uint8_t *GetPassword(void);
        PIPSTRUCT GetIp(void);
        uint8_t GetLastError(void);
        uint8_t GetData(void);
        bool PutData(uint8_t data);
        uint16_t GetDataSoo(void);
        bool CopyQueue(uint8_t *pDest);
        bool FindStringInqueue(uint8_t *pStr);
        bool FindSkipStringInqueue(uint8_t *pConn, uint8_t *pStr);
        void SendString(uint8_t *pStr);
        bool WaitForOK(void);
        bool SetModeStation(void);
        bool Connect(uint8_t *ssid, uint8_t *passwd);
        bool Connect(void);
        bool SetMUX(uint8_t value);
        bool SetServer(uint8_t set, uint16_t port);
        bool CheckConnect(uint8_t *pConn);
        bool CheckClosed(uint8_t *pConn);
        bool Send(uint8_t Conn, uint16_t Length, uint8_t *pSend);
        bool Receive(uint8_t *pConn, uint16_t *pLength, uint8_t *pReceive);
        bool Close(uint8_t Conn);
} ESP8266, *PESP8266;


#endif /* INC_ESP8266_H_ */
