#ifndef _WIRELESS_H_
#define _WIRELESS_H_


// Function declarations
void Wireless_Setup();
void recvMsg(uint8_t *data, size_t len);
void processCharArray();

void sendData();
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

#endif // _WIRELESS_H_
