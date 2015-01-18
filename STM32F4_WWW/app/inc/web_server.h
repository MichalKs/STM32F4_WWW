#ifndef WEB_SERVER_H_
#define WEB_SERVER_H_

#include "ip_arp_udp_tcp.h"
#include "enc28j60.h"
#include "net.h"


// adres MAC
extern uint8_t mymac[6];
// adres IP urz�dzenia
extern uint8_t myip[4];

// server listen port for www
#define MYWWWPORT 80
/**
 * Rozmiar bufora na odebrany pakiet
 */
#define BUFFER_SIZE 850
extern uint8_t buf[BUFFER_SIZE+1];


/**
 * Zdarzenie od kontrolera Ethernetu
 */
uint8_t EthernetEvent();
/**
 * Inicjalizacja kontrolera Ethernet oraz stosu TCP/IP
 */
void WebserverInit();



#endif
