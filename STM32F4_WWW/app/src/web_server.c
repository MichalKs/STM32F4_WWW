/*********************************************
 * vim:sw=8:ts=8:si:et
 * To use the above modeline in vim you must have "set modeline" in your .vimrc
 * Author: Guido Socher
 * Copyright: GPL V2
 *
 * Tuxgraphics AVR webserver/ethernet board
 *
 * http://tuxgraphics.org/electronics/
 * Chip type           : Atmega88/168/328 with ENC28J60
 *
 *
 * MODYFIKACJE: Miros�aw Karda� --- ATmega32
 *
 * Modyfikacje Michal Ksiezopolski 05.2013
 *
 *********************************************/

#include "web_server.h"
#include <stm32f4xx.h>

#define DEBUG

#ifdef DEBUG
#define print(str, args...) printf(""str"%s",##args,"")
#define println(str, args...) printf("WEB--> "str"%s",##args,"\r\n")
#else
#define print(str, args...) (void)0
#define println(str, args...) (void)0
#endif

extern char rf_buf[30];
// ustalamy adres MAC
uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x28};
// ustalamy adres IP urz�dzenia
uint8_t myip[4] = {10,42,0,37};


// server listen port for www
#define MYWWWPORT 80

/**
 * Bufor na odebrany pakiet
 */
uint8_t buf[BUFFER_SIZE+1];

/**
 * Konstrukcja strony internetowej
 */
static uint16_t print_webpage(uint8_t *buf);
/**
 * Przes�anie nag��wk�w HTTP
 */
static uint16_t http200ok(void);

/**
 * Przeslanie informacji o danym czujniku
 */
static uint16_t print_sensor_info(uint8_t *buf,uint16_t plen, uint8_t sensor);

/**
 * Przes�anie nag��wk�w HTTP
 */
uint16_t http200ok(void) {
        return(fill_tcp_data(buf,0, ("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n")));
}

/**
 * Konstrukcja strony internetowej
 */
uint16_t print_webpage(uint8_t *buf) {

        uint16_t plen;
        // naglowek HTTP
        plen=http200ok();
        plen=fill_tcp_data(buf,plen,("<pre style=\"background-color:#A8A8A8\">"));
        plen=fill_tcp_data(buf,plen,("<font color='green' size='6'><b>Przeglad stanu czujnikow:</b>\n</font>"));
        // informacje o czujniku 1
        plen=print_sensor_info(buf,plen,0);
        plen=fill_tcp_data(buf,plen,("</font></pre>\n"));
        return(plen);
}
/**
 * Przeslanie informacji o danym czujniku
 */
uint16_t print_sensor_info(uint8_t *buf,uint16_t plen, uint8_t sensor) {
    plen=fill_tcp_data(buf,plen,("<font color='blue' size='5'><i>Czujnik nr 1:\t</i>"));
    //plen=fill_tcp_data(buf,plen,"<img src=\"http://www.napad.pl/katalog/data/katalog/produkty/srednie/czujnik-ruchu_paradox_pro-plus_glowne_500.jpg\" "
    //		"height=\"100\" width=\"100\">");
    plen=fill_tcp_data(buf,plen,"Hello world");

	//plen=fill_tcp_data(buf,plen,"<img src=\"http://krupers.pl/wp-content/uploads/2012/02/39660808231603111092680.jpg\" "
		//	"alt=\"Burglary\" height=\"400\" width=\"400\">");
	return plen;
}

/**
 * Zdarzenie od kontrolera Ethernetu
 */
uint8_t EthernetEvent() {
	// wskaznik na dane w odebranym pakiecie

	uint16_t dat_p;
	// read packet, handle ping and wait for a tcp packet:
	dat_p=packetloop_icmp_tcp(buf,ENC28J60_PacketReceive(BUFFER_SIZE, buf));

	if(dat_p==0){
			// no http request
			return 0;
	}

	println("Got packet");

	// tcp port 80 begin
	if (strcmp("GET ",(char *)&(buf[dat_p]),4)!=0){
			// head, post and other methods:
			dat_p=http200ok();
			dat_p=fill_tcp_data(buf,dat_p,"<h1>200 OK</h1>");
			dat_p=fill_tcp_data(buf,dat_p,"<h1>STM32F4 server</h1>");
			goto SENDTCP;
	}
	// just one web page in the "root directory" of the web server
	if (strcmp("/ ",(char *)&(buf[dat_p+4]),2)==0){
			dat_p=print_webpage(buf);
			goto SENDTCP;
	}else{
			dat_p=fill_tcp_data(buf,0,("HTTP/1.0 401 Unauthorized\r\nContent-Type: text/html\r\n\r\n<h1>401 Unauthorized</h1>"));
			goto SENDTCP;
	}
SENDTCP:
	www_server_reply(buf,dat_p); // send web page data
	// tcp port 80 end

	return 1;
}

/**
 * Inicjalizacja kontrolera Ethernet oraz stosu TCP/IP
 */
void WebserverInit() {

	//initialize the hardware driver for the enc28j60
	ENC28J60_Init(mymac);

	// za�aczenie diody sygnalizujacej prace kontrolera ethernet
	ENC28J60_PhyWrite(PHLCON,0x476);

	//init the ethernet/ip layer:
	init_ip_arp_udp_tcp(mymac,myip,MYWWWPORT);

	// testowanie komunikacji z modulem (odczyt MAC)
	int rev;
	// rev=enc28j60getrev();
	rev=ENC28J60_Read(MAADR5);
	print("%02x", rev);
	print(":");
	rev=ENC28J60_Read(MAADR4);
  print("%02x", rev);
  print(":");
	rev=ENC28J60_Read(MAADR3);
  print("%02x", rev);
  print(":");
  print("\r\n");

}


