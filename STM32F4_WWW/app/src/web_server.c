/**
 * @file    web_server.c
 * @brief   Web server
 * @date    19 sty 2015
 * @author  Michal Ksiezopolski
 *
 *
 * @verbatim
 * Copyright (c) 2014 Michal Ksiezopolski.
 * All rights reserved. This program and the
 * accompanying materials are made available
 * under the terms of the GNU Public License
 * v3.0 which accompanies this distribution,
 * and is available at
 * http://www.gnu.org/licenses/gpl.html
 * @endverbatim
 */

#include "web_server.h"
#include <stdio.h>
#include <string.h>
#include "ip_arp_udp_tcp.h"
#include "enc28j60.h"
#include "net.h"

#define DEBUG

#ifdef DEBUG
#define print(str, args...) printf(""str"%s",##args,"")
#define println(str, args...) printf("WEBSERVER--> "str"%s",##args,"\r\n")
#else
#define print(str, args...) (void)0
#define println(str, args...) (void)0
#endif


#define HTTP_PORT 80              ///< Port for WWW server
#define BUFFER_SIZE 850           ///< Ethernet buffer size
uint8_t buf[BUFFER_SIZE+1];       ///< Buffer for received packets

uint8_t macAddress[6] = {0x54,0x55,0x58,0x10,0x00,0x28}; ///< Device MAC address
uint8_t ipAddress[4] = {10,42,0,37}; ///< Device IP address

static uint16_t HTTP_PrintWebpage(uint8_t *buf);
static uint16_t HTTP_200OK(void);
static uint16_t HTTP_PrintSensors(uint8_t *buf,uint16_t plen, uint8_t sensor);

#define PAGE_INDEX "/index.html "

/**
 * @brief Initialize HTTP server
 */
void HTTP_Init(void) {

  // initialize the hardware driver for the ENC28J60
  ENC28J60_Init(macAddress);

  // initialize the TCP/IP stack
  init_ip_arp_udp_tcp(macAddress, ipAddress, HTTP_PORT);

}
/**
 * @brief Check for incoming Ethernet packets
 * @retval 0 No packets
 * @retval 1 Received packets
 */
uint8_t HTTP_Event(void) {

  uint16_t pos;
  // wait for packets, returns position of TCP data
  pos = packetloop_icmp_tcp(buf, ENC28J60_PacketReceive(BUFFER_SIZE, buf));

  // no packets
  if(pos == 0) {
      return 0;
  }

  println("Got packet");

  // parse HTTP commands
  if (strncmp("GET ", (char *)&(buf[pos]), 4)==0) {

    println("GET command received");

    // decode page
    if (strncmp("/ ",(char *)&(buf[pos+4]), 2)==0) {
      println("Sending web page");
      pos = HTTP_200OK();
      pos = fill_tcp_data(buf, pos, "<h1>200 OK</h1>");
      pos = fill_tcp_data(buf, pos, "<h1>STM32F4 server</h1>");

    } else if (strncmp(PAGE_INDEX,(char *)&(buf[pos+4]), strlen(PAGE_INDEX)) == 0) {
      println("Sending web page");
      pos = HTTP_PrintWebpage(buf);

    } else {
      println("Web page not found");
      pos = fill_tcp_data(buf, 0, ("HTTP/1.0 401 Unauthorized\r\n"
          "Content-Type: text/html\r\n\r\n"
          "<h1>401 Unauthorized</h1>"));
    }
  }

  // send reply
  www_server_reply(buf, pos);

  return 1;
}
/**
 * @brief Send HTTP header
 * @return Current position in buffer.
 */
uint16_t HTTP_200OK(void) {
  return(fill_tcp_data(buf, 0,
      "HTTP/1.0 200 OK\r\n"
      "Content-Type: text/html\r\n"
      "Pragma: no-cache\r\n\r\n"));
}
/**
 * @brief Print web page
 * @param buf Buffer to write the page to.
 * @return Current position in buffer
 */
uint16_t HTTP_PrintWebpage(uint8_t *buf) {

  uint16_t plen;
  plen = HTTP_200OK();
  plen = fill_tcp_data(buf,plen,("<pre style=\"background-color:#A8A8A8\">"));
  plen = fill_tcp_data(buf,plen,("<font color='green' size='6'>"
      "<b>Monitor sensors:</b></font></br>"));
  plen = HTTP_PrintSensors(buf, plen, 0);
  plen = fill_tcp_data(buf,plen,("</pre></br>Whatever"));
  plen = fill_tcp_data(buf,plen,("</br>"));
  return(plen);
}
/**
 * @brief Print sensor information
 * @param buf Buffer to print data to
 * @param plen Position in buffer to start writing
 * @param sensor Sensor number
 * @return Position in buffers after write
 */
uint16_t HTTP_PrintSensors(uint8_t *buf,uint16_t plen, uint8_t sensor) {

  plen=fill_tcp_data(buf,plen,("<font color='blue' size='5'>"
      "<i>Sensor no. 1:\t</i>"));
  plen=fill_tcp_data(buf,plen,"Hello world</font></br>");

	return plen;
}

