#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS 0                  // dùng FreeRTOS
#define SYS_LIGHTWEIGHT_PROT 1
#define MEM_ALIGNMENT 4
#define MEM_SIZE 1600
#define MEMP_NUM_TCP_PCB 5
#define MEMP_NUM_TCP_SEG 16
#define MEMP_NUM_NETBUF 2
#define MEMP_NUM_NETCONN 4
#define LWIP_TCP 1
#define TCP_QUEUE_OOSEQ 0
#define TCP_MSS 1440
#define TCP_SND_BUF (2*TCP_MSS)
#define TCP_WND (2*TCP_MSS)
#define LWIP_DHCP 1
#define LWIP_NETCONN 1
#define LWIP_SOCKET 0
#define LWIP_HAVE_LOOPIF 1
#define LWIP_STATS 0
#define LWIP_NETIF_LOOPBACK 1

#endif /* LWIPOPTS_H */
