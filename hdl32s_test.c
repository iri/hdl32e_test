//
// For Linux
// gcc -o hdl32e_test -pthread hdl32e_test.c
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <netinet/in.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/udp.h>   //Provides declarations for udp header
#include <netinet/ip.h>    //Provides declarations for ip header
#include <netinet/if_ether.h>  //For ETH_P_ALL
#include <net/ethernet.h>  //For ether_header
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
 

pthread_t tid_sensor_loop, tid_scene_loop;
pthread_mutex_t lock;
static unsigned int grid[36][32];
struct sockaddr_in source,dest;


void cls() {
  const char* CLEAR_SCREE_ANSI = "\e[1;1H\e[2J";
  write(STDOUT_FILENO,CLEAR_SCREE_ANSI,12);
}


char normalize(unsigned int distance) {
    const int  max_dist[]  =  { 100, 150, 200, 250, 300, 400, 500, 1000 };
    const char disp[]      =  { '@', 'O', '0', '*', 'o', 'c', '+', '.'  };
    int i;
    for (i=0;i<8;i++) {
        if (distance<max_dist[i])
            return disp[i];
    }
    return ' ';
}

 
void* read_sensor_loop(void *arg) {
    static int i,j,laser_ind,laser_angle,laser_dist,i1,j1;
    static u_int16_t angle_max = 37000,angle_last;
    double tt,dt,ts,ts0,t,a,a_last;

    static float v_angle[] = { -30.67, -9.33, -29.33, -8.00, -28.00, -6.66, -26.66, -5.33,
                               -25.33, -4.00, -24.00, -2.67, -22.67, -1.33, -21.33,  0.00,
                               -20.00,  1.33, -18.67,  2.67, -17.33,  4.00, -16.00,  5.33,
                               -14.67,  6.67, -13.33,  8.00, -12.00,  9.33, -10.67, 10.67  }; 
    static float v_angle_ind[] = { 31, 15, 30, 14, 29, 13, 28, 12, 
                                   27, 11, 26, 10, 25,  9, 24,  8,
                                   23,  7, 22,  6, 21,  5, 20,  4,
                                   19,  3, 18,  2, 17,  1, 16,  0  };

    // HDL-32E packet data structure
    struct point {
        u_int16_t     d;
        unsigned char i;
    } __attribute__((packed));
    struct block {
        u_int16_t     bid;
        u_int16_t     angle;
        struct point  p[32];     // 32 laser shoots (1 vertical line)
    } __attribute__((packed));
    struct hdl32e_pos {
        struct block  b[12];     // block of 12 32-shoots series
        unsigned int  gps_ms;    // GPS timestamp at the and of block laser shooting
        unsigned char unused_[2];
    } __attribute__((packed));


// read sensor loop
    int saddr_size, size;
    struct sockaddr saddr;
         
    unsigned char *buffer = (unsigned char *) malloc(65536); 
     
    printf("Starting...\n");
     
    int sock_raw = socket( AF_PACKET , SOCK_RAW , htons(ETH_P_ALL)) ;
    setsockopt(sock_raw , SOL_SOCKET , SO_BINDTODEVICE , "eth0" , strlen("eth0")+ 1 );
     
    if(sock_raw < 0)
    {
        //Print the error with proper message
        perror("Socket Error");
        return;
    }
    while(1)
    {
        saddr_size = sizeof saddr;
        //Receive a packet
        size = recvfrom(sock_raw , buffer , 65536 , 0 , &saddr , (socklen_t*)&saddr_size);
        if(size < 0 )
        {
            printf("Recvfrom error , failed to get packets\n");
            return;
        }

        //Now process the packet
        //Get the IP Header part of this packet , excluding the ethernet header
        struct iphdr *iph = (struct iphdr*)(buffer + sizeof(struct ethhdr));
        unsigned short iphdrlen = iph->ihl*4;
        struct udphdr *udph = (struct udphdr*)(buffer + iphdrlen  + sizeof(struct ethhdr));
        struct hdl32e_pos *ps = (struct hdl32e_pos*)(buffer+42);
        int dport;

        switch (iph->protocol) //Check the Protocol and do accordingly...
        {
            case 17: //UDP Protocol
                dport = ntohs(udph->dest);

                if (dport==2368) {        // only HDL-32E data packet

                    tt = 552.96e-6;       // milliseconds per block (12*32 laser firings)
                    dt = tt/384;          // milliseconds per single laser firing
                    ts = ps->gps_ms;      // timestamp at the end of block (milliseconds)
                    ts0 = ts - tt;        // timestamp at the start of block (milliseconds)


                    pthread_mutex_lock(&lock);

                    t = ts0;
                    for (i=0;i<12;i++) {

                        a = 360.0 * ps->b[i].angle / angle_max;
                        if (a_last>a) {
                            cls();
                            for (j1=0;j1<32;j1++) {
                               for (i1=0;i1<36;i1++) {
                                   int c = normalize(grid[i1][j1]);
                                   printf("%c%c%c", c,c,c);
                                   grid[i1][j1] = 0xFFFF;
                               }
                               printf("\n");
                            }
                        }

                        for (j=0;j<32;j++) {
                            laser_angle = (int)(a/10);
                            laser_ind = v_angle_ind[j];
                            laser_dist = ps->b[i].p[j].d / 5;       // distance in cm

                            // save grid
                            if ( laser_dist < grid[laser_angle][laser_ind])
                                grid[laser_angle][laser_ind] = laser_dist;
                            t += dt;
                        }
                        a_last = a;
                        angle_last = ps->b[i].angle;
                    }

                    pthread_mutex_unlock(&lock);

                }

                break;
        }
    }
    close(sock_raw);
    printf("Finished");

    return 0;
}

void* display_scene_loop(void *arg) {
    static int i,j;
return;
    while(1) { 

        pthread_mutex_lock(&lock);

        for (j=0;j<32;j++) {
            for (i=0;i<36;i++) {
                printf("%x ", grid[i][j]);
            }
            printf("\n");
        }

        pthread_mutex_unlock(&lock);

        usleep(300000);   
    }

    return 0;
}

int main() {
    int i = 0;
    int err;

    if (pthread_mutex_init(&lock, NULL) != 0)
    {
        printf("\n mutex init failed\n");
        return 1;
    }

    // start sensor reading routine with calling read_sensor_data() for each packet;
    err = pthread_create(&tid_sensor_loop, NULL, &read_sensor_loop, NULL);
    if (err != 0)
        printf("\ncan't create thread :[%s]", strerror(err));

    // start scene display loop
    err = pthread_create(&tid_scene_loop, NULL, &display_scene_loop, NULL);
    if (err != 0)
        printf("\ncan't create thread :[%s]", strerror(err));

    pthread_join(tid_sensor_loop, NULL);
    pthread_join(tid_scene_loop, NULL);

    pthread_mutex_destroy(&lock);

    return 0;
}


