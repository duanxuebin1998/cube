#ifndef __HOSTCOMMU_H
#define __HOSTCOMMU_H

#define HOSTCOMMU_SENDLENGTH 120

int HostCommuInit(void);
void HostCommuProcess(char *rcvbuff, int rcvcount);

#endif	  

