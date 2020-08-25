#include "mbCommon.h"

void print_recv_Data(int16_t*array, int count,bool bAll)
{
    for(int i = 0; i < count; i++)
    {
        if((bAll || array[i] != 0) /*&& i+READADDRESS!= 59*/)
            cout<<"recv [Address] : "<<i+READADDRESS<<" , [value] : "<<array[i]<<endl;
    }
}

void print_send_Data(uint16_t*array, int count,bool bAll)
{
    for(int i = 0; i < MSGLEN; i++)
    {
        if((bAll || array[i] != 0) /*&& i+WRITEADDRESS != 119*/)
            cout<<"send [Address] : "<<i+WRITEADDRESS<<" , [value] : "<<array[i]<<endl;
    }
}
