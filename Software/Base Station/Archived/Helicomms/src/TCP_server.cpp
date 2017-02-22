/* 
 * File:   TCP_server.cpp
 * Author: yashren
 * 
 * Created on 04 May 2013, 12:15 PM
 */

#include "../include/TCP_server.h"
#include "../include/OpencvCamera.h"
#include <locale.h>

extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_wait(void* arg)
    {
        TCP_server* t = static_cast<TCP_server*>(arg);
        t->waiting4clients();
        return 0;
    }
}

TCP_server::TCP_server(int hostPort) {
    host_port = hostPort;
    
    memset(&fd_client[0],0,NUM_OF_CLIENTS);
    memset(&pc_connected[0],0,NUM_OF_CLIENTS);
        
    hsock = socket(AF_INET, SOCK_STREAM, 0);    
	
    if(hsock == -1){
		printf("Error initializing TCP server socket %d\n", errno);
        exit(0);
	}    
    
    int * p_int;
	p_int = (int*)malloc(sizeof(int));
	*p_int = 1;
		
	if( (setsockopt(hsock, SOL_SOCKET, SO_REUSEADDR, (char*)p_int, sizeof(int)) == -1 )||
		(setsockopt(hsock, SOL_SOCKET, SO_KEEPALIVE, (char*)p_int, sizeof(int)) == -1 ) ){
		printf("Error setting options %d\n", errno);
		free(p_int);
		exit(0);
	}
	free(p_int);    
    
	my_addr.sin_family = AF_INET ;
	my_addr.sin_port = htons(host_port);    
    
    memset(&(my_addr.sin_zero), 0, 8);
	my_addr.sin_addr.s_addr = INADDR_ANY ;
    
	if( bind( hsock, (sockaddr*)&my_addr, sizeof(my_addr)) == -1 ){
		fprintf(stderr,"Error binding to socket, make sure nothing else is listening on this port %d\n",errno);
		exit(0);
	}
	if(listen( hsock, NUM_OF_CLIENTS) == -1 ){
		fprintf(stderr, "Error listening %d\n",errno);
		exit(0);
	}    
        
    quit_wait_thread = false;
    // if server setup was successful, here we run a thread to wait for clients
    if (pthread_create(&wait_4_client_thread, 0, &thread_catch_wait, this) != 0)
    {
        fprintf(stderr, "Error creating listening thread %d\n",errno);
    }        
}

void TCP_server::waiting4clients(void){
    socklen_t addr_size = 0;
    num_clients = 0;
    sockaddr_in sadr;
    addr_size = sizeof(sockaddr_in);
    
    struct pollfd fds;
    fds.fd = hsock;
    fds.events = POLLIN;

    int poll_timeout = 1000;
    
    std::cout << "waiting4clients thread started" << std::endl;
    printf("Waiting for connections\r\n");    
    while(!quit_wait_thread){
        int poll_ret = poll(&fds,1,poll_timeout);                
        if(poll_ret > 0)
        {    
            if((fd_client[num_clients++] = accept( hsock, (sockaddr*)&sadr, &addr_size))!= -1){
                printf("---------------------\r\nReceived connection from %s\r\n",inet_ntoa(sadr.sin_addr));
                printf("Number of clients connected: %d\r\n", num_clients );
            }
            else{
    			fprintf(stderr, "Error accepting %d\n", errno);
    		} 
            
            pc_file[num_clients - 1] = new Computer_channel();
            pc_file[num_clients - 1]->instantiate_computer(num_clients, fd_client[num_clients - 1]);

            pc_connected[num_clients - 1] = true;
            
            // need to register signal here too
            channelLostConnection[num_clients - 1] = pc_file[num_clients - 1]->getChannelLostSignal()->connect(boost::bind(&TCP_server::signalLost,this));
            dataReceivedConnection[num_clients - 1] = pc_file[num_clients - 1]->getDataReceivedSignal()->connect(boost::bind(&TCP_server::displayReceivedData,this,_1));
            
            
        }
    }
    std::cout << "waiting4clients thread terminated" << std::endl;
}

void TCP_server::write2allClients(char * str){
    for (int i = 0; i < num_clients; i++){
        pc_file[i]->write2client(str);
    }      
}

void TCP_server::displayReceivedData(char * data){
    // relay signal one up to Window
    dataReceived(data);
    std::cout << data << " " << pc_file[0]->getChannelDelay() << std::endl;
}

void TCP_server::signalLost(){
    std::cout << "Signal lost" << std::endl;
}

bool TCP_server::channelExists(int channel){
    if ((channel > (NUM_OF_CLIENTS + 1)) || (channel < 1))
        return false;
    
    if (pc_file[channel -1]->getID() == channel){
        return true;
    }
    else
        return false;
}

bool TCP_server::channelConnected(int channel){
    return pc_connected[channel];
}

void TCP_server::stopReceiveThread(void){
    for (int i = 0; i < num_clients; i++){
        pc_file[i]->stopReceiveThread();
    }
    
    quit_wait_thread = true;  
    // wait for thread to end
    pthread_join(wait_4_client_thread, NULL);        
}



Computer_channel * TCP_server::getCommChannel(int id){
    if (pc_file[id] != NULL)
        return pc_file[id];
    else
        return NULL;
}

int TCP_server::getNumClients(void){
    return num_clients;
}

TCP_server::~TCP_server() {
    for (int i = 0; i < num_clients; i++){
        pc_file[i]->send2client(Computer_channel::TERMINATE);
        channelLostConnection[i].disconnect();
        dataReceivedConnection[i].disconnect();
        pc_file[i]->~Computer_channel();
        close(fd_client[i]);
        //delete pc_file[i];
    }    
}


