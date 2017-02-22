#include "../inc/TCP_server.h"
#include "../inc/OpencvCamera.h"
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
	
	// point pc_file to NULL at first
	for (int i = 0; i < NUM_OF_CLIENTS; i++){
		pc_file[i] = NULL;
	}
    
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
    static int connection_number = 0;
    socklen_t addr_size = 0;
    num_clients = 0;
    sockaddr_in sadr;
    addr_size = sizeof(sockaddr_in);
    
    struct pollfd fds;
    fds.fd = hsock;
    fds.events = POLLIN;

    int poll_timeout = 1000;
    
    printf("Waiting for connections\r\n");    
    while(!quit_wait_thread){
        int poll_ret = poll(&fds,1,poll_timeout);                
        if(poll_ret > 0)
        {    
            if((fd_client[num_clients++] = accept( hsock, (sockaddr*)&sadr, &addr_size))!= -1){
                printf("Connection #%d received from %s\r\n",++connection_number, inet_ntoa(sadr.sin_addr));
                printf("Number of clients connected: %d\r\n", num_clients );
            }
            else{
    			fprintf(stderr, "Error accepting %d\n", errno);
    		} 
            
			if (pc_file[num_clients - 1] == NULL){
				pc_file[num_clients - 1] = new Computer_channel();
				pc_file[num_clients - 1]->instantiate_computer(num_clients, fd_client[num_clients - 1]);
				pc_connected[num_clients - 1] = true;
            
				// need to register signal here too
				callbackConnection[num_clients - 1] = pc_file[num_clients - 1]->getCallbackSignal()->connect(boost::bind(&TCP_server::ComputerChannelCallback,this,_1,_2));
            
				// call main to open camera
				main_callback_signal(CONNECTION_MADE_RELAY);				
			}
			else
			{
				std::cout << "Trying to write to memory already allocated" << std::endl;
			}

        }
    }
}

void TCP_server::disconnectSignals(int id){
    callbackConnection[id].disconnect();
}

void TCP_server::ComputerChannelCallback(relay_callback_msg msg , int id){
    switch (msg){
        case CLIENT_DISCONNECT_RELAY:
            disconnectSignals(id - 1);
            delete pc_file[id - 1];
            pc_file[id - 1] = NULL;
            num_clients--;
            std::cout << "Client disconnected " << id << endl;
            
            // call main to disconnect camera
            main_callback_signal(CLIENT_DISCONNECT_RELAY);            
            break;
            
        case GET_DATA_RELAY:
            //std::cout << "CAPTURE RECEIVED " << id << endl;
            main_callback_signal(GET_DATA_RELAY);  
            break;
            
        case START_AUTO_RELAY:
            //std::cout << "CAPTURE RECEIVED " << id << endl;
            main_callback_signal(START_AUTO_RELAY);  
            break;   
            
        case STOP_AUTO_RELAY:
            //std::cout << "CAPTURE RECEIVED " << id << endl;
            main_callback_signal(STOP_AUTO_RELAY);  
            break;               
            
        case WRITE_FAIL_RELAY:
            std::cout << "DBG: Write Fail " << id << endl;
            break;  
            
        case CALIBRATE_RELAY:
            main_callback_signal(CALIBRATE_RELAY);  
            break;              
            
        case KILL_PROGRAM_RELAY:
            main_callback_signal(KILL_PROGRAM_RELAY);  
            break;
            
        default:
            break;
    }
}

int TCP_server::getNumClients(void){
    return num_clients;
}

void TCP_server::write2clients(char * str){
    for (int i = 0; i < num_clients; i++){
        pc_file[i]->write2client(str);
    }
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

TCP_server::~TCP_server() {
    for (int i = 0; i < num_clients; i++){
        //disconnectSignals(i);
        if (pc_file[i] != NULL)
	{
		delete pc_file[i];
		pc_file[i] = NULL;
	}
    }    
}


