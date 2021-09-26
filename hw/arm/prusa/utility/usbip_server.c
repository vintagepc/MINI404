#include "usbip_server.h"
#include "usbip.h"

static void pack(int * data, int size)
{
   int i;
   size=size/4;
   for(i=0;i<size;i++)
   {
      data[i]=htonl(data[i]);
   }
   //swap setup
   i=data[size-1];
   data[size-1]=data[size-2];
   data[size-2]=i;
}  

/* Device Descriptor */
const USB_DEVICE_DESCRIPTOR dev_dsc=
{
    0x12,                   // Size of this descriptor in bytes
    0x01,                   // DEVICE descriptor type
    0x0200,                 // USB Spec Release Number in BCD format
    0x02,                   // Class Code
    0x00,                   // Subclass code
    0x00,                   // Protocol code
    0x10,                   // Max packet size for EP0, see usb_config.h
    0x2706,                 // Vendor ID
    0x000A,                 // Product ID
    0x0100,                 // Device release number in BCD format
    0x01,                   // Manufacturer string index
    0x03,                   // Product string index
    0x00,                   // Device serial number string index
    0x01                    // Number of possible configurations
};
const USB_DEVICE_QUALIFIER_DESCRIPTOR dev_qua = { 
    0x0A,       // bLength
    0x06,       // bDescriptorType
    0x0200,     // bcdUSB 
    0x02,       // bDeviceClass
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocol
    0x10,       // bMaxPacketSize
    0x01,       // iSerialNumber
    0x00        //bNumConfigurations*/
};


/* Configuration 1 Descriptor */
const CONFIG_CDC  configuration_cdc={{
    /* Configuration Descriptor */
    0x09,//sizeof(USB_CFG_DSC),    // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                // CONFIGURATION descriptor type
    sizeof(CONFIG_CDC),                 // Total length of data for this cfg
    2,                      // Number of interfaces in this cfg
    1,                      // Index value of this configuration
    0,                      // Configuration string index
    0xC0,      
    50,                     // Max power consumption (2X mA)
    },{ 
    /* Interface Descriptor */
    0x09,//sizeof(USB_INTF_DSC),   // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,               // INTERFACE descriptor type
    0,                      // Interface Number
    0,                      // Alternate Setting Number
    1,                      // Number of endpoints in this intf
    0x02,                   // Class code
    0x02,                   // Subclass code
    0x01,                   // Protocol code
    0                       // Interface string index
    },{
    /* CDC Class-Specific Descriptors */
    /* Header Functional Descriptor */    
    0x05,                   // bFNLength
    0x24,                   // bDscType
    0x00,                   // bDscSubType
    0x0110                  // bcdCDC
    },{ 
    /* Call Management Functional Descriptor */   
    0x05,                   // bFNLength
    0x24,                   // bDscType
    0x01,                   // bDscSubType
    0x01,                   // bmCapabilities
    0x01                    // bDataInterface
    },{
    /* Abstract Control Management Functional Descriptor */    
    0x04,                   // bFNLength
    0x24,                   // bDscType
    0x02,                   // bDscSubType 
    0x02                    // bmCapabilities 
    },{
    /* Union Functional Descriptor */    
    0x05,                   // bFNLength
    0x24,                   // bDscType
    0x06,                   // bDscSubType
    0x00,                   // bMasterIntf
    0x01                    // bSaveIntf0
    },{
     /* Endpoint Descriptor */
    0x07,/*sizeof(USB_EP_DSC)*/
    USB_DESCRIPTOR_ENDPOINT,    //Endpoint Descriptor
    0x81,                       //EndpointAddress
    0x03,                       //Attributes
    0x0008,                     //size
    0x0A                        //Interval
    },{    
    /* Interface Descriptor */
    0x09,//sizeof(USB_INTF_DSC),   // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,               // INTERFACE descriptor type
    1,                      // Interface Number
    0,                      // Alternate Setting Number
    2,                      // Number of endpoints in this intf
    0x0A,                   // Class code
    0x00,                   // Subclass code
    0x00,                   // Protocol code
    2,                      // Interface string index
    },{
    /* Endpoint Descriptor */
    0x07,/*sizeof(USB_EP_DSC)*/
    USB_DESCRIPTOR_ENDPOINT,    //Endpoint Descriptor
    0x02,                       //EndpointAddress
    0x02,                       //Attributes
    0x0020,                     //size
    0x00                        //Interval
    },{
    /* Endpoint Descriptor */
    0x07,/*sizeof(USB_EP_DSC)*/
    USB_DESCRIPTOR_ENDPOINT,    //Endpoint Descriptor
    0x82,                       //EndpointAddress
    0x02,                       //Attributes
    0x0020,                     //size
    0x00                        //Interval
}};


const unsigned char string_0[] = { // available languages  descriptor
		0x04,                  
                USB_DESCRIPTOR_STRING, 
		0x09, 
                0x04 
		};

const unsigned char string_1[] = { //
		0x0A, 
                USB_DESCRIPTOR_STRING, // bLength, bDscType
		'T', 0x00, //
		'e', 0x00, //
		's', 0x00, //
		't', 0x00, //
		};

const unsigned char string_2[] = { //
		0x10, 
                USB_DESCRIPTOR_STRING, //
		'U', 0x00, //
		'S', 0x00, //
		'B', 0x00, //
		' ', 0x00, //
		'C', 0x00, //
		'D', 0x00, //
		'C', 0x00, //
		};

const unsigned char string_3[] = { //
		0x18, 
                USB_DESCRIPTOR_STRING, //
		'V', 0x00, //
		'i', 0x00, //
		'r', 0x00, //
		't', 0x00, //
		'u', 0x00, //
		'a', 0x00, //
		'l', 0x00, //
		' ', 0x00, //
                'U', 0x00, //
                'S', 0x00, //
                'B', 0x00, //
		};


const char *configuration = (const char *)&configuration_cdc; 

const USB_INTERFACE_DESCRIPTOR *interfaces[]={ &configuration_cdc.dev_int0, &configuration_cdc.dev_int1};

const unsigned char *strings[]={string_0, string_1, string_2, string_3};


static void unpack(int * data, int size)
{
   int i;
   size=size/4;
   for(i=0;i<size;i++)
   {
      data[i]=ntohl(data[i]);
   }
   //swap setup
   i=data[size-1];
   data[size-1]=data[size-2];
   data[size-2]=i;
}  

static void handle_device_list(const USB_DEVICE_DESCRIPTOR *dev_dsc, OP_REP_DEVLIST *list)
{
  CONFIG_GEN * conf= (CONFIG_GEN *)configuration;   
  int i;
  list->header.version=htons(273);
  list->header.command=htons(5);
  list->header.status=0;
  list->header.nExportedDevice=htonl(1);
  memset(list->device.usbPath,0,256);
  strcpy(list->device.usbPath,"/sys/devices/pci0000:00/0000:00:01.2/usb1/1-1");
  memset(list->device.busID,0,32);
  strcpy(list->device.busID,"1-1");
  list->device.busnum=htonl(1);
  list->device.devnum=htonl(2);
  list->device.speed=htonl(2);
  list->device.idVendor=htons(dev_dsc->idVendor);
  list->device.idProduct=htons(dev_dsc->idProduct);
  list->device.bcdDevice=htons(dev_dsc->bcdDevice);
  list->device.bDeviceClass=dev_dsc->bDeviceClass;
  list->device.bDeviceSubClass=dev_dsc->bDeviceSubClass;
  list->device.bDeviceProtocol=dev_dsc->bDeviceProtocol;
  list->device.bConfigurationValue=conf->dev_conf.bConfigurationValue;
  list->device.bNumConfigurations=dev_dsc->bNumConfigurations; 
  list->device.bNumInterfaces=conf->dev_conf.bNumInterfaces;
  list->interfaces=malloc(list->device.bNumInterfaces*sizeof(OP_REP_DEVLIST_INTERFACE));
  for(i=0;i<list->device.bNumInterfaces;i++)
  {     
    list->interfaces[i].bInterfaceClass=interfaces[i]->bInterfaceClass;
    list->interfaces[i].bInterfaceSubClass=interfaces[i]->bInterfaceSubClass;
    list->interfaces[i].bInterfaceProtocol=interfaces[i]->bInterfaceProtocol;
    list->interfaces[i].padding=0;
  }
};

static void handle_attach(const USB_DEVICE_DESCRIPTOR *dev_dsc, OP_REP_IMPORT *rep)
{
  CONFIG_GEN * conf= (CONFIG_GEN *)configuration; 
    
  rep->version=htons(273);
  rep->command=htons(3);
  rep->status=0;
  memset(rep->usbPath,0,256);
  strcpy(rep->usbPath,"/sys/devices/pci0000:00/0000:00:01.2/usb1/1-1");
  memset(rep->busID,0,32);
  strcpy(rep->busID,"1-1");
  rep->busnum=htonl(1);
  rep->devnum=htonl(2);
  rep->speed=htonl(2);
  rep->idVendor=dev_dsc->idVendor;
  rep->idProduct=dev_dsc->idProduct;
  rep->bcdDevice=dev_dsc->bcdDevice;
  rep->bDeviceClass=dev_dsc->bDeviceClass;
  rep->bDeviceSubClass=dev_dsc->bDeviceSubClass;
  rep->bDeviceProtocol=dev_dsc->bDeviceProtocol;
  rep->bNumConfigurations=dev_dsc->bNumConfigurations; 
  rep->bConfigurationValue=conf->dev_conf.bConfigurationValue;
  rep->bNumInterfaces=conf->dev_conf.bNumInterfaces;
}

extern bool usbip_read_payload(usbip_cfg_t* cfg, char* buffer, unsigned int size)
{
    int nb;
    if ((nb = recv (cfg->_private_fd, buffer , size, 0)) != size)
    {
        printf ("read_payload recv error : %s %d of %d bytes \n", strerror (errno), nb, size);
        return false;
    };
    return true;
}

 extern void usbip_send_reply(usbip_cfg_t *cfg, USBIP_RET_SUBMIT * usb_req, const char * data, unsigned int size, unsigned int status)
{
        usb_req->command=0x3;
        usb_req->status=status;
        usb_req->actual_length=size;
        usb_req->start_frame=0x0;
        usb_req->number_of_packets=0x0;
	
        usb_req->setup=0x0;
        usb_req->devid=0x0;
	    usb_req->direction=0x0;
        usb_req->ep=0x0;    
    
        pack((int *)usb_req, sizeof(USBIP_RET_SUBMIT));
 
        if (send (cfg->_private_fd, (char *)usb_req, sizeof(USBIP_RET_SUBMIT), 0) != sizeof(USBIP_RET_SUBMIT))
        {
          printf ("send error : %s \n", strerror (errno));
          exit(-1);
        };

        if(size > 0)
        {
           if (send (cfg->_private_fd, data, size, 0) != size)
           {
             printf ("send error : %s \n", strerror (errno));
             exit(-1);
           };
        }
} 

void * usbip_thread_run(void* run_info)
{

    usbip_cfg_t* cfg = (usbip_cfg_t*)run_info;
  struct sockaddr_in serv, cli;
  int listenfd, sockfd, nb;
#ifdef __linux__
  unsigned int clilen;
#else
  int clilen;
#endif
  unsigned char attached;



#ifndef __linux__
  WSAStartup (wVersionRequested, &wsaData);
  if (wsaData.wVersion != wVersionRequested)
    {
      fprintf (stderr, "\n Wrong version\n");
      exit (-1);
    }

#endif

  if ((listenfd = socket (PF_INET, SOCK_STREAM, 0)) < 0)
    {
      printf ("socket error : %s \n", strerror (errno));
      exit (1);
    };

  int reuse = 1;
  if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0)
      perror("setsockopt(SO_REUSEADDR) failed");

  memset (&serv, 0, sizeof (serv));
  serv.sin_family = AF_INET;
  serv.sin_addr.s_addr = htonl (INADDR_ANY);
  serv.sin_port = htons (cfg->port);

  if (bind (listenfd, (sockaddr *) & serv, sizeof (serv)) < 0)
    {
      printf ("bind error : %s \n", strerror (errno));
      exit (1);
    };

  if (listen (listenfd, SOMAXCONN) < 0)
    {
      printf ("listen error : %s \n", strerror (errno));
      exit (1);
    };

  for (;;)
    {

      clilen = sizeof (cli);
      if (
          (sockfd =
           accept (listenfd, (sockaddr *) & cli,  & clilen)) < 0)
        {
          printf ("accept error : %s \n", strerror (errno));
          exit (1);
        };
        printf("Connection address:%s\n",inet_ntoa(cli.sin_addr));
        attached=0;
        cfg->_private_fd = sockfd;
        while(1)
        {
          if(! attached)
          {
             OP_REQ_DEVLIST req;
             if ((nb = recv (sockfd, (char *)&req, sizeof(OP_REQ_DEVLIST), 0)) != sizeof(OP_REQ_DEVLIST))
             {
               //printf ("receive error : %s \n", strerror (errno));
               break;
             };
#ifdef _DEBUG
             print_recv((char *)&req, sizeof(OP_REQ_DEVLIST),"OP_REQ_DEVLIST");
#endif
             req.command=ntohs(req.command);
             printf("Header Packet\n");  
             printf("command: 0x%02X\n",req.command);
             if(req.command == 0x8005)
             {
               OP_REP_DEVLIST list;
               printf("list of devices\n");

               handle_device_list(&dev_dsc,&list);

               if (send (sockfd, (char *)&list.header, sizeof(OP_REP_DEVLIST_HEADER), 0) != sizeof(OP_REP_DEVLIST_HEADER))
               {
                   printf ("send error : %s \n", strerror (errno));
                   break;
               };
               if (send (sockfd, (char *)&list.device, sizeof(OP_REP_DEVLIST_DEVICE), 0) != sizeof(OP_REP_DEVLIST_DEVICE))
               {
                   printf ("send error : %s \n", strerror (errno));
                   break;
               };
               if (send (sockfd, (char *)list.interfaces, sizeof(OP_REP_DEVLIST_INTERFACE)*list.device.bNumInterfaces, 0) != sizeof(OP_REP_DEVLIST_INTERFACE)*list.device.bNumInterfaces)
               {
                   printf ("send error : %s \n", strerror (errno));
                   break;
               };
               free(list.interfaces);
             }
             else if(req.command == 0x8003) 
             {
               char busid[32];
               OP_REP_IMPORT rep;
               printf("attach device\n");
               if ((nb = recv (sockfd, busid, 32, 0)) != 32)
               {
                 printf ("receive error : %s \n", strerror (errno));
                 break;
               };
#ifdef _DEBUG
             print_recv(busid, 32,"Busid");
#endif
               handle_attach(&dev_dsc,&rep);
               if (send (sockfd, (char *)&rep, sizeof(OP_REP_IMPORT), 0) != sizeof(OP_REP_IMPORT))
               {
                   printf ("send error : %s \n", strerror (errno));
                   break;
               };
               attached = 1;
             }
          }
          else
          {
             printf("------------------------------------------------\n"); 
             printf("handles requests\n");
             USBIP_CMD_SUBMIT cmd;
             USBIP_RET_SUBMIT usb_req;
             if ((nb = recv (sockfd, (char *)&cmd, sizeof(USBIP_CMD_SUBMIT), 0)) != sizeof(USBIP_CMD_SUBMIT))
             {
               printf ("receive error : %s nb: %d \n", strerror (errno), nb);
               break;
             };
#ifdef _DEBUG
             print_recv((char *)&cmd, sizeof(USBIP_CMD_SUBMIT),"USBIP_CMD_SUBMIT");
#endif
             unpack((int *)&cmd,sizeof(USBIP_CMD_SUBMIT));               
             printf("usbip cmd %u\n",cmd.command);
             printf("usbip seqnum %u\n",cmd.seqnum);
             printf("usbip devid %u\n",cmd.devid);
             printf("usbip direction %u\n",cmd.direction);
             printf("usbip ep %u\n",cmd.ep);
             printf("usbip flags %u\n",cmd.transfer_flags);
             printf("usbip number of packets %u\n",cmd.number_of_packets);
             printf("usbip interval %u\n",cmd.interval);
            //  printf("usbip setup %"PRI"\n",cmd.setup);
             printf("usbip buffer lenght  %u\n",cmd.transfer_buffer_length);
             usb_req.command=0;
             usb_req.seqnum=cmd.seqnum;
             usb_req.devid=cmd.devid;
             usb_req.direction=cmd.direction;
             usb_req.ep=cmd.ep;
             usb_req.status=0;
             usb_req.actual_length=0;
             usb_req.start_frame=0;
             usb_req.number_of_packets=0;
             usb_req.error_count=0;
             usb_req.setup=cmd.setup;
             
             if(cmd.command == 1)
             {
                printf("usb_req ep %d\n",usb_req.ep);
                USBIPServerClass *s = USBIP_SERVER_GET_CLASS(cfg->self);
                s->usbip_handle_packet(cfg->self, &cmd, &usb_req);
             }
               //handle_usb_request(sockfd, &usb_req, cmd.transfer_buffer_length);
             

             if(cmd.command == 2) //unlink urb
             {
                printf("####################### Unlink URB %u  (not working!!!)\n",cmd.transfer_flags);
             //FIXME
               /*              
                USBIP_RET_UNLINK ret;  
                printf("####################### Unlink URB %u\n",cmd.transfer_flags);
                ret.command=htonl(0x04);
                ret.devid=htonl(cmd.devid);
                ret.direction=htonl(cmd.direction);
                ret.ep=htonl(cmd.ep);
                ret.seqnum=htonl(cmd.seqnum);
                ret.status=htonl(1);
 
                if (send (sockfd, (char *)&ret, sizeof(USBIP_RET_UNLINK), 0) != sizeof(USBIP_RET_UNLINK))
                {
                  printf ("send error : %s \n", strerror (errno));
                  exit(-1);
                };
               */ 
             }

             if(cmd.command > 2)
             {
                printf("Unknown USBIP cmd!\n");  
                close (sockfd);
                cfg->_private_fd = 0;
#ifndef __linux__
                WSACleanup ();
#endif
                return NULL;  
             };
 
          } 
       }
       close (sockfd);
        cfg->_private_fd = 0;
    };
#ifndef __linux__
  WSACleanup ();
#endif
};

static const TypeInfo usbip_server_type_info = {
    .name = TYPE_USBIP_SERVER,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(USBIPServerClass),
};

static void usbip_server_register_types(void)
{
    type_register_static(&usbip_server_type_info);
}

type_init(usbip_server_register_types)
