#include <stdio.h>
#include <string.h>
#include <stdlib.h>


//#include <sys/time.h>
#include "datatype.h"
#include "cmd.h"

typedef struct	frame_info{
	   uint32 bytes_received;
	   uint32 frame_size;
	   }frame_info;

#define CRC_INIT 0xffff 
#define GOOD_CRC 0xf0b8
int count=0,frame_count=0;
unsigned int lost_count=0;
int m=0;
unsigned int frame_time_last=0;
FILE *fp ,*fp_fly_status,*fp_servo_test;
unsigned int gap=0;
unsigned char mode=1;
unsigned int data_refresh_rate=20;
unsigned char imu_type = 0;
unsigned char generate_servo_test=0;

float sonar_data_original_last=22;
float sonar_data_processed_last=22;
float sonar_filter_x1k=0;
float sonar_filter_x2k=0;
#define SONAR_FILTER_T  0.02
#define SONAR_FILTER_A 20

int sonar_data_filter(flying_status_s *flying_status)
{
   int i=0;
   float sonar_data_processed;
   float sonar_data_preprocess;
   float sonar_data_original;
   float dx1k,dx2k;
   sonar_data_original=((float)flying_status->sonar_h)/58;

   if((sonar_data_original-sonar_data_original_last >= 5)||(sonar_data_original-sonar_data_processed_last >= 5) ){

         sonar_data_preprocess=sonar_data_processed_last+5 ;
  
   }else if((sonar_data_original-sonar_data_original_last <=-5)||(sonar_data_original-sonar_data_processed_last <= -5) ){

         sonar_data_preprocess=sonar_data_processed_last -5;

   }else{
       sonar_data_preprocess=sonar_data_original;
   }
   sonar_data_processed = sonar_filter_x1k;
   dx1k=sonar_filter_x2k;
   dx2k=-2*SONAR_FILTER_A*sonar_filter_x2k-SONAR_FILTER_A*SONAR_FILTER_A*(sonar_filter_x1k-sonar_data_preprocess);
   sonar_filter_x1k=sonar_filter_x1k+dx1k*SONAR_FILTER_T;
   sonar_filter_x2k=sonar_filter_x2k+dx2k*SONAR_FILTER_T;
   
   /*
   if((sonar_data_original-sonar_data_original_last >= 5)||(sonar_data_original-sonar_data_processed_last >= 5) ){
      if(sonar_data_original >= 770)
         sonar_data_processed=sonar_data_processed_last;
      else
         sonar_data_processed=sonar_data_processed_last + 5;
  
   }else if((sonar_data_original-sonar_data_original_last <=-5)||(sonar_data_original-sonar_data_processed_last <= -5) ){
      if(sonar_data_original_last >= 770)
         sonar_data_processed=sonar_data_processed_last;
      else
         sonar_data_processed=sonar_data_processed_last - 5;

   }else{
     sonar_data_processed=sonar_data_original;
   }
*/
   sonar_data_original_last = sonar_data_original;
   //sonar_data_processed_last = sonar_data_processed;
   sonar_data_processed_last = sonar_data_preprocess;
   return sonar_data_processed;
//return sonar_data_preprocess;
}



static unsigned short crc16_ccitt_table[256] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

unsigned short crc_checksum16(unsigned char *message, unsigned int len)
{
	unsigned short crc_reg = CRC_INIT;
	while (len--)
		crc_reg = (crc_reg >> 8) ^ crc16_ccitt_table[(crc_reg ^ *message++) & 0xff];
	return crc_reg;
}
void flying_status_parse(uint8 *data,flying_status_s *flying_status)
{

    flying_status->roll = *(float*)data;

    flying_status->pitch =  *(float*)(data +4);
    flying_status->yaw =  *(float*)(data +8);
    flying_status->gx =  *(float*)(data +12);
    flying_status->gy =  *(float*)(data +16);
    flying_status->gz =  *(float*)(data +20);
    flying_status->ax =  *(float*)(data +24);
    flying_status->ay =  *(float*)(data +28);
    flying_status->az =  *(float*)(data +32);
    flying_status->g_time =  *(uint32*)(data +36);
    flying_status->vn =  *(int*)(data +40);
    flying_status->ve =  *(int*)(data +44);
    flying_status->vd =  *(int*)(data +48);
    flying_status->heading =  *(int*)(data +52);
    flying_status->b_h =  *(int*)(data +56);
    flying_status->lat =  *(double*)(data +60);
    flying_status->Long =  *(double*)(data +68);
    flying_status->g_h =  *(double*)(data +76);
    flying_status->vx =  *(float*)(data +84);
    flying_status->vy =  *(float*)(data +88);
    flying_status->vz =  *(float*)(data +92);
    flying_status->sonar_h =  *(uint32*)(data +96);
    flying_status->waypoint_dest =  *(uint32*)(data +100);
    flying_status->pwm[0] = *(uint16*)(data+102);
    flying_status->pwm[1] = *(uint16*)(data+104);
    flying_status->pwm[2] = *(uint16*)(data+106);
    flying_status->pwm[3] = *(uint16*)(data+108);
    flying_status->pwm[4] = *(uint16*)(data+110);
    flying_status->pwm[5] = *(uint16*)(data+112);
    flying_status->pwm[6] = *(uint16*)(data+114);
    flying_status->pwm[7] = *(uint16*)(data+116);
    flying_status->pwm[8] = *(uint16*)(data+118);
    flying_status->pwm[9] = *(uint16*)(data+120);
    flying_status->status =  *(uint16*)(data+122);
    flying_status->gps_status =  *(uint8*)(data+124);
    flying_status->imu_status =  data[125];
    flying_status->cpu_s =  data[126];
    flying_status->voltage =  data[127]*3.3*11/256;
    flying_status->engine =  data[128];
    flying_status->cpu_temp =  *(int8*)(data+129);
    flying_status->rsv =  data[130];
    if(mode==1){
       flying_status->reserv[0] =  *(uint16*)(data+131);
       flying_status->reserv[1] =  *(uint16*)(data+133);
       flying_status->reserv[2] =  *(uint16*)(data+135);
       flying_status->reserv[3] =  *(uint16*)(data+137);
       flying_status->reserv[4] =  *(uint16*)(data+139);
       flying_status->reserv[5] =  *(uint16*)(data+141);
       flying_status->reserv[6] =  *(uint16*)(data+143);
       flying_status->reserv[7] =  *(uint16*)(data+145);
       flying_status->reserv[8] =  *(uint16*)(data+147);
       flying_status->reserv[9] =  *(uint16*)(data+149);
    }else if(mode == 0){
       flying_status->reserv[0] = 0;
       flying_status->reserv[1] = 0;
       flying_status->reserv[2] =  0;
       flying_status->reserv[3] = 0;
       flying_status->reserv[4] = 0;
       flying_status->reserv[5] = 0;
       flying_status->reserv[6] =  0;
       flying_status->reserv[7] =0;
       flying_status->reserv[8] =0;
       flying_status->reserv[9] = 0;
    }
}

void save_flying_status(flying_status_s *flying_status)
{
	frame_count++;
	fprintf(fp_fly_status,"%d,%d,",frame_count,flying_status->g_time);
	fprintf(fp_fly_status,"%f,%f,%f,",flying_status->roll,flying_status->pitch,flying_status->yaw);
	fprintf(fp_fly_status,"%f,%f,%f,",flying_status->gx,flying_status->gy,flying_status->gz);
	fprintf(fp_fly_status,"%f,%f,%f,",flying_status->ax,flying_status->ay,flying_status->az);
	fprintf(fp_fly_status,"%d,%d,%d,",flying_status->vn,flying_status->ve,flying_status->vd);
	fprintf(fp_fly_status,"%d,%d,",flying_status->heading,flying_status->b_h);
	fprintf(fp_fly_status,"%f,%f,%f,",flying_status->lat,flying_status->Long,flying_status->g_h);
	fprintf(fp_fly_status,"%f,%f,%f,",flying_status->vx,flying_status->vy,flying_status->vz);
	fprintf(fp_fly_status,"%d,%d,",flying_status->sonar_h,flying_status->waypoint_dest);
	fprintf(fp_fly_status,"%d,%d,%d,%d,%d,",flying_status->pwm[0],flying_status->pwm[1],flying_status->pwm[2],flying_status->pwm[3],flying_status->pwm[4]);
	fprintf(fp_fly_status,"%d,%d,%d,%d,%d,",flying_status->pwm[5],flying_status->pwm[6],flying_status->pwm[7],flying_status->pwm[8],flying_status->pwm[9]);
	fprintf(fp_fly_status,"%x,%x,%x,%x,",flying_status->status,flying_status->gps_status,flying_status->imu_status,flying_status->cpu_s);
	fprintf(fp_fly_status,"%f,%x,%d,%x,%d,",flying_status->voltage,flying_status->engine,flying_status->cpu_temp,flying_status->rsv,sonar_data_filter(flying_status));
        fprintf(fp_fly_status,"%d,%d,%d,%d,%d,",flying_status->reserv[0],flying_status->reserv[1],flying_status->reserv[2],flying_status->reserv[3],flying_status->reserv[4]);
		fprintf(fp_fly_status, "%d,%d,%d,%d,%d,", flying_status->reserv[5], flying_status->reserv[6], flying_status->reserv[7], flying_status->reserv[8], flying_status->reserv[9]);
		fprintf(fp_fly_status, "%d,%d,%d,%d,%d\n", flying_status->reserv[10], flying_status->reserv[11], flying_status->reserv[12], flying_status->reserv[13], flying_status->reserv[14]);

}
int control_data_parse(unsigned char *buf, frame_info *frame_info)
{


	unsigned char frame_type = buf[10];
	flying_status_s flying_status;
	int ret=-1;
	int i;
	unsigned char data_buf[4096];


	    if(frame_type==CTRL_FRAME_TYPE_FLY_STATUS) {
	        //copy data field to data_buf to avoid alignment trap
	    	memcpy(data_buf,buf+CTRL_FRAME_MASK_DATA,frame_info->frame_size);
	    	// if the frame is flying status,extract the flying status

            flying_status_parse(data_buf,&flying_status);
			//generate binary file for servo test
			if (generate_servo_test)
				fwrite(data_buf + 131, 20, 1, fp_servo_test);

	    	// save flying status as CSV format,so that we can use EXCEL to open it
	    	save_flying_status(&flying_status);


		}else {
                   printf("frame type error \n");
                }


	return 0;
}
unsigned int serial_data_recv_ctrl(frame_info *frame_info ,unsigned char *buf)
{
	  unsigned int nread=0;
	  unsigned int i=0;
	  unsigned int frame_head_found=0;
	  unsigned int frame_crc;
        



 	  frame_info->bytes_received ++;

 	while(frame_info->bytes_received > 0){
 		//print_debug("ctrl bytes received %d\n",frame_info->bytes_received);
 	   // start searching frame head if at least 2 bytes has been received
 	   if(frame_info->bytes_received >= 2){
 	        for(i=0;i<frame_info->bytes_received-1;i++){
 	             if((buf[i]==CTRL_FRAME_START1)&&(buf[i+1]==CTRL_FRAME_START2)){
 	                //found the frame head ,remove useless received bytes before the beginning of the frame
 	                  if(i>0){
 	                       memmove(buf,buf+i,frame_info->bytes_received-i);
 	                       frame_info->bytes_received=frame_info->bytes_received-i;
 	                  }
 	                       frame_head_found = 1;
 	                       //print_debug("ctrl head found\n");
 	                       break;
 	              }
 	         }
 	    }else{
 	    	// not enough data
 	    	return 0;
 	    }

 	  if(frame_head_found){
            // frame head has been found ,start extract frame if we have received minimum bytes of a frame
 	        if(frame_info->bytes_received < CTRL_FRAME_MINIMUM_LEN){
 	        	//do not have enough data for a valid frame
 	        	//print_debug("ctrl :do not have enough data for a valid frame\n");
 	        	return 0;

 	        }
            // we have found the frame head ,extract the machine ID
            //frame_info->machine_id = (buf[3]<<8) |  buf[2];
             //extract the frame size
            frame_info->frame_size = (buf[CTRL_FRAME_MASK_FRAME_SIZE+1]<<8) | buf[CTRL_FRAME_MASK_FRAME_SIZE];

            //check if frame size is valid
            if((frame_info->frame_size >= CTRL_FRAME_MINIMUM_LEN)&&(frame_info->frame_size <= CTRL_FRAME_MAX_LEN)){
                 if(frame_info->frame_size > frame_info->bytes_received){
            	    // do not have received whole frame
                	 //print_debug("ctrl :do not have received whole frame\n");
            	     return 0;
                 }

                 // we have received the whole frame ,so check the frame tail
                 if(buf[frame_info->frame_size-1]==CTRL_FRAME_END){

            	     frame_crc = (buf[frame_info->frame_size-2]<<8) | buf[frame_info->frame_size-3];
            	     if(frame_crc==crc_checksum16(buf, frame_info->frame_size-3)){
            	     //if(1){
            		    // we have a valid CRC
                      count++;
                     if(frame_time_last==0)
                          gap =0;
                     else  
                          gap = *(uint32*)(buf+47)-frame_time_last;
					 if (imu_type == 1)// frame time unit of sbg ellipse is us
						 gap = (gap / 1000);
                     if(gap != data_refresh_rate){
                         print_debug("----------------lost---------------%d ,frame %8d,frame time :%d\n",gap,count,*(uint32*)(buf+47));
                         lost_count += ((gap/data_refresh_rate)-1);
                      }
                     
                        // print_debug("%d ,frame %8d,frame time :%d\n",gap,count++,*(uint32*)(buf+47));
                     
                     frame_time_last=*(uint32*)(buf+47); 
            		     return frame_info->frame_size;
            	     }else{
                        // invalid CRC ,remove the whole frame from the buffer
                        print_debug("invalid crc frame,frame time :%d\n",*(uint32*)(buf+47));
                           getchar();
            		    memmove(buf,buf+frame_info->frame_size,frame_info->bytes_received-frame_info->frame_size);
            	    	frame_info->bytes_received=frame_info->bytes_received-frame_info->frame_size;
            	    	frame_info->frame_size = 0;
            		    frame_head_found = 0;


            	     }

                 }else{
            	     //frame tail not found ,so the frame is invalid,
            	     //we should have incorrectly detected a start of frame
                     //remove the 2 frame head bytes and start searching frame head again
                	 print_debug("ctrl :frame tail not found ,so the frame is invalid\n");
            	     memmove(buf,buf+2,frame_info->bytes_received-2);
            	     frame_info->bytes_received=frame_info->bytes_received-2;
            	     frame_info->frame_size = 0;
            	     frame_head_found = 0;
            	     continue;
                 }
            }else{
                // invalid frame_size ,which means wrong frame head is detected
                 // we need to remove the 2 wrong frame head bytes
            	 print_debug("ctrl :invalid frame_size \n");
                 memmove(buf,buf+2,frame_info->bytes_received-2);
                 frame_info->bytes_received=frame_info->bytes_received-2;
                 frame_info->frame_size = 0;
                 frame_head_found = 0;
            }
       }else{
            //unable to find a valid start of frame
            //so check the last byte is FRAME_START1 in order to keep it for next time
    	    print_debug("ctrl :invalid start of frame\n");
            if(buf[frame_info->bytes_received-1]==CTRL_FRAME_START1){
                buf[0] = CTRL_FRAME_START1;
                frame_info->bytes_received = 1;
            }else{
                //discard the whole buffer
                frame_info->bytes_received = 0;
            }
            return 0;
        }


 	}
 	return 0;
}
void file_fly_status_init()
{
   fprintf(fp_fly_status,"frame,time(ms),");
   fprintf(fp_fly_status,"roll,pitch,yaw,");
   fprintf(fp_fly_status,"gx,gy,gz,");
   fprintf(fp_fly_status,"ax,ay,az,");
   fprintf(fp_fly_status,"vn,ve,vd,");
   fprintf(fp_fly_status,"heading,b_high,");
   fprintf(fp_fly_status,"lat,long,g_h,");
   fprintf(fp_fly_status,"vx,vy,vz,");
   fprintf(fp_fly_status,"sonar,wp_id,");
   fprintf(fp_fly_status,"CH1/pwm0,CH2/pwm1,CH3/pwm2,rudder/pwm3,throttle/pwm4,pwm5,period/pwm6,pwm7,pwm8,pwm9,");
   fprintf(fp_fly_status,"fly_status,gps_s,imu_s,cpu_s,voltage,engine,cpu_t,rsv,sonar_process,");
   fprintf(fp_fly_status,"rsv1,rsv2,rsv3,rsv4,rsv5, rsv6,rsv7(int)  ,rsv8 ,rsv9 ,rsv10,");
   fprintf(fp_fly_status, "rsv11,rsv12,rsv13,rsv14,rsv15\n");

}

int read_file_list(char *p)
{  
	char current_dir[100];
	char cmd[200];
	FILE *fp = NULL;
	char file_list[200];
	int file_number = 0;
	getcwd(current_dir, 100);
	char file_name[100];
	int file_name_length=0;

	sprintf(cmd, "dir %s /b > %s\\filelist.txt",current_dir,current_dir);
	
	system(cmd);
	strcpy(file_list, current_dir);
	strcat(file_list, "\\filelist.txt");
	fp = fopen(file_list,"r");
	if (fp == NULL){
		printf("can not open %s\n",file_list);
		printf("press any key to close\n");
		getchar();
		exit(0);
	}

	while (!feof(fp)){

		fgets(file_name, 50, fp);
		file_name[strlen(file_name) - 1] = '\0';
		file_name_length = strlen(file_name);
		//printf("%s,%d \n", file_name + file_name_length - 4, file_name_length);
		if (file_name_length > 6){
			
			if ((strcmp(file_name + file_name_length - 4, ".raw") == 0)){
				strcpy(p + file_number * FILE_NAME_LENGTH, file_name);
				file_number++;
			}
		}
		
	}
	return file_number;
	
	 }
void file_merge(int file_number, char file[400][FILE_NAME_LENGTH]){
	int i;
	FILE *dest_fp,*fp;
	char buf=0;
	dest_fp = fopen("file_merge.raw", "wb");
	if (dest_fp == NULL){
		printf("can not open file_merge.raw\n");
		printf("press any key to close\n");
		getchar();
		return;
	}

	for (i = 0; i < file_number; i++){
		printf("###process file %s\n",file[i]);
		fp = fopen(file[i], "rb");

		if (fp == NULL){
			printf("can not open %s\n", file[i]);
			printf("press any key to close\n");
			getchar();
			return;
		}
		
		while (fread(&buf, 1, 1, fp) != 0){
			fwrite(&buf,1,1,dest_fp);

		}
		printf("###file %s merge over\n",file[i]);
		fclose(fp);

	}
	fclose(dest_fp);
}
void main(){
       FILE *fp ;
       unsigned char buf[4096];
	   int data_len =0;
	   unsigned char console_buf[128];

	   frame_info frame_info_ctrl={0,0};
           char source_dir[128];
           char dest_format[5]=".csv";
           char dest_dir[50];
		   char file_name[400][FILE_NAME_LENGTH];
		   int  file_number;
		   char current_dir[128];
		   int i;
		   
		  
		   printf("###created by Jin 2016-01-13\n");
     printf("###this program is used to parse UAV data saved on the ground or in the air\n");
	 printf("###please enter mode : 0 for  145 bytes frame size,which is saved on the ground\n");
	 printf("###                    1 for  175 bytes frame size,which is saved in the air\n");
	 fgets(console_buf, 128, stdin); 
	 console_buf[strlen(console_buf) - 1] = '\0';
       mode = atol(console_buf);

	   file_number = read_file_list(file_name);
       printf("###please choose data refresh rate: 0 for 5ms\n");
	   printf("###                                 1 for 10ms\n");
	   printf("###                                 2 for 20ms\n");
	   fgets(console_buf, 128, stdin); 
	   console_buf[strlen(console_buf) - 1] = '\0';
       switch( atol(console_buf)){
		   case 0:
		      data_refresh_rate = 5;
			  break;
		   case 1:
			  data_refresh_rate = 10;
			  break;
		   case 2:
			  data_refresh_rate = 20;
			  break;
		   default:
		      data_refresh_rate = 20;
			  break;
	   }   

	   printf("###please choose IMU type : 0 for SBG IG500\n");
	   printf("###                         1 for SBG ELLIPSE\n");

	   fgets(console_buf, 128, stdin);
	   console_buf[strlen(console_buf) - 1] = '\0';
	   imu_type = atol(console_buf);

	   
   process:
	   count = 0;
	   frame_count = 0;
	   lost_count = 0;
	    m = 0;
	    frame_time_last = 0;
	   memset(source_dir, 128, 0);
	   getcwd(source_dir, 100);//get current dir 
	   printf("###identify %d raw files :\n", file_number);
	   for (i = 0; i < file_number; i++){
		   printf("###%3d %s\n", i+1, file_name[i]);
	   }
	   
	  // printf("###please enter source file dir and name:\n");
	   printf("###please enter file ID to choose which file to parse,enter m to merge all raw files,enter q to exit\n");
	   fgets(console_buf, 128, stdin);
	   console_buf[strlen(console_buf) - 1] = '\0';
	   if (strcmp(console_buf, "m") == 0){
		   file_merge(file_number, file_name);
		   goto process;

	   }  else if (strcmp(console_buf, "q") == 0){
		   exit(0);
	   }
	   else{
		   strcat(source_dir, "\\");
		   strcat(source_dir, file_name[atol(console_buf) - 1]);
		   //fgets(source_dir, 50, stdin);
		   // source_dir[strlen(source_dir) - 1] = '\0';
		   if (mode == 1){
			   printf("do you want to generate servo test file? y or n?");
			   fgets(console_buf, 128, stdin);
			   console_buf[strlen(console_buf) - 1] = '\0';
			   if (strcmp(console_buf, "y") == 0)
				   generate_servo_test = 1;

		   }
		   if (generate_servo_test){
			   fp_servo_test = fopen("servo_test.raw", "wb");
			   if (fp_servo_test == NULL){
				   printf("can not open servo_test.raw\n");
				   printf("press any key to close\n");
				   getchar();
				   return;
			   }
			   printf("processing file %s in mode %d and genrating servo test file\n", source_dir, mode);
		   }
		   else
			   printf("processing file %s in mode %d \n", source_dir, mode);
		   //source_dir  = argv[2]; 
		   strcpy(dest_dir, source_dir);
		   strcat(dest_dir, ".csv");

		   fp = fopen(source_dir, "rb");
		   if (fp == NULL){
			   printf("can not open raw data file:%s\n", source_dir);
			   printf("press any key to close\n");
			   getchar();
			   return;
		   }
		   fp_fly_status = fopen(dest_dir, "w");
		   if (fp_fly_status == NULL){
			   printf("can not open dest file:%s\n", dest_dir);
			   printf("press any key to close\n");
			   getchar();
			   exit(0);
		   }
		   file_fly_status_init();
		   while (1){
			   if (fread(buf + frame_info_ctrl.bytes_received, 1, 1, fp) == 0){
				   printf("total %d frames ;lost %d frames \n", count, lost_count);
				   printf("process over,press q to close,press r to continue\n");
				   fgets(console_buf, 128, stdin);
				   console_buf[strlen(console_buf) - 1] = '\0';
				   if (strcmp(console_buf, "q") == 0){
					   fclose(fp);
					   fclose(fp_fly_status);
					   if (generate_servo_test)
						   fclose(fp_servo_test);
					   exit(0);
				   }
				   else if (strcmp(console_buf, "r") == 0)
					   goto process;


			   }


			   m++;

			   data_len = serial_data_recv_ctrl(&frame_info_ctrl, buf);
			   if (data_len > 0){
				   control_data_parse(buf, &frame_info_ctrl);
				   if (frame_info_ctrl.bytes_received > frame_info_ctrl.frame_size){
					   memmove(buf, buf + frame_info_ctrl.frame_size, frame_info_ctrl.bytes_received - frame_info_ctrl.frame_size);
				   }
				   frame_info_ctrl.frame_size = 0;
				   frame_info_ctrl.bytes_received -= data_len;
			   }
		   }

	   }

	

}
