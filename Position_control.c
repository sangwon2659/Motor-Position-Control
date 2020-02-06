#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
 
// Define experiment Time
#define EXP 16000
 
// Define time duration
#define LOOPTIME 10
#define SAMPLING 5
#define DEMOTIME 2000
 
#define ENCODERA 6
#define ENCODERB 13
#define ENC2REDGEAR 216
#define MOTOR1 19
#define MOTOR2 26
 
#define TOLERANCE 0.05
#define MAX 100000

//PID GAIN
#define PGAIN 700
#define IGAIN 400
#define DGAIN 6.3187
#define _CRT_SECURE_NO_WARNINGS
 
char buf1[10];
char buf2[10];
float position_record[MAX] = {0};
int iter = 0;
int encA;
int encB;
int encoderPosition = 0;
float redGearPosition = 0;
float referencePosition = 0;
float errorPosition = 0 ;
float errorrecord[MAX] = { 0 };
int motorinput = 0;
 
unsigned int startTime;
unsigned int checkTime;
unsigned int checkTimeBefore;
 
void funcEncoderA(){

	encA = digitalRead(ENCODERA);
	encB = digitalRead(ENCODERB);
	if (encA == HIGH){
		if (encB == LOW)
			encoderPosition--;
		else
			encoderPosition++;
	}
	else{
		if (encB == LOW)
			encoderPosition++;
		else
			encoderPosition--;
	}
	redGearPosition = (float)encoderPosition / ENC2REDGEAR;
	errorPosition = referencePosition - redGearPosition;
	//printf("Encoder: %d\tRedGear: %f\n",encoderPosition,redGearPosition);
}
 
void funcEncoderB(){

	encA = digitalRead(ENCODERA);
	encB = digitalRead(ENCODERB);
	if (encB == HIGH){
		if (encA == LOW)
			encoderPosition++;
		else
			encoderPosition--;
	}
	else{
		if (encA == LOW)
			encoderPosition--;
		else
			encoderPosition++;
	}
	redGearPosition = (float)encoderPosition / ENC2REDGEAR;
	errorPosition = referencePosition - redGearPosition;
	//printf("Encoder: %d\tRedGear: %f\n",encoderPosition,redGearPosition);
}
 
void pid_control(int iter){

	float g1 = (PGAIN + IGAIN * (float)LOOPTIME/1000.0 + DGAIN / (float)LOOPTIME*1000);
	float g2 = (-PGAIN - 2 * DGAIN / (float)LOOPTIME*1000);
	float g3 = (DGAIN / (float)LOOPTIME*1000);
	motorinput = (int)fabs(motorinput + g1 * errorrecord[iter] + g2 * errorrecord[iter - 1] + g3 * errorrecord[iter - 2]);
  if (motorinput > 100){
		motorinput = 100;
	}
	if (errorPosition > 0){
		softPwmWrite(MOTOR1, motorinput);
		softPwmWrite(MOTOR2, 0);
	}
	else{
		softPwmWrite(MOTOR1, 0);
		softPwmWrite(MOTOR2, motorinput);
	}
}

int main(void){	

	FILE *fp_rec =fopen("data_rec.txt","w");
	FILE *fp_plbk =fopen("data_plbk.txt","w");
  
	wiringPiSetupGpio();
	pinMode(ENCODERA, INPUT);
	pinMode(ENCODERB, INPUT);
  
	wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderA);
	wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderB);
  softPwmCreate(MOTOR1, 0, 100);
	softPwmCreate(MOTOR2, 0, 100);
  
	srand((unsigned int)time(0));
	
	char order;
	int cnt = 0;
  
	while(1){
		softPwmWrite(MOTOR1, 0);
		softPwmWrite(MOTOR2, 0);
		printf("Command(R or T): ");
		scanf("%c",&order);
    
		if(order=='R'){
			printf("Reading data\n");
			startTime = millis();
			checkTimeBefore = millis();
			checkTime = millis();
			unsigned int DemoTime = millis();
      
			while(checkTime-startTime < EXP){
				checkTime = millis();
				// Sampling the position from demo
        
				if(checkTime - checkTimeBefore >= SAMPLING){
					position_record[cnt] = redGearPosition;
					printf("%d, %0.4f\n",cnt,position_record[cnt]);
					sprintf(buf1,"%0.5f\n",position_record[cnt]);
					fwrite(buf1,strlen(buf1),1,fp_rec);
					cnt++;
					checkTimeBefore = checkTime;
				}
			}
		}
    
		else if(order=='T'){
			printf("Traking path\n");
			encoderPosition = 0;
			redGearPosition = 0;
			errorPosition = 0;
			startTime = millis();
			checkTimeBefore = millis();
			checkTime = millis();
			int sample_cnt = 0;
			unsigned int sampleTime = millis();
      
			while(checkTime-startTime < EXP){
				//printf("reference: %0.3f error: %0.3f\n",redGearPosition,errorPosition);
				checkTime = millis();
				
				//reference position reconstruction
        
				if(checkTime - sampleTime >= SAMPLING && sample_cnt < cnt){
					referencePosition = position_record[sample_cnt];
          printf("%d, %0.4f\n",sample_cnt,referencePosition);
					sprintf(buf2,"%0.5f\n",redGearPosition);
					fwrite(buf2,strlen(buf2),1,fp_plbk);
          sample_cnt++;
					sampleTime = checkTime;
				}
        
				errorPosition = referencePosition - redGearPosition;
				// control every LOOPTIME
        
				if(checkTime - checkTimeBefore >= LOOPTIME){
					errorrecord[iter] = fabs(errorPosition);
					if (iter > 1){
						pid_control(iter);
					}
					else{
						if(referencePosition>0){
							softPwmWrite(MOTOR1, 0);
							softPwmWrite(MOTOR2, 0);
						}
						else{
							softPwmWrite(MOTOR1, 0);
							softPwmWrite(MOTOR2, 0);
						}
					}
					iter++;
					checkTimeBefore = checkTime;
				}				
			}
		}
		else
			continue;
	}
	fclose(fp_plbk);
	fclose(fp_rec);
	return 0;
}
