/*
 * main.cpp
 *
 *  Created on: 2015. 01. 29.
 *      Author: Tackeuch Ryo and sukoshi D.O.
 *      Initial: Robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include <cstring>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"
#include "VisionMode.h"

#include "motionController.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

void walkStop(Point2D point){
	// the X,Y coordinates will stop
	
}

int main(void)
{
	MotionController *controller = new MotionController();
	bool initialized = false;
	initialized = controller->initMotionManager();
	usleep(500000); // 1 second = 1,000,000 us

	if (initialized)
	{
		printf("initialized\n");
	}
	else printf("\n Initialization failed\n");

	minIni* ini = new minIni(INI_FILE_PATH);
	Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
	
	LinuxCamera::GetInstance()->Initialize(0);
	LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
	LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini

	mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

	ColorFinder* ball_finder = new ColorFinder();
	ball_finder->LoadINISettings(ini);
	httpd::ball_finder = ball_finder;

	BallTracker tracker = BallTracker();
	BallFollower follower = BallFollower();

	/*ColorFinder* red_finder = new ColorFinder(0, 15, 45, 0, 0.3, 50.0);
	red_finder->LoadINISettings(ini, "RED");
	httpd::red_finder = red_finder;

	ColorFinder* yellow_finder = new ColorFinder(80, 20, 45, 0, 0.3, 50.0);
	yellow_finder->LoadINISettings(ini, "YELLOW");
	httpd::yellow_finder = yellow_finder;

	ColorFinder* blue_finder = new ColorFinder(225, 15, 45, 0, 0.3, 50.0);
	blue_finder->LoadINISettings(ini, "BLUE");
	httpd::blue_finder = blue_finder;*/

	ColorFinder* green_finder = new ColorFinder(100,  27,  26, 100,  40, 100, 0.1, 60.0);

	ColorFinder* white_finder = new ColorFinder(180, 180,   0,  26, 56, 100, 0.3, 50.0);

	//green_finder->LoadINISettings(ini, "YELLOW"); //wakaran
	//httpd::yellow_finder = green_finder;
	//httpd ha nai

	httpd::ini = ini;

	//////////////////// Framework Initialize ////////////////////////////
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		linux_cm730.SetPortName(U2D_DEV_NAME1);
		if(MotionManager::GetInstance()->Initialize(&cm730) == false)
		{
			printf("Fail to initialize Motion Manager!\n");
			return 0;
		}
	}

	Walking::GetInstance()->LoadINISettings(ini);

	MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	MotionManager::GetInstance()->LoadINISettings(ini);

	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	MotionManager::GetInstance()->SetEnable(true);

	//sitdown
	Action::GetInstance()->Start(15);
	while(Action::GetInstance()->IsRunning()) usleep(8*1000);
	usleep(500000);

	//standup
	/*Action::GetInstance()->Start(16);
	while (Action::GetInstance()->IsRunning()) usleep(8*1000);*/
	
	LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");

	printf("enter to main loop\n");

	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	Head::GetInstance()->MoveByAngle(0, 40);
	
	usleep(500000);

	bool stop = false;
	bool AvoidObstaclesRight = false;
	bool AvoidObstaclesLeft = false;

	int PhaseAvoid = 0;

	double direction;
	double stepSize;
	
	int phase = 0;
	int pre_phase = 0;
	int phase_count = 0;
	bool Next_movement = false;

	while(1)
	{
		//update dymanixel
		StatusCheck::Check(cm730);// Check the robot position, felldown or not

		stop = false;
		AvoidObstaclesRight = false;
		AvoidObstaclesLeft = false;

		//color, objects position
		Point2D ball_pos, red_pos, yellow_pos, blue_pos, green_pos, white_pos;

		//camera capture
		LinuxCamera::GetInstance()->CaptureFrame();
		//copy to opencv mat
		memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

		//update color finder
		//red_pos = red_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		green_pos = green_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		white_finder->GetPositionObstacles(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame, 200, 80);
		
		//Print the Color Masks, which we are found
		unsigned char r, g, b;
		int ar = 0;
		for (int i = 0; i < rgb_output->m_NumberOfPixels; i++)
		{
			r = 0; g = 0; b = 0;

			if (green_finder->m_result->m_ImageData[i] == 1){
				r = 0;
				g = 255;
				b = 0;
			}

			if (r > 0 || g > 0 || b > 0)
			{
				rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 0] = r;
				rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 1] = g;
				rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 2] = b;
			}

			r = 0; g = 0; b = 0;

			if (white_finder->m_result->m_ImageData[i] == 1){
				r = 255;
				g = 255;
				b = 255;
				ar++;
			}

			if (r > 0 || g > 0 || b > 0)
			{
				rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 0] = r;
				rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 1] = g;
				rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 2] = b;
			}
		}
		
		//send image to server		
		streamer->send_image(rgb_output);

		//printf("X = %f , Y = %f\n", green_pos.X, green_pos.Y);
		/*
		for (int i = 0; i < 10; i++){
			printf("%d\t", white_finder->obstacles[i]);
			if (white_finder->obstacles[i] > 2200){	
				if (i > 1 && i <= 4) AvoidObstaclesLeft = true;
				if (i < 9 && i >= 5) AvoidObstaclesRight = true;
			}
		}*/
		//printf("\n");

		for (int i = 0; i < 10; i++){
			//printf("%d\t", white_finder->nearobstacles[i]);
			if (i > 0 && i < 9){
				if (white_finder->nearobstacles[i] > 400){
					stop = true;// if we have white obstacles near to robot stop walking, to not damage the robot
					if (i > 1 && i <= 4) AvoidObstaclesLeft = true;
					if (i < 9 && i >= 5) AvoidObstaclesRight = true;
				}
			}				
		}
				
		//if (Action::GetInstance()->IsRunning() == 0){
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			if (green_pos.X < 0 && green_pos.Y < 0) {
				Head::GetInstance()->MoveByAngle(0, 40); 
				usleep(100000);
			}
			else tracker.Process(green_pos);

			//	MotionManager::GetInstance()->SetEnable(true)
		//}
		
		if (AvoidObstaclesLeft){
			printf(" Obstacles on left\t");//turn to right
			direction = -5.0;
			if (Next_movement) PhaseAvoid = 1;
			else PhaseAvoid = 0;
		}
		else{
			if (AvoidObstaclesRight){
				printf(" Obstacles on right\t");//turn to left
				direction = 5.0;	
				if (Next_movement) PhaseAvoid = 1;
				else PhaseAvoid = 0;
			}
			else{
				if (!stop){
					printf(" Walking forward\t");
					PhaseAvoid = 119;
					direction = 0.0;
				}
				else{					
					PhaseAvoid = 404;
					direction = 0.0;
				}
			}
		}

		if (stop){ 
			printf("obstacles to close\n");
			
			if (AvoidObstaclesLeft){
				direction = -20.0;//turn to right
				//printf("Left\n");
			}
			else{
				direction = -20.0;//turn to left
				//printf("Right\n");
			}

			if (Next_movement) PhaseAvoid = 1;
			else PhaseAvoid = 0;
		}
		
		printf("PhaseAvoid %d direction %f\n", PhaseAvoid, direction);
		PhaseAvoid = 404;
		switch (PhaseAvoid){
		case 0: //Go to Right, to avoid obstacles
			//printf("Turn to left or right\n");
			if (Action::GetInstance()->IsRunning() == 0)
			{
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
				MotionManager::GetInstance()->SetEnable(true);

				Walking::GetInstance()->A_MOVE_AMPLITUDE = direction;
				Walking::GetInstance()->Start();
			}

			phase = Walking::GetInstance()->GetCurrentPhase();

			//if (phase == 0 ) 
				phase_count++;
			//pre_phase = phase;

				printf("phase_count\t%d\n", phase_count);
			
			if (phase_count > 10) {
				PhaseAvoid = 1;
				phase = 0;
				phase_count = 0; 
				Walking::GetInstance()->Stop();
				usleep(500000);
				Next_movement = true;
			}
			break;

		case 1:
			printf("A little bit forward    ---------->\n");
			//LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
			if (Action::GetInstance()->IsRunning() == 0)
			{
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
				MotionManager::GetInstance()->SetEnable(true);

				Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 5.0;
				Walking::GetInstance()->Start();
			}

			phase = Walking::GetInstance()->GetCurrentPhase();
			printf("phase_count\t%d\n", phase_count);
			//if (phase == 0) 
			phase_count++;
			//pre_phase = phase;
			Next_movement = true;

			if (phase_count > 40) {
				PhaseAvoid = 119;					
				//pre_phase = 0;
				phase_count = 0;
				Walking::GetInstance()->Stop();
				Next_movement = false;
				usleep(500000);
			}
			
			break;

		case 404:
			if (Action::GetInstance()->IsRunning() == 0)
			{
				Walking::GetInstance()->Stop();
			}
			PhaseAvoid = 0;
			break;

		case 119:
			if (Action::GetInstance()->IsRunning() == 0)
			{
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
				MotionManager::GetInstance()->SetEnable(true);
				follower.Process(tracker.Goal_position); //change to green_pos
			}
			PhaseAvoid = 0;
			phase_count = 0;
			break;

		default:
			printf("Not correct Phase %d", PhaseAvoid);
		}	


		/*
		if (green_pos.X < 0.0 || green_pos.Y < 0.0){

			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);

			Head::GetInstance()->MoveByAngle(0, Angle_i);

			usleep(8000);
			Angle_i = Angle_i + 0.5;
			if (Angle_i > 40.0){ Angle_i = 40.0; }
			printf("Mark was not found angle = %f\n", Angle_i);

		}
		else*/
	}

	return 0;
}
