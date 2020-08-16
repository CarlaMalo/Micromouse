/*
 Name:		Micromouse_final_2020.ino
 Created:	18/06/2020 19:43:03
 Author:	Carla Malo
*/

//PERSONAL LIBRARIES
#include <Floodfill.h>
#include <Controller_M.h>
#include <Profile_Velocity.h>
#include <Motor_ESP32.h>
#include <Encoder.h>

//EXTERNAL LIBRARIES
#include <BluetoothSerial.h>
#include <Useful_Methods.h>

//MOTOR DRIVER PINS, to control the direction of spin of the wheels (OUTPUT)
#define LEFT_PIN_A1 21 
#define LEFT_PIN_A2 22
#define RIGHT_PIN_A1 19
#define RIGHT_PIN_A2 18

//INCREMENTAL ENCODER PINS (SIGNAL A AND B) (INPUT)
#define LEFT_PIN_A 15
#define LEFT_PIN_B 2
#define RIGHT_PIN_A 4
#define RIGHT_PIN_B 16

//EMITTERS PINS (OUTPUT)
#define EMITTER_PIN_LF 13
#define EMITTER_PIN_L 12
#define EMITTER_PIN_R 14
#define EMITTER_PIN_RF 27

//RECEIVER PINS (INPUT)
#define RECEIVER_PIN_LF 35
#define RECEIVER_PIN_L 34
#define RECEIVER_PIN_R 33
#define RECEIVER_PIN_RF 32

hw_timer_t * timer1 = NULL;
volatile uint8_t flag_sensor = 0;
uint8_t emitter_pin[] = { EMITTER_PIN_L, EMITTER_PIN_LF,EMITTER_PIN_RF,EMITTER_PIN_R };
uint8_t receiver_pin[] = { RECEIVER_PIN_L, RECEIVER_PIN_LF,RECEIVER_PIN_RF,RECEIVER_PIN_R };
volatile uint16_t  receiver_reading[] = { 0,0,0,0 };
volatile uint8_t cycle_sensor = 0;
volatile uint16_t measure_last = 0;
volatile uint16_t measure_next = 0;



//BRIDGE: GENERATES THE PWM SIGNAL TO MODULATE THE SPEED OF THE WHEELS
HBridgeMosfet motorLeft(0, 1, LEFT_PIN_A1, LEFT_PIN_A2);
HBridgeMosfet motorRight(2, 3, RIGHT_PIN_A1, RIGHT_PIN_A2);

//ENCODER READINGS
Encoder encoderLeft(LEFT_PIN_A, LEFT_PIN_B, 0.2);
Encoder encoderRight(RIGHT_PIN_A, RIGHT_PIN_B, 0.2);
volatile float encoderLeft_Speed;
volatile float encoderRight_Speed;
float average_Speed;

//TRIANGULAR/TRAPEZOIDAL PROFILE
//by controlling the speed you can estime the displacement (area of the profile)
Profile profile(0.01, 3);
Profile profile_right_turn(0.04, 20);
Profile profile_left_turn(0.04, 20);
Profile profile_full_turn(0.04, 20);

//BLUETOOTH AND COMMUNICATION
BluetoothSerial SerialBT;

//CONTROLLERS
uint8_t dt = 5;
Controlador control_translational(dt);
Controlador control_rotational(dt);
Controlador control_sensor(dt);
unsigned long elapsed_time;
unsigned long start_time;
uint8_t duty_cycle;
vector<float> v_output;
vector<float> v_dif;
vector<float> control_output1_array;
vector<float> control_output2_array;
uint8_t flag_sequence;
uint16_t i;
uint16_t profile_length;
float control_output1;
float control_output2;
float speed_difference;
float y_dif;
uint8_t f;
float y_left;
float y_right;
char movement;
uint16_t next_profile_distance;

//Algorithm
coordinate startPosition(3, 3);
coordinate targetPosition(0, 0);
coordinate currentPosition = targetPosition;
char orientation = 'N';
int wall_indx_order[4] = { 0,0,0,0 };
int indx_N[4] = { 0,1,2,3 }; //F,0,L,R
int indx_S[4] = { 1,0,3,2 }; //0,F,R,L
int indx_W[4] = { 3,2,0,1 }; //R,L,F,0
int indx_E[4] = { 2,3,1,0 };//L,R,0,F
int* p_indx = indx_N;
uint8_t wall_reading[4] = { 0,0,0,0 };
char two_movements;

// IR Sensors interruption
void IRAM_ATTR onTimer1() {
	if (cycle_sensor < 4) {
		if (flag_sensor == 1) {
			measure_last = analogRead(receiver_pin[cycle_sensor]); //take away ambient light before turning emitter ON
			digitalWrite(emitter_pin[cycle_sensor], 1); //turns ON
			flag_sensor = 2;

		}
		else if (flag_sensor == 2) {
			measure_next = analogRead(receiver_pin[cycle_sensor]); //reads when the receiver is fully charged
			receiver_reading[cycle_sensor] = measure_next - measure_last;
			digitalWrite(emitter_pin[cycle_sensor], 0); //turns OFF
			flag_sensor = 1;
			cycle_sensor = cycle_sensor + 1;
		}
	}
}

//Detecting the precense of walls (binary)
void wallReading() {
	int indx1[4] = {1,2,0,3};
	//
	//L Left,LF Left Front,RF Right Front,R Right , passed to order --F,0,L,R
	for (unsigned int i = 0; i < 4; i++) {
		if (receiver_reading[indx1[i]] > 1300) //1100
			wall_reading[i] = 1; //there's a wall
		else
			wall_reading[i] = 0; //No wall
	}
	wall_reading[1] = 0; //RF is not used

}

//Sensor Calibration (converting the ADC reading to the equivalent in counts of the encoder)
void sidesReading(float &y_dif) {


	uint16_t d = 600; //400 //600 //800
	uint16_t  x_left = receiver_reading[0];
	uint16_t  x_right = receiver_reading[3];

	//LEFT SENSOR ESTIMATION OF DISTANCE FROM SIDE WALL, IT COMES FROM CALIBRATION
	if (x_left <= (1342 + d)) {
		y_left = 1000; //there's no wall on the left side, use only the right side
	}
	else if (x_left < 2419) {
		y_left = -0.2915*(x_left)+1635.6;
	}
	else if (x_left < 3359) {
		y_left = -0.1968*(x_left)+1395.4;
	}
	else if (x_left < 3778) {
		y_left = -0.3336*(x_left)+1831.7;
	}
	else {
		y_left = -1.3198*(x_left)+5558.4;
	}
	//RIGHT SENSOR ESTIMATION OF DISTANCE FROM SIDE WALL
	if (x_right <= (824 + 800)) {
		y_right = 993; //there's no wall on the right side, use only the left side
	}
	else if (x_right < 1808) {
		y_right = -0.5312*(x_right)+1995;
	}
	else if (x_right < 3464) {
		y_right = -0.1681*(x_right)+1309.3;
	}
	else {
		y_right = -0.6236*(x_right)+2879;
	}
	y_dif = y_right - y_left;

	//there's no walls on both sides, turn off sensor control
	if (x_right <= (824 + 200) & x_left <= (1342 + 200)) {
		y_dif = 0;
	}
}


void updateWalls(int(&walls)[width*height][4], int(&maze)[width][height],Floodfill &floodfill ) {
		/* --------------------------------State 1.1: Read Walls and update according to orientation -------------------------------*/
		SerialBT.println("State: Read Walls");
		cycle_sensor = 0;
		timerAlarmWrite(timer1, 100, true); // 100 * 1 us = 1000 us, autoreload true
		timerAlarmEnable(timer1); // enable
		control_translational.setK(70.0, 40.0, 0.03);
		control_rotational.setK(3.5, 2.2, 0.01); //Kp = 2.6, Ki = 1.6, Kd=0.008

		switch (orientation) {
		case 'N':
			p_indx = indx_N;
			break;
		case 'S':
			p_indx = indx_S;
			break;
		case 'W':
			p_indx = indx_W;
			break;
		case 'E':
			p_indx = indx_E;
			break;
		default:
			break;
		}
		while (cycle_sensor < 4);
		timerAlarmDisable(timer1);
		cycle_sensor = 0;
		wallReading();
		SerialBT.printf("y_left: %.1f ,y_right: %.1f , y_dif: %.1f\n", y_left, y_right, y_dif);
		SerialBT.printf("L: %d ,LF: %d, RF: %d, R: %d \n", receiver_reading[0], receiver_reading[1], receiver_reading[2], receiver_reading[3]);
		SerialBT.printf("F: %d ,0: %d , L: %d, R: %d \n", wall_reading[0], wall_reading[1], wall_reading[2], wall_reading[3]);

		for (unsigned int i = 0; i < 4; i++) {
			walls[currentPosition.x*width + currentPosition.y][i] = wall_reading[*(p_indx + i)];
		}
		int neighbour[4][2] = { {0,1},{0,-1} ,{-1,0} ,{1,0} }; //F,D,L,R
		int indx[4] = { 1,0,3,2 }; //updating interior and exterior walls
		for (unsigned int i = 0; i < 4; i++) {
			coordinate position(currentPosition.x + neighbour[i][0], currentPosition.y + neighbour[i][1]);
			if (position.x > (width - 1) || position.x < 0 || position.y >(height - 1) || position.y < 0) continue;
			walls[position.x*width + position.y][indx[i]] = *(walls[currentPosition.x*width + currentPosition.y] + i);
		}

		floodfill.generateMaze(maze, walls);
}

void controlLoop() {
		/* ----------------------------------State 2.1: Configure movement -------------------------------*/
		SerialBT.println("State: Configure Movement");
		switch (movement) {
		case 'F': //forward movement
			profile.v_list.clear();
			if (two_movements == true)
				profile.setPerfil(next_profile_distance); //it comes from a turn movement
			else
				profile.setPerfil(2772); //2672 //2790

			profile_length = profile.v_list.size();
			control_sensor.setK(0.06, 0, 0); //0.05
			control_sensor.setReference(0);
			cycle_sensor = 0;
			timerAlarmWrite(timer1, 100, true); // 100 * 1 us = 1000 us, autoreload true
			timerAlarmEnable(timer1); // enable
			two_movements = false;
			break;
		case 'L': //left movement
			profile_length = profile_left_turn.v_list.size();
			control_translational.setReference(0);
			two_movements = true;
			next_profile_distance = 2333;
			break;
		case 'R': //right movement
			profile_length = profile_right_turn.v_list.size();
			control_translational.setReference(0);
			two_movements = true;
			next_profile_distance = 2333;
			break;
		case 'B': //180° turn
			profile_length = profile_full_turn.v_list.size();
			control_translational.setReference(0);
			two_movements = true;
			next_profile_distance = 1740;
			break;
		default:
			break;
		}
		
		/* ----------------------------------State 2.2: Control loop of movement -------------------------------*/
		start_time = millis();
		SerialBT.println("State: Control loop");
		SerialBT.printf("profile length: %d \n", profile_length);

		while (i <= (profile_length - 1)) {
			elapsed_time = millis() - start_time;
			if (elapsed_time >= dt && (movement!='F' || cycle_sensor==4)) {
				start_time = millis();
				switch (movement) {
				case 'F': //forward
					sidesReading(y_dif);
					control_rotational.setReference(control_sensor.getPID(y_dif));
					control_translational.setReference(profile.v_list[i]);
					break;
				case 'L': //left
					control_rotational.setReference(profile_left_turn.v_list[i]);
					break;
				case 'R': //right
					control_rotational.setReference(profile_right_turn.v_list[i]);
					break;
				case 'B': //180° turn
					control_rotational.setReference(profile_full_turn.v_list[i]);
					break;
				default:
					break;
				}
				encoderLeft_Speed = encoderLeft.getSpeed();
				encoderRight_Speed = encoderRight.getSpeed();
				average_Speed = (encoderLeft_Speed + encoderRight_Speed) / 2;
				average_Speed = average_Speed / 5;
				control_output1 = control_translational.getPID(average_Speed);
				speed_difference = encoderRight_Speed - encoderLeft_Speed;
				control_output2 = control_rotational.getPID(speed_difference);
				motorLeft.setSpeed(control_output1 - control_output2);
				motorRight.setSpeed(control_output1 + control_output2);
				v_output.push_back(average_Speed);
				v_dif.push_back(speed_difference);
				control_output1_array.push_back(control_output1 - control_output2);
				control_output2_array.push_back(control_output1 + control_output2);
				i++;
				cycle_sensor = 0;
			}

		}
		motorLeft.setSpeed(0);
		motorRight.setSpeed(0);
		if (movement == 'F') {
			timerAlarmDisable(timer1);
			cycle_sensor = 0;
			for (unsigned char i = 0; i < 4; i++)
				digitalWrite(emitter_pin[i], 0); //turns OFF
		}
		i = 0;
		v_output.clear();
		v_dif.clear();
		control_output1_array.clear();
		control_output2_array.clear();
		control_rotational._integral = 0;
		control_translational._integral = 0;
}


void setup() {
	//----------------Communication Setup-------------------------------
	SerialBT.begin("ESP32_Micromouse");
	//----------------------------Wall Sensors Setup--------------------
	/*Timer 1: This time interruption toggles each one of the infrared sensors (4 in total) every 100us
	in order to avoid noise from the light bouncing between the walls*/
	timer1 = timerBegin(1, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
	timerAttachInterrupt(timer1, &onTimer1, true); // edge (not level) triggered 
	flag_sensor = 1;
	cycle_sensor = 0;
	pinMode(EMITTER_PIN_L, OUTPUT);
	digitalWrite(EMITTER_PIN_L, LOW);
	pinMode(EMITTER_PIN_LF, OUTPUT);
	digitalWrite(EMITTER_PIN_LF, LOW);
	pinMode(EMITTER_PIN_RF, OUTPUT);
	digitalWrite(EMITTER_PIN_RF, LOW);
	pinMode(EMITTER_PIN_R, OUTPUT);
	digitalWrite(EMITTER_PIN_R, LOW);
	//-------------------------Motor Setup ----------------------------
	motorLeft.setSpeed(0);
	motorRight.setSpeed(0);
	//-------------------------Controller Setup-------------------------
	profile.setSampling(dt);
	profile_left_turn.setSampling(dt);
	profile_right_turn.setSampling(dt);
	profile_full_turn.setSampling(dt);
	profile.setPerfil(2772); //2672 //2790
	profile_left_turn.setPerfil(13034); //13500 //14050 
	profile_right_turn.setPerfil(-13034);//13034
	profile_full_turn.setPerfil(23510);
	elapsed_time = 0;
	start_time = 0; //millis
	i = 0;
	flag_sequence = 0;

	//-------------------Algorithm--------------------------------------
	two_movements = false;
	next_profile_distance = 2772;
}

// the loop function runs over and over again until power down or reset
void loop() {

	int  maze[width][height] = { {0} };
	int  walls[width*height][4] = { {0} };
	Floodfill floodfill(startPosition, targetPosition);
	MotionPlanner motion;
	orientation = 'N';
	//Start program with an enter
	SerialBT.println("Waiting for input...");
	while (SerialBT.available() == 0);
	SerialBT.flush();

	/* /////////////////////////////// EXPLORE SEQUENCE /////////////////////////////////////////// */

	while ((!(currentPosition == startPosition)) || ((currentPosition == startPosition) && two_movements == true)) {

		/* ---------------------------State 1: Choose motion according to Walls -------------------*/
		updateWalls(walls, maze, floodfill);
		coordinate nextPosition = floodfill.getBestNeighbour(walls, currentPosition);
		movement = motion.getMovement(currentPosition, nextPosition);
		orientation = motion.getOrientation();
		SerialBT.printf("Orientarion: %c, Movement: %c , (%d , %d) \n", orientation, movement, currentPosition.x, currentPosition.y);

		/* ----------------------------------State 2: Configure & Control loop --------------------*/
		controlLoop();
		if (two_movements == true) {
			movement = 'F';
			controlLoop();
		}		
	}
	for (signed int i = (height - 1); i > -1; i--) {
		for (unsigned int j = 0; j < width; j++) {
			SerialBT.print(maze[j][i]);
			SerialBT.print(" ");
		}
		SerialBT.println();
	}

	/* /////////////////////////////////////// RETURN SEQUENCE //////////////////////////////////////*/

	vector <coordinate> finalpath = floodfill.getPath();
	for (unsigned int i = 1; i < finalpath.size(); i++) {

		/* ----------------------------------State 1: Motion according to path ---------------------*/
		orientation = motion.getOrientation();
		movement = motion.getMovement(currentPosition, finalpath[i]);
		SerialBT.printf("Orientarion: %c, Movement: %c \n", orientation, movement);

		/* ----------------------------------State 2: Configure & Control loop --------------------*/
		controlLoop();
		if (two_movements == true) {
			movement = 'F';
			controlLoop();
		}

	}
}

