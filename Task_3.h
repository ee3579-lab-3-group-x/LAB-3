#ifndef Task_3_h
#define Task_3_h

class delta_speed{
public:
float delta_speed_fun(int inp_speed, float delta_percent, int time_durr, bool add)
{
	float new_output_speed;													//Blank Local Variable
	
	if (delta_percent >= 100)												//conditional dissalows invalid selection i.e 110%
	{
		new_output_speed = inp_speed;										//Return Input speed as safety measure
		return new_output_speed;											//Return Calculated Value
	}
	
	if(add == true)
	{
		new_output_speed = inp_speed + (inp_speed*(delta_percent/100));		//Add Percentage
		return new_output_speed;											//Return Calculated Value
	}else if(add==false){
		new_output_speed = inp_speed - (inp_speed*(delta_percent/100));		//Subtract Percentage
		return new_output_speed;											//Return Calculated Value
	}	
}
};
#endif

