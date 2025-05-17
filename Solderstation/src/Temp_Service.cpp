#include <Temp_Service.h>

void Temp_Init()
{
    analogReadResolution(12);
    pinMode(Soldering_Pin, INPUT);
    pinMode(Heatgun_Pin, INPUT);
}

void Temp_Task()
{
    SolderingTemp = analogRead(Soldering_Pin) / 5.75f;   
    HeatgunTemp = analogRead(Heatgun_Pin);
    Serial.print("Soldering Temp: ");
    Serial.print(SolderingTemp);
    Serial.print(" Heatgun Temp: ");
    Serial.println(HeatgunTemp);
}