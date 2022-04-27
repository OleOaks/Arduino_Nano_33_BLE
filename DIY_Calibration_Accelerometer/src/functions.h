
void printParam(char txt[], float param[3]);
void printSetParam(char txt[], float param[3]);
void calibrateAccelMenu(uint8_t accelFSindex, uint8_t accelODRindex);
void calibrateAccel(uint16_t NofSamples);
char readChar();
void readAnswer(char msg[], uint16_t& param);
void raw_N_Accel(uint16_t N, float& averX, float& averY, float& averZ);

