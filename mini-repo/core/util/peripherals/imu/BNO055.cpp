#include "BNO055.h"
#include "mbed.h"

BNO055::BNO055(I2C &i2c, uint8_t addr) noexcept
    : //_i2c_p(&i2c),
      _i2c(i2c), 
      _res(NC),
      chip_addr(addr),
      cantReadDataCount(0)
{
    
}

BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept
    : //_i2c_p(&i2c),
      _i2c(i2c),
      _res(p_reset, 0),
      chip_addr(addr), 
      cantReadDataCount(0)
{
    
}

void BNO055::init() noexcept{
    _i2c.frequency(100000); // 100Khz

    dt[0] = UNIT_SEL;
    dt[1] = 0x00; // Windows orientation; Euler Degrees; GYR Dps; ACC m/s^2
    _i2c.write(chip_addr, dt, 2, false);

    dt[0] = OPR_MODE;
    dt[1] = NDOF_MODE;
    _i2c.write(chip_addr, dt, 2, false);
    ThisThread::sleep_for(400);

    cantReadDataCount = 0;
}    

void BNO055::reset() noexcept{
    if (_res.read() != NC){
        _res.write(0);
        ThisThread::sleep_for(1);
        _res.write(1);
        ThisThread::sleep_for(650);
    } else {
        dt[0] = SYS_TRIGGER;
        dt[1] = 0x20; // (bit 5) RST_SYS = 1
        _i2c.write(chip_addr, dt, 2, false);
    }
    init();
}

/** Get Accel data
 * @param double type of 3D data address
 */
void BNO055::get_accel(BNO055_VECTOR_TypeDef *la){
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t x,y,z;
    dt[0] = {ACC_DATA_X_LSB};
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult){
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 6, false);

        x = dt[1] << 8 | dt[0];
        y = dt[3] << 8 | dt[2];
        z = dt[5] << 8 | dt[4];

        la -> x = (double) x / 100.0; // in m/s^2
        la -> y = (double) y / 100.0;
        la -> z = (double) z / 100.0;

    } else {
        cantReadDataCount++;
    }
}



/** Get Gyro data
 * @param double type of 3D data address
 */
void BNO055::get_gyro(BNO055_VECTOR_TypeDef *gr){
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t x,y,z;
    dt[0] = {GYR_DATA_X_LSB};
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult){
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 6, false);

        x = dt[1] << 8 | dt[0];
        y = dt[3] << 8 | dt[2];
        z = dt[5] << 8 | dt[4];

        gr -> x = (double) x / 16.0; // in Dps
        gr -> y = (double) y / 16.0;
        gr -> z = (double) z / 16.0;
    } else {
        cantReadDataCount++;
    }
}

/** Change fusion mode
  * @param fusion mode
  * @return none
  */


void BNO055::change_fusion_mode(uint8_t mode){
    dt[0] = OPR_MODE;
    dt[1] = mode;
    _i2c.write(chip_addr, dt, 2, true);
}

IMU::EulerAngles BNO055::read(){ //
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return imuAngles;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t yaw, roll, pitch;
    dt[0] = {EUL_DATA_X_LSB};
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult){
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 6, false);

        yaw = dt[1] << 8 | dt[0];
        roll = dt[3] << 8 | dt[2];
        pitch = dt[5] << 8 | dt[4];

        imuAngles.yaw = (float) yaw / 16.0; // in degrees
        imuAngles.roll = (float) roll / 16.0;
        imuAngles.pitch = (float) pitch / 16.0;
    } else {
        cantReadDataCount++;
    }
    return imuAngles;
}

IMU::EulerAngles BNO055::getImuAngles(){
    return imuAngles;
}

/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us 
to nicely talk about rotation, and the functions spit out pitch roll and yaw. 
*/
void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
}
