#ifndef ROBOT_IO
#define ROBOT_IO

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define lerp(a,b,c) (((b) - (a)) * (c) + (a))
#define TableSize 60

extern volatile double Humid, Temp, Pres, Mag;
extern volatile double roll, pitch, yaw;
extern volatile int32_t Altitude, Satellites;

extern volatile double table[4][TableSize];
extern volatile double t_counter;

extern void rawmotor(int32_t, int32_t);
extern void rawservo(int32_t, int32_t);
extern void gnd(int32_t, int32_t);
extern double aRead(int32_t);
extern double lux(void);
extern double dist(void);
extern void ioinit(void);
extern double vbat(void);
extern double cbat(double);
extern void updateSensors(void);
extern double mlxTemp(void);

extern void motorsinit(void);
extern bool servo(int32_t, double);
extern double getServo(int32_t);
extern bool moveFin();
extern void move(double dist, double speed);
extern void rotate(double dist, double speed);
extern void rotate_cmp(double angle, double speed);
#endif