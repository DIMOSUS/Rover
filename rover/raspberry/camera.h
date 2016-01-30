#ifndef CAMERA
#define CAMERA
extern volatile bool IR;

extern void camerainit(void);
extern void setirt(int32_t);
extern std::string takeShot(bool);
extern std::string takePanoram(bool);
extern std::string takeIRShot();
#endif