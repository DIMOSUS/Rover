#ifndef RADIO
#define RADIO
extern void radioinit(void);
extern void SendText(std::string);
extern bool SendFile(std::string, std::string);
void regRXstrfile(void (*strf)(std::string), void (*filef)(std::string));
void setLag(unsigned int);
#endif