#ifndef RADIO
#define RADIO
extern void radioinit(void);
extern void SendText(std::string);
extern void SendFile(std::string);
void regRXstrfile(void (*strf)(std::string), void (*filef)(std::string));
void setLag(uint32_t);
#endif