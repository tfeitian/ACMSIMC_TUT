#ifndef LOG_SOCKET_H
#define LOG_SOCKET_H

int socket_vinit(void);
int socket_vClose();
void socket_vSend(char *name, float value);

#endif // !LOG_SOCKET_H
