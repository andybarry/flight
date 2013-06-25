#ifndef PROCESS_CONTROL_PROC_H
#define PROCESS_CONTROL_PROC_H

#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <pty.h>
#include <iostream>
#include <signal.h>

using namespace std;

class ProcessControlProc
{
    public:
        ProcessControlProc(char **argumentsIn, int numArgsIn);
        pid_t StartProcess();
        void StopProcess();
        void PrintIO();
        
    private:
        char **arguments;
        pid_t pid;
        int numArgs;
        int my_fd;

};


#endif
