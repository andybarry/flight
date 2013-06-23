#ifndef PROCESS_CONTROL_PROC_H
#define PROCESS_CONTROL_PROC_H

#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <pty.h>

using namespace std;

class ProcessControlProc
{
    public:
        ProcessControlProc(char **argumentsIn, int numArgsIn);
        pid_t StartProcess();
        void StopProcess();
        
    private:
        char **arguments;
        pid_t pid;
        int numArgs;

};


#endif
