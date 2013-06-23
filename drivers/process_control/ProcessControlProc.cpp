#include "ProcessControlProc.hpp"


ProcessControlProc::ProcessControlProc(char **argumentsIn, int numArgsIn)
{
    arguments = argumentsIn;
    numArgs = numArgsIn;
    pid = -1;
}


pid_t ProcessControlProc::StartProcess() // the first argument in processArgs must be the file to execute
{
    if (pid > 0)
    {
        // this process is already running, don't do anything
        return pid;
    }
    
    int stdin_fd;
    
    // this next line forks the program, so that means that
    // it's ok for this function to block, since it will be
    // in another program with another PID
    int pid = forkpty(&stdin_fd, NULL, NULL, NULL);

    if (0 == pid) {
        // this branch will execute in the child process
        
        // go!
        execvp (arguments[0], arguments);

        fprintf (stderr, "ERROR!!!! couldn't start [%s]!\n", arguments[0]);

        // if execv returns, the command did not execute successfully
        // (e.g. permission denied or bad path or something)

        exit(-1);
    } else if (pid < 0) {
        fprintf(stderr, "forkpty ERROR");
        return -1;
    } else {
        // this branch will execute in the parent process
        return pid;
    }
}

void ProcessControlProc::StopProcess()
{
    if (pid <= 0)
    {
        // already stopped, don't do anything
        return;
    }
    //kill(pid);
}


// TODO DESTRUCTOR
