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
    //int pid = forkpty(&stdin_fd, NULL, NULL, NULL);
    pid = fork();
    if (0 == pid) {
        // this branch will execute in the child process
        // go!
        int count = 0;
        printf("Starting ");
        while (arguments[count] != NULL)
        {
            printf(" %s", arguments[count]);
            count ++;
        }
        printf("\n");

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
        my_fd = stdin_fd;
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
    printf("Stopping %s, pid = %d\n", arguments[0], pid);
    if (kill(pid, SIGINT) == 0)
    {
        pid = -1;
    } else {
        // error
        printf("ERROR: failed to kill %s with pid = %d\n", arguments[0], pid);
    }
}

bool ProcessControlProc::IsAlive()
{
    // issue a kill with a null signal for the pid
    if (pid <= 0)
    {
        return false;
    }

    if (kill(pid, 0) == 0)
    {
        // process exists
        return true;
    } else {

        return false;
    }
}

void ProcessControlProc::PrintIO()
{
    printf("printing io...\n");
    // do non-blocking IO for this

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(my_fd, &fds);

    // wait a limited amount of time for an incoming message
    struct timeval timeout = {
        0,  // seconds
        1   // microseconds
    };


    int status = select(my_fd + 1, &fds, 0, 0, &timeout);

    if(0 == status) {
        // no messages
        //do nothing
        printf("no data\n");

    } else if(FD_ISSET(my_fd, &fds)) {
        // data is on the IO port
        printf("got data...\n");
        char buf[1024];
        int bytes_read = read (my_fd, buf, sizeof (buf)-1);
        if (bytes_read > 0)
        {
            printf("------ %s -----\n %s\n--------------------\n", arguments[0], buf);
        }
    }

}


// TODO DESTRUCTOR
