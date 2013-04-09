#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <getopt.h>
#include <inttypes.h>
#include <arpa/inet.h>
#ifndef __APPLE__
// TODO: This just masks symptoms, need to find proper fix.
#include <sys/timex.h>
#endif
#include <math.h>

#include <time.h>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <lcmtypes/timesync.h>

// if this node is forced to be a slave, it's ID will be at least this
// big
#define SLAVE_ID (RAND_MAX+1LL)
#define TIMESYNC_DEFAULT_PERIOD 1000

typedef struct
{
    lcm_t     *lcm;
    int verbose;
    int64_t   my_id;       // signed type, but guaranteed positive
    int64_t   master_id;
    int64_t   next_nonce;

    int64_t   last_request_nonce;
    int64_t   last_request_send_time;
    int64_t   last_request_response_time;


    int       got_master_message;
    int       sent_master_message;

    double    mean_rtt;
    double    mean_rtt_gain; // [0,1], values near 1 mean faster adjustments of mean_rtt

    double    mean_err;
    int       mean_count;
    int       period;
    int       nop_flag;
} state_t;

void adjust_time(double err)
{
    if (fabs(err) > 0.010)
    {
        printf("    Warping by %8.2fms", - err * 1000);

        struct timespec ts;
        bot_timespec_now(&ts);
        bot_timespec_adjust(&ts, - err);
#ifndef __APPLE__
        clock_settime(CLOCK_REALTIME, &ts);
#endif
    }
    else
    {
        printf("    Slewing by %8.2fms", - err*1000);

        struct timeval delta, olddelta;

        bot_timeval_set(&delta, -err);
        int res = adjtime(&delta, &olddelta);
        if (res)
            perror("adjtime");
    }
}

void update_time_estimate(state_t *state, double err, double rtt)
{
    if (state->mean_rtt == 0) {
        state->mean_rtt = rtt;
    } else {
        state->mean_rtt = (1.0-state->mean_rtt_gain)*state->mean_rtt +
            state->mean_rtt_gain * rtt;
    }

    printf("[%"PRId64"]: ERR: %8.2fms     RTT: %8.2fms     AVGRTT %8.2fms   ", state->last_request_send_time,
            err*1000, rtt*1000, state->mean_rtt*1000);

    // it's a large error, greater than the rtt even. Jump immediately.
    if (rtt < fabs(err) && fabs(err) > 0.10) {
        if (state->nop_flag)
            printf("[%"PRId64"]: NOACTION (timesync in nop mode)",state->last_request_send_time);
        else
            adjust_time(err);
        goto exit;
    }

    // don't do updates on samples with unusually high RTTs
    // We bias this comparison by 0.1ms so that if we have really consistent
    // RTTs, we won't eliminate every update.
    if (rtt > (state->mean_rtt + 0.0001)) {
        printf("    (RTT > AVGRTT reject)");
        goto exit;
    }

    // if the error is really small, don't warp.
    if (fabs(err) < 0.001) {
        printf("    (no adjust)");
        goto exit;
    }

    // changed by prentice - increased threshold from 1ms -> 5ms
    if (state->mean_rtt > 0.005 ) {
        printf("    (AVGRTT > 1ms reject)");
        goto exit;
    }
    if (state->nop_flag)
        printf("[%"PRId64"]: NOACTION (timesync in nop mode)",state->last_request_send_time);
    else
        adjust_time(err);
exit:
    printf("\n");
}

int elect(state_t *state, int64_t id)
{
    if (id >= SLAVE_ID || id == 0)
        return 0;

    if (id < state->master_id) {
        state->master_id = id;
        return 1;
    }

    return 0;
}

static void
timesync_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                                      const timesync_update_t *ts, void *userdata)
{
    state_t *state = (state_t*) userdata;

    // update master election
    int elected = elect(state, ts->requester_id) |
                  elect(state, ts->responder_id);

    if (elected) {
        int is_me = state->master_id == state->my_id;

        printf("[%"PRId64"]: new master %08x %s\n", state->last_request_send_time,
                (int) state->master_id,
                is_me ? ", that's me!" : "");

        // if we're the master, stop any slewing that we might have been doing.
        if (is_me) {
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 0;
            adjtime(&tv, NULL);
        }
    }

    // if we got a message from the master, remember the fact.  we use
    // this to elect new masters as
    // necessary
    if (state->master_id == ts->responder_id ||
        state->master_id == ts->requester_id) {
        state->got_master_message = 1;
    }

    // am I the master?
    if (state->master_id == state->my_id) {

        if (ts->responder_utime == 0) {
            // this is a request, and we're the master. send a response.
            timesync_update_t resp;
            resp.requester_id    = ts->requester_id;
            resp.requester_nonce = ts->requester_nonce;
            resp.responder_id    = state->my_id;
            resp.responder_utime = bot_timestamp_now();
            timesync_update_t_publish(state->lcm, "TIMESYNC", &resp);
        }

    } else if (ts->requester_id == state->my_id && ts->responder_utime) {

        // this is a response to one of our messages.
        if (ts->requester_nonce != state->last_request_nonce) {
            printf("Got stale response\n");
            return;
        }

        int64_t now = bot_timestamp_now();
        int64_t rtt = now - state->last_request_send_time;
        int64_t err = state->last_request_send_time +
                      rtt/2 - ts->responder_utime;

        state->last_request_response_time = now;
        update_time_estimate(state,
                (double) err/1000000.0, (double) rtt/1000000.0);
    }
}

void poll_master(void *userdata)
{
    timesync_update_t ts;
    state_t *state = (state_t*) userdata;

    if (state->sent_master_message == 10) {
        if (state->got_master_message == 0) {
            printf("Master died! Re-electing\n");
            state->master_id = SLAVE_ID;
        }
        state->sent_master_message = 0;
        state->got_master_message = 0;
    }

    ts.requester_id    = state->my_id;
    ts.requester_nonce = state->next_nonce++;
    ts.responder_id    = 0;
    ts.responder_utime = 0;

    state->last_request_nonce = ts.requester_nonce;
    state->last_request_send_time = bot_timestamp_now();

    timesync_update_t_publish(state->lcm, "TIMESYNC", &ts);

    state->sent_master_message++;
}

static void usage()
{
    fprintf (stderr, "usage: ar-timesync [options]\n"
             "\n"
             "  -h, --help             shows this help text and exits\n"
             "  -v, --verbose          be verbose\n"
             "  -p, --period <msec>    Period (in ms) to query the master\n"
             "  -s, --slave            Force slave operation\n"
             "  -m, --master           Force master operation\n"
             "  -n, --nop              don't actually set time (for debugging timesync)\n"
             "  -l, --logfile <fname>  Write output to logfile\n"
        );
}


int main(int argc, char *argv[])
{
    srand(bot_timestamp_now());

    state_t *state = (state_t*) calloc(1, sizeof(state_t));
    state->next_nonce = rand();
    while (state->my_id == 0)
        state->my_id = rand();         // forbid id of 0

    state->master_id = SLAVE_ID;
    state->mean_rtt_gain = 0.05;

    char *optstring = "hp:vsml:n";
    struct option long_opts[] = {
        {"help", no_argument, 0, 'h'},
        {"verbose", no_argument, 0, 'v'},
        {"period", required_argument, 0, 'p'},
        {"slave", no_argument, 0, 's'},
        {"master", no_argument, 0, 'm'},
        {"nop", no_argument, 0, 'n'},
        {"logfile", required_argument, 0, 'l'},
        {0, 0, 0, 0}};


    state->period = TIMESYNC_DEFAULT_PERIOD;
    char *logfilename = NULL;
    int c=-1;
    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0)
    {
        switch (c)
        {
        case 'p':
            state->period = atoi(optarg);
            if (state->period<1) {
                printf("ERROR: can't set period <1msec:%s (%d)\n",optarg,state->period);
                state->period = TIMESYNC_DEFAULT_PERIOD;
            }
            break;
        case 's':
            if (state->verbose)
                printf("TIMESYNC: forcing slave mode\n");
            state->my_id += SLAVE_ID;
            break;
        case 'm':
            if (state->verbose)
                printf("TIMESYNC: forcing master mode\n");
            break;
        case 'l':
            logfilename = strdup (optarg);
            break;
        case 'v':
            state->verbose = 1;
            break;
        case 'n':
            state->nop_flag = 1;
            break;
        case 'h':
        default:
            usage();
            return 1;

        }
    }


     // redirect stdout and stderr to a log file if the -l command line flag
     // was specified.
    if (logfilename && strlen (logfilename)) {
        int fd = open (logfilename, O_WRONLY | O_APPEND | O_CREAT, 0644);
        if (fd < 0) {
            perror ("open");
            fprintf (stderr, "couldn't open logfile %s\n", logfilename);
            return 1;
        }
        close(1); close(2);
        if (dup2(fd, 1) < 0) { return 1; }
        if (dup2(fd, 2) < 0) { return 1; }
        close (fd);
        setlinebuf (stdout);
        setlinebuf (stderr);
    }

    // line-buffer stdout instead of block-buffer.  This is needed when used
    // with procman
    setlinebuf( stdout );

    // create the lcm_t structure for doing IPC
    state->lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    if( !state->lcm ) {
        fprintf(stderr, "error allocating LC.\n" );
        return 1;
    }

    if (state->verbose)
        printf("TIMESYNC: my id: %08x\n", (int) state->my_id);
    timesync_update_t_subscribe( state->lcm, "TIMESYNC", timesync_handler, state );

    int lcm_fd = lcm_get_fileno(state->lcm);

    struct timespec poll_ts;
#ifndef __APPLE__
    clock_gettime(CLOCK_MONOTONIC, &poll_ts);
#endif
    
    while (1) {
        struct timespec now_ts;
#ifndef __APPLE__
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
#endif
        if (bot_timespec_compare(&now_ts, &poll_ts) > 0) {
            poll_master(state);

            poll_ts = now_ts;
            int interval = state->period + state->period*rand()/RAND_MAX;
            bot_timespec_addms(&poll_ts, interval);
        }

        struct timespec wait_ts = poll_ts;
        bot_timespec_subtract(&wait_ts, &now_ts);

        struct timeval timeout;
        bot_timespec_to_timeval(&wait_ts, &timeout);

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(lcm_fd, &readfds);

        int   status = select(lcm_fd + 1, &readfds, 0, 0, &timeout);
        if (status < 0) {
            perror("select");
            return -1;
        }

        if (FD_ISSET(lcm_fd, &readfds)) {
            lcm_handle(state->lcm);
        }
    }

    // release resources
    printf("cleaning up\n");
    lcm_destroy( state->lcm );
    free (state);

    return 0;
}
