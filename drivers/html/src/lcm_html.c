#include <stdlib.h>
#include <stdio.h>
#include "../../../LCM/lcmt_html.h"
#include <time.h>
#include <sys/time.h>
#include <getopt.h>

#define INIT (-1)
#define RELEASED 1
#define HELD 0


void main( int argc, unsigned char *argv[] )
{

    int heldVar;
    int c;
    
    lcm_t * lcm;
    
    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
        return 1;
    
    struct timeval thisTime;
    
    heldVar = INIT;

    while (1) {
        int this_option_optind = optind ? optind : 1;
        int option_index = 0;
        static struct option long_options[] = {
            {"grabbed", 0, 0, 'g'},
            {"released", 0, 0, 'r'},
            {"help", 0, 0, 'h'},
            {0, 0, 0, 0}
        };

        c = getopt_long (argc, argv, "grh",
                 long_options, &option_index);
        if (c == -1)
            break;

        switch (c) {
            /*
            * Use this case if you have an option that does not correspond to a shorter version
            */
            /*
            case 0:
                printf ("option %s", long_options[option_index].name);
                if (optarg)
                    printf (" with arg %s", optarg);
                printf ("\n");
                break;
            */
            case 'g':
                heldVar = HELD;
                break;
            case 'r':
                heldVar = RELEASED;
                break;
                
            case 'h':
            case '?':
                printf ("\n");
                printf ("Usage: lcm-html (g|r)\n");
                printf ("Publishes if the airplane is held or released over LCM\n");
                printf ("\n");
                printf ("Options:\n");
                printf ("  -g, --grabbed\t\tplane is being held\n");
                printf ("  -r, --released\tplane is NOT being held\n");
                printf ("\  -h, --help\t\tdisplay this message and exit\n");
                printf ("\n");
                
                exit(0);
                break;

            default:
                printf ("?? getopt returned character code 0%o ??\n", c);
                exit(-1);
        }
    }
    
    if (optind < argc) {
        printf ("non-option ARGV-elements: ");
        while (optind < argc)
            printf ("%s ", argv[optind++]);
        printf ("\n");
    }



    lcmt_html msg;

    gettimeofday(&thisTime, NULL);
    msg.timestamp = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;

    msg.held = heldVar;
            
            

    lcmt_html_publish (lcm, "hotrod_html", &msg);
    
    if (heldVar == INIT)
    {
        printf("You didn't specify -r or -g, so we just gave up and put -1 on the channel.");
    }

}
