/*
 * Reads an MP4 file from the GoPro and plays it back into a format that
 * the HUD can read, synced to LCM message playback.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "video-playback.hpp"
#include "../../externals/ConciseArgs.hpp"

lcm_t * lcm_;

std::string video_channel = "ground-video";
std::string servo_out_channel = "servo_out";

int main(int argc,char** argv) {

    std::string filename;

    ConciseArgs parser(argc, argv);
    parser.add(video_channel, "v", "video-channel", "LCM channel to send video messages on.");
    parser.add(servo_out_channel, "s", "servo-out-channel", "LCM channel to receive servo messages on (for sync).");
    parser.add(filename, "f", "file", "Video file to play", true);
    parser.parse();


    lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

    if (!lcm_)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    // control-c handler
    signal(SIGINT,sighandler);

    // open the MP4 file



    GstElement *pipeline, *sink;
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;

    /* Initialize GStreamer */
    gst_init (&argc, &argv);

    /* Build the pipeline */
    std::string gstreamer_uri = "playbin2 uri=file://" + filename;
    pipeline = gst_parse_launch(gstreamer_uri.c_str(), NULL);
    //pipeline = gst_parse_launch("playbin2 uri=http://docs.gstreamer.com/media/sintel_trailer-480p.webm", NULL);

    //sink = gst_element_factory_make ("mfw_v4lsink", "sink");


    if (!pipeline) {
        g_printerr ("Pipeline could be created.\n");
        return -1;
    }

    //if (!sink) {
      //  g_printerr ("sink could be created.\n");
      //  return -1;
    //}

    //g_object_set( GST_OBJECT(pipeline), "video-sink", sink, NULL);

    /* Build the pipeline */
    //gst_bin_add_many (GST_BIN (pipeline), source, sink, NULL);
    //if (gst_element_link (source, sink) != TRUE) {
        //g_printerr ("Elements could not be linked.\n");
        //gst_object_unref (pipeline);
        //return -1;
    //}

  /* Modify the source's properties */
  //g_object_set (source, "pattern", 0, NULL);

  /* Start playing */
    ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to set the pipeline to the playing state.\n");
        gst_object_unref (pipeline);
        return -1;
    }

    /* Start playing */
//    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    /* Wait until error or EOS */
    bus = gst_element_get_bus (pipeline);
    msg = gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE, GstMessageType( GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    /* Free resources */
    if (msg != NULL) {
        gst_message_unref (msg);
    }
    gst_object_unref (bus);
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (pipeline);
/*
    printf("Sending LCM:\n\t%s\n", video_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm_);
    }

    return 0;
    */
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcm_destroy (lcm_);

    printf("done.\n");

    exit(0);
}
