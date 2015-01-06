/*
 * Plays back FPGA stereo data into a format suitable for the HUD.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2014
 *
 */

#include "fpga-playback.hpp"

using namespace std;

lcm_t * lcm;

vector<long> frame_table;
string fpga_img_path = "";
string publish_channel = "stereo_image_left";
long fpga_offset;
long fpga_delta = 8333; // 120 fps
long global_timestamp = -1;
int frame_offset = 0;


// globals for subscription functions, so we can unsubscribe in the control-c handler
lcmt_deltawing_u_subscription_t *servo_out_sub;

int main(int argc,char** argv) {

    Scalar bm_color(.5, .5, .5);
    Scalar block_match_color(.8, 0, 0);
    Scalar block_match_fill_color(1, 1, 1);
    Scalar octomap_color(1, 0, .4);

    string listen_channel = "servo_out";
    string fpga_log_filename = "";


    ConciseArgs parser(argc, argv);
    parser.add(fpga_log_filename, "f", "fpga-log", "FPGA log file in CSV format.", true);
    parser.add(fpga_img_path, "i", "image-path", "Path to FPGA images.", true);
    parser.add(listen_channel, "l", "listen-channel", "Channel to listen for deltawing_u messages on.");
    parser.add(publish_channel, "p", "publish-channel", "Channel to publish images on.");
    parser.add(fpga_delta, "d", "frame-delta", "Frame delta in microseconds. Default 120fps");
    parser.add(frame_offset, "o", "frame-offset", "Offset frames by this amount.");
    parser.parse();


    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    servo_out_sub = lcmt_deltawing_u_subscribe(lcm, listen_channel.c_str(), &servo_out_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    // build FPGA data from file

    cout << "Loading FPGA data from " << fpga_log_filename << endl;



    io::CSVReader<2> csv_reader(fpga_log_filename);
    csv_reader.read_header(io::ignore_extra_column, "frame", "vicon_time_us");
    int frame;
    long timestamp;

    while (csv_reader.read_row(frame, timestamp)) {
        // load this row

        frame_table.insert(frame_table.begin()+frame, timestamp);

    }

    cout << "Done reading CSV file." << endl;

    // the table is not super-accurate, so figure out a constant
    // timestamp for each frame

    if (frame_table.size() < 200) {
        cout << "Error frame table smaller than 200 frames (minus ignored frames)." << endl;
        return -1;
    }

    fpga_offset = frame_table.at(200) - 200*fpga_delta;

    cout << "FPGA offset = " << fpga_offset << endl;

    //fpga_img_path = fpga_log_filename.substr(0, fpga_log_filename.find_last_of("/"));

    cout << "img path: " << fpga_img_path << endl;

    //namedWindow("Input", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);

    bool change_flag = false;

    while (true) {
        // read the LCM channel
        while (NonBlockingLcm(lcm)) {
            change_flag = true;
        }

        if (change_flag == true) {
            change_flag = false;

            int frame_number = GetNearestFpgaFrame(global_timestamp);

            cout << frame_number << " vs " << GetNearestFpgaFrameFromTable(global_timestamp) << endl;

            string filename = GetFpgaImageFilename(frame_number);

            cout << filename << endl;

            Mat image = imread(filename);

            //imshow("Input", image);

            //waitKey(10);


            SendImageOverLcm(lcm, publish_channel, image, -1);
        }

    }

    return 0;
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}

// from: http://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
{
    std::vector<std::string>   result;
    std::string                line;
    std::getline(str,line);

    std::stringstream          lineStream(line);
    std::string                cell;

    while(std::getline(lineStream,cell,','))
    {
        result.push_back(cell);
    }
    return result;
}

int GetNearestFpgaFrame(long timestamp) {

    if (timestamp < fpga_offset) {
        return 0 + frame_offset;
    } else if (timestamp > (int) frame_table.size() * fpga_delta + fpga_offset) {
        return frame_table.size() - 1 + frame_offset;
    } else {

        return (timestamp - fpga_offset) / fpga_delta + frame_offset;
    }
}

int GetNearestFpgaFrameFromTable(long timestamp) {
    // a binary search would be more efficient

    long best_diff = -1, this_diff;
    int best_frame = -1;

    bool first = true;



    for (int i = 0; i < (int) frame_table.size(); i++) {

        long this_time = frame_table.at(i);

        this_diff = abs(this_time - timestamp);

        if (first == true || this_diff < best_diff) {
            first = false;

            best_diff = this_diff;
            best_frame = i;
        }

    }

    return best_frame;
}

string GetFpgaImageFilename(int frame_number) {

    boost::format fmt("%s/%d.png");

    fmt % fpga_img_path % frame_number;

    return fmt.str();
}




void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user) {

    // got a servo message

    // use the timestamp to figure out the closest frame from the FPGA log, read that frame, and publish it

    global_timestamp = msg->timestamp;

}

/**
 * Processes LCM messages without blocking.
 *
 * @param lcm lcm object
 *
 * @retval true if processed a message
 */
bool NonBlockingLcm(lcm_t *lcm)
{
    // setup an lcm function that won't block when we read it
    int lcm_fd = lcm_get_fileno(lcm);
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcm_fd, &fds);

    // wait a limited amount of time for an incoming message
    struct timeval timeout = {
        0,  // seconds
        1   // microseconds
    };


    int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

    if(0 == status) {
        // no messages
        //do nothing
        return false;

    } else if(FD_ISSET(lcm_fd, &fds)) {
        // LCM has events ready to be processed.
        lcm_handle(lcm);
        return true;
    }
    return false;

}

