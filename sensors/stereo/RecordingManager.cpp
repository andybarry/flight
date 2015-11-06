#include "RecordingManager.hpp"

RecordingManager::RecordingManager() {

    using_video_from_disk_ = false;
    using_video_directory_ = false;
    current_video_number_ = -1;
    record_hud_setup_ = false;
    video_directory_ = "";
    video_number_ = -1;
    hud_video_number_ = -1;

    quiet_mode_ = false;

    left_video_capture_ = NULL;
    right_video_capture_ = NULL;

    init_ok_ = false;

}

RecordingManager::~RecordingManager() {

    if (left_video_capture_) {
        delete left_video_capture_;
    }

    if (right_video_capture_) {
        delete right_video_capture_;
    }
}

void RecordingManager::Init(OpenCvStereoConfig stereo_config) {

    stereo_config_ = stereo_config;
    init_ok_ = true;

}

bool RecordingManager::InitRecording(Mat image_left, Mat image_right) {

    if (init_ok_ != true) {
        cerr << "Error: You must call Init() before using RecordingManager." << endl;
        return false;
    }

    rec_num_frames_ = 0;

    // allocate a huge buffer for video frames
    printf("Allocating ringbuffer data... ");
    fflush(stdout);

    for (int i=0; i<RINGBUFFER_SIZE; i++)
    {
        ringbufferL[i].create(image_left.size(), image_left.type());
        ringbufferR[i].create(image_right.size(), image_right.type());
    }

    printf("done.\n");

    BeginNewRecording();

    return true;

}

/**
 * Sets up variables to start recording on the next frame.
 */
void RecordingManager::BeginNewRecording() {

    // get a new filename
    if (video_number_ < 0) {
        video_number_ = GetNextVideoNumber(stereo_config_.usePGM, true);
    } else {
        video_number_ ++;
    }

    // reset the number of frames we've recorded
    rec_num_frames_ = 0;

    recording_on_ = true;
}

void RecordingManager::AddFrames(Mat image_left, Mat image_right) {

    if (recording_on_) {
        ringbufferL[rec_num_frames_%RINGBUFFER_SIZE] = image_left;
        ringbufferR[rec_num_frames_%RINGBUFFER_SIZE] = image_right;

        rec_num_frames_ ++;
    }
}

void RecordingManager::FlushBufferToDisk() {


    printf("Writing video...\n");

    int endI, firstFrame = 0;
    if (rec_num_frames_ < RINGBUFFER_SIZE)
    {
        endI = rec_num_frames_;
    } else {
        // our buffer is smaller than the full movie.
        // figure out where in the ringbuffer we are
        firstFrame = rec_num_frames_%RINGBUFFER_SIZE+1;
        if (firstFrame > RINGBUFFER_SIZE)
        {
            firstFrame = 0;
        }

        endI = RINGBUFFER_SIZE;

        printf("\nWARNING: buffer size exceeded by %d frames, which have been dropped.\n\n", rec_num_frames_ - RINGBUFFER_SIZE);

    }

    if (stereo_config_.usePGM) {
        printf("Using PGM format...\n");

        string video_l_dir = SetupVideoWriterPGM("videoL-skip-" + std::to_string(firstFrame), true);

        string video_r_dir = SetupVideoWriterPGM("videoR-skip-" + std::to_string(firstFrame), false);

        // write the video
        for (int i = 0; i < endI; i++) {

            boost::format formatter_left = boost::format("/left%05d.pgm") % i;
            string im_name_left = formatter_left.str();

            boost::format formatter_right = boost::format("/right%05d.pgm") % i;
            string im_name_right = formatter_right.str();

            imwrite( video_l_dir + im_name_left, ringbufferL[(i+firstFrame)%RINGBUFFER_SIZE]);

            imwrite( video_r_dir + im_name_right, ringbufferR[(i+firstFrame)%RINGBUFFER_SIZE]);

            if (quiet_mode_ == false || i % 100 == 0) {
                printf("\rWriting video: (%.1f%%) -- %d/%d frames", (float)(i+1)/endI*100, i+1, endI);
                fflush(stdout);
            }
        }


    } else {
        VideoWriter recordL = SetupVideoWriterAVI("videoL-skip-" + std::to_string(firstFrame), ringbufferL[0].size(), true);

        VideoWriter recordR = SetupVideoWriterAVI("videoR-skip-"
            + std::to_string(firstFrame), ringbufferR[0].size(), false);

        // write the video
        for (int i=0; i<endI; i++)
        {
            recordL << ringbufferL[(i+firstFrame)%RINGBUFFER_SIZE];
            recordR << ringbufferR[(i+firstFrame)%RINGBUFFER_SIZE];

            if (quiet_mode_ == false || i % 100 == 0) {
                printf("\rWriting video: (%.1f%%) -- %d/%d frames", (float)(i+1)/endI*100, i+1, endI);
                fflush(stdout);
            }
        }
    }
    printf("\ndone.\n");
}



/**
 * Sets up a video writer for .AVI files with the given filename
 *
 * @param frameSize size of the frames
 * @param filenamePrefix prefix for the new video filename
 * @param configStruct OpenCvStereoConfig structure
 *   for reading the directory things should be saved in
 *   and other data.
 * @param increment_number Default is true.  Set to false to
 *   not increment the video number.  Used for stereo vision
 *   systems if you want to have a left and right video with
 *   the same number (ie set to false for the second writer
 *   you initialize.)
 * @param this_video_number (optional) if non-null, then the video number will be supplied
 * @param is_color true if it is color
 *  @default false
 *
 * @retval VideoWriter object
 */
VideoWriter RecordingManager::SetupVideoWriterAVI(string filenamePrefix, Size frameSize, bool increment_number, bool is_color, int *this_video_number) {

    VideoWriter recorder;

    CheckOrCreateDirectory(stereo_config_.videoSaveDir);

    string filename = GetNextVideoFilename(filenamePrefix, false, increment_number, this_video_number);

    char fourcc1 = stereo_config_.fourcc.at(0);
    char fourcc2 = stereo_config_.fourcc.at(1);
    char fourcc3 = stereo_config_.fourcc.at(2);
    char fourcc4 = stereo_config_.fourcc.at(3);

    recorder.open(filename, CV_FOURCC(fourcc1, fourcc2, fourcc3, fourcc4), 30, frameSize, is_color);
    if (!recorder.isOpened())
    {
        printf("VideoWriter failed to open!\n");
    } else {
        cout << endl << "Opened " << filename << endl;
    }

    return recorder;
}


/**
 * Sets up the system to write .pgm files into a directory named in a nice way.
 *
 * @param dirnamePrefix prefix for the save directory
 * @param configStruct OpenCvStereoConfig struct, used for configStruct.videoSaveDir
 *
 * @retval directory path to use for saving images
 *
 */
string RecordingManager::SetupVideoWriterPGM(string dirnamePrefix, bool increment_number) {

    // make or use parent directory

    CheckOrCreateDirectory(stereo_config_.videoSaveDir);

    // make a new directory for a bunch of images
    string dirpath = GetNextVideoFilename(dirnamePrefix, true, increment_number);


    if (boost::filesystem::create_directory(dirpath)) {
        cout << "Successfully created directory: " << dirpath << endl;
    } else {
        cout << "Warning: failed to create directory: " << dirpath << ", attempting to proceed with the default save directory." << endl;

        return stereo_config_.videoSaveDir;
    }
    return dirpath;
}

/**
 * Loads video files, either .avi or a directory of .pgm files.
 *
 * @param video_file_left path to directory of pgm files or .avi file for the left camera
 * @param video_file_right same as above for the right camera
 *
 * @retval true on success, false on failure.
 */
bool RecordingManager::LoadVideoFiles(string video_file_left, string video_file_right) {

    if (init_ok_ != true) {
        cerr << "Error: You must call Init() before using RecordingManager." << endl;
        return false;
    }

    using_video_from_disk_ = true;

    // determine if we are using pgm files or avi files
    if (boost::iequals(video_file_left.substr(video_file_left.length() - 4), ".avi")) {

        // using avi files

        reading_pgm_ = false;

        // attempt to create video capture objects

        if (!left_video_capture_) {
            left_video_capture_ = new VideoCapture();
        } else {
            left_video_capture_->release();
        }

        if (!right_video_capture_) {
            right_video_capture_ = new VideoCapture();
        } else {
            right_video_capture_->release();
        }

        if (left_video_capture_->open(video_file_left) != true) {
            cerr << endl << "Error: failed to open " << video_file_left
                << endl;
            return false;
        } else {
            cout << endl << endl << "Opened " << video_file_left << endl;
        }

        if (right_video_capture_->open(video_file_right) != true) {
            cerr << endl << "Error: failed to open " << video_file_right
                << endl;
            return false;
        } else {
            cout << "Opened " << video_file_right << endl;
        }

    } else {

        // using pgm directory

        reading_pgm_ = true;

        // ensure that both directories exist

        if (!boost::filesystem::exists(video_file_left)) {
            cerr << "Error: failed to find directory of PGM files: " << video_file_left << endl;

            return false;
        }

        if (!boost::filesystem::exists(video_file_right)) {
            cerr << "Error: failed to find directory of PGM files: " << video_file_right << endl;

            return false;
        }

        pgm_left_dir_ = video_file_left;
        pgm_right_dir_ = video_file_right;

    }

    return true;
}

void RecordingManager::GetFrames(Mat &left_image, Mat &right_image) {

    // check to see if there is a frame to get

    if (using_video_directory_ && current_video_number_ < 0) {

        // this is probably happening because we're waiting for
        // LCM messages to load the video

        // create an image to display that says "waiting for LCM"

        left_image = Mat::zeros(240, 376, CV_8UC1);
        right_image = Mat::zeros(240, 376, CV_8UC1);

        // put text on the maps
        putText(left_image, "Waiting for LCM messages...", Point(50,100), FONT_HERSHEY_DUPLEX, .5, Scalar(255));
        putText(right_image, "Waiting for LCM messages...", Point(50,100), FONT_HERSHEY_DUPLEX, .5, Scalar(255));

    } else {

        if (reading_pgm_) {
            GetFramePGM(left_image, right_image);
        } else {
            GetFrameAVI(left_image, right_image);
        }
    }
}

void RecordingManager::GetFramePGM(Mat &left_image, Mat &right_image) {

    // load PGM files from a directory

    // at this point, everything is already loaded up

    // get the name of the image we want based on our position in the video

    bool fail_flag = false;
    string fail_text_left = "", fail_text_right = "";

    int load_number = file_frame_number_ - file_frame_skip_;

    if (load_number < 0) {
        fail_flag = true;
        boost::format formatter_skip = boost::format("First frame: %05d (at %05d)") % file_frame_skip_ % file_frame_number_;
        fail_text_left = formatter_skip.str();
        fail_text_right = fail_text_left;

    } else {

        boost::format formatter_left = boost::format("left%05d.pgm") % load_number;
        string im_name_left = formatter_left.str();

        boost::format formatter_right = boost::format("right%05d.pgm") % load_number;
        string im_name_right = formatter_right.str();


        left_image = imread(pgm_left_dir_ + "/" + im_name_left, -1);
        right_image = imread(pgm_right_dir_ + "/" + im_name_right, -1);

        if (left_image.data == NULL) {
            fail_flag = true;
            fail_text_left = "Missing PGM file: " + im_name_left;
        }

        if (right_image.data == NULL) {
            fail_flag = true;
            fail_text_right = "Missing PGM file: " + im_name_right;
        }

    }



    if (fail_flag) {
        left_image = Mat::zeros(240, 376, CV_8UC1);
        putText(left_image, fail_text_left, Point(50,100), FONT_HERSHEY_DUPLEX, .5, Scalar(255));


        right_image = Mat::zeros(240, 376, CV_8UC1);
        putText(right_image, fail_text_right, Point(50,100), FONT_HERSHEY_DUPLEX, .5, Scalar(255));
    }


}

/**
 * Gets a frame from an AVI file.  Gets the frame at locations based
 * on the already-set file_frame_number_ and file_frame_skip_ values.
 *
 * @param left_image left image to put frame into
 * @param right_image right image to put frame into
 *
 */
void RecordingManager::GetFrameAVI(Mat &left_image, Mat &right_image) {

    Mat matL_file, matR_file;

    // make sure we don't run off the end of the video file and crash
    if (file_frame_number_ - file_frame_skip_
        >= left_video_capture_->get(CV_CAP_PROP_FRAME_COUNT)) {

        file_frame_number_ = left_video_capture_->get(CV_CAP_PROP_FRAME_COUNT) - 1 + file_frame_skip_;
    }

    // make sure we don't try to play before the file starts
    if (file_frame_number_ - file_frame_skip_ < 0) {
        file_frame_number_ = file_frame_skip_;
    }

    left_video_capture_->set(CV_CAP_PROP_POS_FRAMES, file_frame_number_ - file_frame_skip_);
    right_video_capture_->set(CV_CAP_PROP_POS_FRAMES, file_frame_number_ - file_frame_skip_);

    (*left_video_capture_) >> matL_file;
    (*right_video_capture_) >> matR_file;

    // convert from a 3 channel array to a one channel array
    cvtColor(matL_file, left_image, CV_BGR2GRAY);
    cvtColor(matR_file, right_image, CV_BGR2GRAY);
}

bool RecordingManager::SetPlaybackVideoDirectory(string video_directory) {

    if (video_directory.length() <= 0) {
        return false;
    }

    // if the directory has a trailing "/", zap it
    if (video_directory.back() == '/') {
        video_directory = video_directory.substr(0, video_directory.length() - 1);
    }

    // first, ensure the directory exists
    if (!boost::filesystem::exists(video_directory)) {
        cerr << "Warning: directory does not exist: " << video_directory << endl;
        return false;
    }

    // ok, we're happy
    video_directory_ = video_directory;

    using_video_directory_ = true;
    using_video_from_disk_ = true;

    return true;
}

void RecordingManager::SetPlaybackVideoNumber(int video_number, long long timestamp) {

    if (using_video_directory_ && video_number != current_video_number_ && video_number >= 0) {

        // load a new video file
        file_frame_skip_ = LoadVideoFileFromDir(timestamp, video_number);

        if (file_frame_skip_ >= 0) {
            current_video_number_ = video_number;
        } else {
            current_video_number_ = -1;
        }
    }
}


void RecordingManager::SetPlaybackFrameNumber(int frame_number) {

    if (frame_number >= 0) {
        file_frame_number_ = frame_number;
    }
}


string RecordingManager::GetNextVideoFilename(string filename_prefix, bool use_pgm, bool increment_number, int *this_video_number) {

    // format the number string
    char filenumber[100];


    if (video_number_ < 0) {
        int vidnum = GetNextVideoNumber(use_pgm, increment_number);

        sprintf(filenumber, "%02d", vidnum);
        if (this_video_number != NULL) {
            *this_video_number = vidnum;
        }
    } else {
        sprintf(filenumber, "%02d", video_number_);
        if (this_video_number != NULL) {
            *this_video_number = video_number_;
        }
    }

    string retstring = stereo_config_.videoSaveDir
                + "/" + filename_prefix + "-"
                + GetDateSring()
                + "." + filenumber;

    if (!use_pgm) {
        return retstring + ".avi";
    } else {
        return retstring;
    }
}


/**
 * Gets next availible filename for a video file
 *
 * @param use_pgm true if we should search for directories of PGM files, false if
 *  using AVI files.
 * @param increment_number true if we should increment video number
 *
 * @retval number of the next availible name
 *
 */
int RecordingManager::GetNextVideoNumber(bool use_pgm, bool increment_number) {

    string datechar = GetDateSring();

    int max_number = 0;

    if (boost::filesystem::exists(stereo_config_.videoSaveDir)) {

        max_number = MatchVideoFile(stereo_config_.videoSaveDir,  datechar, use_pgm);

    } else {
        cerr << "Warning: attemtped to find video files in " << stereo_config_.videoSaveDir <<
            " but that directory does not exist." << endl;
    }

    if (increment_number == true) {
        return max_number + 1;
    } else {
        return max_number;
    }
}

int RecordingManager::GetFrameNumber() {
    if (!UsingLiveCameras()) {
        return file_frame_number_;
    } else {
        return rec_num_frames_;
    }
}

void RecordingManager::SetHudNumbers(Hud *hud) {

    if (!UsingLiveCameras()) {
        hud->SetFrameNumber(file_frame_number_);
    }
    hud->SetVideoNumber(current_video_number_);
}

void RecordingManager::RecFrameHud(Mat hud_frame, bool is_color, std::string prepend_str) {
    if (!record_hud_setup_) {
        record_hud_writer_ = SetupVideoWriterAVI(prepend_str, hud_frame.size(), true, is_color, &hud_video_number_);
        record_hud_setup_ = true;
    }

    Mat write_hud;

    if (is_color) {
        hud_frame.convertTo(write_hud, CV_8UC3, 255.0);
    } else {
        write_hud = hud_frame;
    }

    record_hud_writer_ << write_hud;
}

/**
 * LoadVideoFileFromDir loads two video files from a directory, given a video
 * directory, a timestamp, and video number.
 *
 * Assigns values to left_video_capture_ and right_video_capture_ or to pgm_left_dir_
 * and pgm_right_dir_ depending on stereo_config_.usePGM's value.
 *
 * @param timestamp timestamp to extract the date out of
 * @param video_number video number to load
 *
 * @retval frame skip amount, or -1 on failure.
 *
 */
int RecordingManager::LoadVideoFileFromDir(long long timestamp, int video_number) {
    // first, load the log directory
    std::string cpu_dir, datetime;

    if (hostname_.length() == 0) {
        std::cout << "Waiting for hostname..." << std::endl;
        return -1;
    }

    std::tie(cpu_dir, datetime) = GetVideoDirectory(timestamp, video_directory_);

    std::cout << cpu_dir << std::endl;

    int skip_amount = MatchVideoFile(video_directory_ + "/" + cpu_dir + "/onboard-vids/"
        + hostname_, datetime, stereo_config_.usePGM, video_number);


    // format the video number as a two-decimal value
    char filenumber[100];
    sprintf(filenumber, "%02d", video_number);
    string video_number_str = filenumber;

    // now we have the full filename
    string left_video = video_directory_ + "/" + cpu_dir + "/onboard-vids/"
        + hostname_ + "/videoL-skip-" + to_string(skip_amount) + "-" + datetime
        + "." + video_number_str;


    string right_video = video_directory_ + "/" + cpu_dir + "/onboard-vids/"
        + hostname_ + "/videoR-skip-" + to_string(skip_amount) + "-" + datetime
        + "." + video_number_str;

    if (stereo_config_.usePGM == false) {
        left_video += ".avi";
        right_video += ".avi";
    }

    // attempt to create video capture objects
    if (LoadVideoFiles(left_video, right_video) == true) {
        printf("\nLoaded %s and %s with frame-skip: %d\n", left_video.c_str(), right_video.c_str(), skip_amount);
        return skip_amount;
    } else {
        return -1;
    }
}

/**
 * Searches a directory for video files of the form
 * video[L|R]-skip-xxx-xxxx-xx-xx.xx.avi
 * so for example, videoL-skip-0-2013-11-16.00.avi
 *
 * If you give a match_number, we will return the skip value or -1 on failure
 *
 * If you do not supply a match_number, we will return the maximum video file
 * number in the directory.
 *
 * @param directory directory to search for videos
 * @param datestr string containing date to search for (ex. 2013-11-16)
 * @param using_avi if true, match for .avi, otherwise without that (ie for a directory)
 * @param match_number (optional).  If provided, will return the skip value for
 *  that number.  Otherwise will return the largest video number in the
 *  directory.
 *
 * @retval if match_number, the skip value, else, the largest video number in
 *  the directory.
 *
 */
int RecordingManager::MatchVideoFile(string directory, string datestr, bool using_pgm, int match_number) {

    int return_number = 0;

    boost::filesystem::directory_iterator end_itr; // default construction
                                                   // yields past-the-end
    for (boost::filesystem::directory_iterator itr(directory);
        itr != end_itr; ++itr ) {

        // iterate through the videos, keeping track
        // of the highest number

        // munch on the filename to pull out
        // just the ending part if using_avi

        // 17 characters in 2013-11-21.01.avi
        // or in            xxxx-xx-xx.xx.avi

        int avi_offset;
        if (using_pgm) {
            avi_offset = -4; // everything is 4 characters less since we don't have ".avi" at the end
        } else {
            avi_offset = 0;
        }

        string this_file = itr->path().leaf().string();

        if (int(this_file.length()) > 18 + avi_offset) {

            // might be a video file

            // read the date in the string
            string file_date = this_file.substr(
                this_file.length() - (17 + avi_offset), 10);

            // compare the date string and see if it
            // is today
            if (file_date.compare(datestr) == 0) {

                // matches date
                // get the number
                string file_number = this_file.substr(
                    this_file.length() - (6 + avi_offset), 2);

                // attempt to convert the string to a number
                try {
                    int this_number = stoi(file_number);

                    if (match_number >= 0) {
                        // if we are checking for a specific file,
                        // find the skip amount in that file

                        if (match_number == this_number) {
                            // this is the file we're searching for!
                            return GetSkipNumber(this_file);
                        }

                    } else {
                        // if we are not checking for a match, determine if this
                        // is the max number
                        if (return_number < this_number) {
                            return_number = this_number;
                        }
                    }
                } catch (...) {
                    // failed to convert, don't do anything
                    // since this isn't a number or thus a video match
                }
            }
        }
    }

    return return_number;
}

/**
 * Reads a video filename and returns the skip number
 *
 * @param filename video file name in the standard video filename format
 *  (ex. videoL-skip-0-2013-11-16.00.avi)
 *
 * @retval skip amount or -1 on failure.
 *
 */
int RecordingManager::GetSkipNumber(string filename) {

    // videoL-skip- is 12 characters
    string filename2 = filename.substr(12);


    // now we have 226-2013-11-16.00.avi

    // need to match the first "-" and then we'll have our number
    int dash_pos = filename2.find("-");

    if ((unsigned int)dash_pos == string::npos) {
        // failed to find it, bail out
        return -1;
    }

    // get just the numbers
    string skipstr = filename2.substr(0, dash_pos);

    // convert to an integer
    try {
        int this_number = stoi(skipstr);

        return this_number;
    } catch (...) {
        // failed to convert, don't do anything
        // since this isn't
        return -1;
    }

    return -1;
}

/**
 * Returns the current date for use in video filenames
 *
 * @retval string of the date in "yyyy-mm-dd" format
 */
string RecordingManager::GetDateSring() {
    // get the date and time for the filename
    time_t rawtime;
    struct tm * timeinfo;
    char datechar [80];

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    strftime (datechar, 80, "%Y-%m-%d", timeinfo);

    return datechar;
}

/**
 * Checks to see if a directory exists.  If it does, returns, otherwise
 * creates the directory.
 *
 * @param dir directory path
 *
 * @retval directory path to use.  Will be the directory requested unless
 *  we failed to create that directory.
 *
 */
string RecordingManager::CheckOrCreateDirectory(string dir) {
    // check to make sure the directory exists (otherwise the
    // videowriter will seg fault)
    if (!boost::filesystem::exists(dir))
    {
        // directory does not exist, create it
        cout << "Warning: video save directory does not exist, creating it." << endl;

        if (boost::filesystem::create_directory(dir)) {
            cout << "Successfully created directory: " << dir << endl;
            return dir;
        } else {
            cout << "Warning: failed to create video save directory, attempting to proceed with the current directory." << endl;

            // note that this is only a local change to configStruct!
            return ".";
        }
    }
    return dir;
}
