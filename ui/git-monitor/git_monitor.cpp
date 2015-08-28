/*
 * Monitors local git repo and sends LCM messages about it
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */


#include "git_monitor.hpp"
#include "../../externals/ConciseArgs.hpp"

GitMonitor::GitMonitor(lcm::LCM *lcm, std::string git_status_channel) {
    lcm_ = lcm;
    git_status_channel_ = git_status_channel;
}

void GitMonitor::PublishMessage() {
    std::string sha = ExecuteProcessGetString(GetRealtimeDir() + "/scripts/getCurrentSHA");

    lcmt::git_status msg;

    msg.timestamp = GetTimestampNow();
    msg.sha = sha;

    // get stuff from the last build file
    std::ifstream last_build_stream(GetRealtimeDir() + "/lastbuild.txt");
    if (last_build_stream.is_open() == false) {
        msg.sha_last_build = "";
        msg.last_build_timestamp = 0;
        lcm_->publish(git_status_channel_, &msg);
        return;
    }

    std::string last_build_string;
    std::getline(last_build_stream, last_build_string);

    // parse file
    // it looks like:
    // SHA at last build: 113737bc481244a943bc78cb0d572395b7681a84
    // Timestmap of last build: Fri Aug 28 18:56:28 EDT 2015

    size_t pos = last_build_string.find_last_of(":");
    if (pos == std::string::npos) {
        msg.sha_last_build = "";
        msg.last_build_timestamp = 0;
        lcm_->publish(git_status_channel_, &msg);
        return;
    }

    msg.sha_last_build = last_build_string.substr(pos+2); // + 2 to remove the space

    // get the timestamp at last build
    std::getline(last_build_stream, last_build_string);

    pos = last_build_string.find_last_of(":");

    if (pos == std::string::npos) {
        msg.sha_last_build = "";
        msg.last_build_timestamp = 0;
        lcm_->publish(git_status_channel_, &msg);
        return;
    }

    std::string last_build_time_str = last_build_string.substr(pos+2); // + 2 to remove the space

    msg.last_build_timestamp = int64_t(stoi(last_build_time_str)) * 1000000;


    lcm_->publish(git_status_channel_, &msg);
}


int main(int argc,char** argv) {

    bool ttl_one = false;
    std::string git_status_channel;

    // use this computers hostname by default
    char hostname[100];
    size_t hostname_len = 100;

    gethostname(hostname, hostname_len);
    git_status_channel = "git-status-" + std::string(hostname);

    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.parse();

    std::string lcm_url;
    // create an lcm instance
    if (ttl_one) {
        lcm_url = "udpm://239.255.76.67:7667?ttl=1";
    } else {
        lcm_url = "udpm://239.255.76.67:7667?ttl=0";
    }
    lcm::LCM lcm(lcm_url);

    if (!lcm.good()) {
        std::cerr << "LCM creation failed." << std::endl;
        return 1;
    }

    printf("Sending LCM:\n\tGit Status: %s\n", git_status_channel.c_str());

    GitMonitor monitor(&lcm, git_status_channel);

    while (true) {

        monitor.PublishMessage();

        // wait 5 seconds
        sleep(5);
    }

    return 0;
}
